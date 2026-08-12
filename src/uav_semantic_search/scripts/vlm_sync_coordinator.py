#!/usr/bin/env python3
"""Synchronous VLM coordinator with cloud-safe local inference and deterministic recovery.

Cloud VLM endpoints often throttle or serialize image requests.  This
coordinator therefore supports sequential local VLM dispatch for multi-robot
initialization and target-query updates.  Gazebo/PX4 remain running while the
remote calls are in progress; only the high-level task epoch is frozen.
"""
from __future__ import annotations

import os
import sys
import threading
import time
from typing import Any, Dict, List, Optional

import rospy
from std_srvs.srv import Empty
from std_msgs.msg import Bool, String

_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if _SCRIPT_DIR not in sys.path:
    sys.path.insert(0, _SCRIPT_DIR)

from vlm_common import compact_json, now_wall, safe_json_loads


class SyncCoordinator:
    def __init__(self):
        self.root = rospy.get_param('/vlm_semantic_search')
        self.robots = list(rospy.get_param('/vehicles', [])) + list(rospy.get_param('/ground_robots', []))
        self.all_robot_ids = [str(robot['name']) for robot in self.robots]
        self.lock = threading.RLock()
        self.active = False
        self.current_epoch: Optional[str] = None
        # 当前正在处理的完整 trigger event。
        self.current_event: Optional[Dict[str, Any]] = None
        # 高优先级待处理事件队列。
        self.pending_events: List[Dict[str, Any]] = []
        self.local_reports: Dict[str, Dict[str, Any]] = {}
        self.plan_result: Optional[Dict[str, Any]] = None
        self.task_finished = False
        self.epoch_pub = rospy.Publisher('/vlm/epoch_active', Bool, queue_size=2, latch=True)
        self.status_pub = rospy.Publisher('/vlm/sync_status', String, queue_size=5, latch=True)
        self.local_req_pub = rospy.Publisher('/vlm/local_perception_request', String, queue_size=5)
        self.central_req_pub = rospy.Publisher('/vlm/central_plan_request', String, queue_size=5)
        self.fallback_req_pub = rospy.Publisher('/vlm/backend_fallback_request', String, queue_size=10)
        self.color_hold_pubs = {}
        for robot in self.robots:
            name = robot['name']
            self.color_hold_pubs[name] = rospy.Publisher(
                '/%s/vlm/observation_hold_command' % name,
                String,
                queue_size=5,)
        rospy.Subscriber('/vlm/trigger_event', String, self._trigger_cb, queue_size=10)
        rospy.Subscriber('/vlm/local_semantic_observation', String, self._local_cb, queue_size=30)
        rospy.Subscriber('/vlm/validated_plan', String, self._validation_cb, queue_size=10)
        rospy.Subscriber('/experiment/task_finished', String, self._task_finished_cb, queue_size=1)
        self.pause_srv = rospy.ServiceProxy(str(self.root.get('pause_service', '/gazebo/pause_physics')), Empty)
        self.unpause_srv = rospy.ServiceProxy(str(self.root.get('unpause_service', '/gazebo/unpause_physics')), Empty)
        self.epoch_pub.publish(False)
        self.status_pub.publish('IDLE')

    def _task_finished_cb(self, msg: String) -> None:
        event = safe_json_loads(msg.data, None)
        if not isinstance(event, dict) or not bool(event.get('finished', False)):
            return
        with self.lock:
            self.task_finished = True
            self.pending_events = []
        # Keep physics and MAVROS timers running so UAV executors can maintain
        # their terminal hover setpoints.
        self._unpause()
        self.epoch_pub.publish(False)
        self.status_pub.publish('TASK_FINISHED')
    
    @staticmethod
    def _is_color_event(event: Dict[str, Any]) -> bool:
        return str(
            event.get('reason', '')
        ).startswith('COLOR_CANDIDATE_')


    def _event_priority(
            self,
            event: Dict[str, Any]
    ) -> int:
        """Smaller value means higher queue priority."""

        reason = str(
            event.get('reason', '')
        )

        if reason == 'ROBOT_BLOCKED':
            return 0

        if reason.startswith(
            'COLOR_CANDIDATE_'
        ):
            return 1

        if reason == 'TARGET_QUERY_CHANGE':
            return 2

        if reason.startswith('HRI_'):
            return 2

        if reason == 'GOAL_REACHED':
            return 3

        return 10


    def _queue_key(
            self,
            event: Dict[str, Any]
    ):
        """Only coalesce repeated color events from the same robot/query."""

        if not self._is_color_event(event):
            return None

        return (
            'COLOR',
            str(event.get('robot_id', '')),
            int(event.get('query_version', 0)),
        )


    def _enqueue_event_locked(
            self,
            event: Dict[str, Any]
    ) -> None:
        """Queue an event while another epoch is active.

        For the same robot and query version, newer color snapshots replace an old
        queued color snapshot. This keeps the most recent valid observation while
        avoiding repeated duplicate COLOR_CANDIDATE_RED epochs.
        """

        key = self._queue_key(event)

        if key is not None:
            retained = []

            for old in self.pending_events:
                if self._queue_key(old) != key:
                    retained.append(old)

            self.pending_events = retained

        self.pending_events.append(
            dict(event)
        )

        self.pending_events.sort(
            key=lambda item: (
                self._event_priority(item),
                float(
                    item.get(
                        'trigger_wall_time',
                        now_wall(),
                    )
                ),
            )
        )

        max_pending = max(
            1,
            int(
                self.root.get(
                    'scheduler',
                    {}
                ).get(
                    'event_queue',
                    {}
                ).get(
                    'max_pending_events',
                    12,
                )
            ),
        )

        if len(self.pending_events) > max_pending:
            dropped = self.pending_events.pop()

            rospy.logwarn(
                'VLM event queue full; dropped %s from %s.',
                dropped.get('reason'),
                dropped.get('robot_id'),
            )


    def _start_event_locked(
            self,
            event: Dict[str, Any]
    ) -> None:
        """Activate one event. Caller must hold self.lock."""

        self.active = True
        self.current_epoch = event.get('epoch_id')
        self.current_event = dict(event)
        self.local_reports = {}
        self.plan_result = None


    def _pop_next_event_locked(
            self
    ) -> Optional[Dict[str, Any]]:
        if not self.pending_events:
            return None

        return self.pending_events.pop(0)


    def _release_color_hold_after_delay(
            self,
            event: Dict[str, Any],
            finish_status: str
    ) -> None:
        """Release only the hold belonging to the completed color epoch."""

        if not self._is_color_event(event):
            return

        details = event.get('details', {})

        if not isinstance(details, dict):
            return

        candidate = details.get('color_candidate', {})

        if not isinstance(candidate, dict):
            candidate = {}

        robot_id = str(
            event.get('robot_id', '')
        )

        hold_id = str(
            details.get(
                'hold_id',
                candidate.get(
                    'snapshot_id',
                    '',
                ),
            )
        )

        pub = self.color_hold_pubs.get(robot_id)

        if pub is None or not hold_id:
            return

        delay = float(
            self.root.get(
                'scheduler',
                {}
            ).get(
                'color_candidate_trigger',
                {}
            ).get(
                'release_delay_sec',
                1.0,
            )
        )

        def _release():
            time.sleep(max(0.0, delay))

            pub.publish(
                compact_json({
                    'robot_id': robot_id,
                    'active': False,
                    'hold_id': hold_id,
                    'reason': 'COLOR_EPOCH_FINISHED',
                    'finish_status': finish_status,
                })
            )

            rospy.loginfo(
                'Released color observation hold for %s id=%s.',
                robot_id,
                hold_id,
            )

        threading.Thread(
            target=_release,
            daemon=True,
        ).start()

    def _trigger_cb(self, msg):
        event = safe_json_loads(msg.data, None)

        if not isinstance(event, dict):
            return

        start_event = None
        queued = False

        with self.lock:
            if self.task_finished:
                self.status_pub.publish('TASK_FINISHED')
                return
            if self.active:
                self._enqueue_event_locked(event)
                queued = True
            else:
                self._start_event_locked(event)
                start_event = dict(event)

        if queued:
            rospy.loginfo(
                'Queued VLM event %s from %s while epoch %s is active.',
                event.get('reason'),
                event.get('robot_id'),
                self.current_epoch,
            )

            self.status_pub.publish(
                'QUEUED:%s:%s' % (
                    event.get('reason'),
                    event.get('robot_id'),
                )
            )
            return

        threading.Thread(
            target=self._run_epoch,
            args=(start_event,),
            daemon=True,
        ).start()

    def _local_cb(self, msg):
        report = safe_json_loads(msg.data, None)
        if not isinstance(report, dict):
            return
        with self.lock:
            if not self.active or report.get('epoch_id') != self.current_epoch:
                return
            rid = str(report.get('robot_id', ''))
            if rid:
                self.local_reports[rid] = report

    def _validation_cb(self, msg):
        result = safe_json_loads(msg.data, None)
        if not isinstance(result, dict):
            return
        with self.lock:
            if not self.active or result.get('epoch_id') != self.current_epoch:
                return
            self.plan_result = result

    def _participants(self, event: Dict[str, Any]) -> List[str]:
        planner_cfg = self.root.get('central_planner', {})
        reason = str(event.get('reason', ''))
        if reason == 'TARGET_QUERY_CHANGE' and str(planner_cfg.get('query_change_participants', 'all')) == 'all':
            return [r['name'] for r in self.robots]
        if reason == 'HRI_PRIORITY_REGION_UPDATE' and not bool(
                event.get('details', {}).get('refresh_local_perception', False)
                if isinstance(event.get('details', {}), dict) else False):
            # Region-only updates change allocation preference, not visual evidence.
            return []
        if event.get('robot_id') == 'all':
            return [r['name'] for r in self.robots]
        return [str(event.get('robot_id'))]

    def _ordered_participants(self, participants: List[str]) -> List[str]:
        """Return participants in deterministic cloud-safe request order."""
        dispatch_cfg = self.root.get('local_dispatch', {})
        configured = [str(x) for x in dispatch_cfg.get('participant_order', [])]
        ordered = [name for name in configured if name in participants]
        ordered.extend(name for name in participants if name not in ordered)
        return ordered

    def _pause(self):
        if not bool(self.root.get('pause_gazebo_during_epoch', False)):
            return
        try:
            rospy.wait_for_service(str(self.root.get('pause_service', '/gazebo/pause_physics')), timeout=2.0)
            self.pause_srv()
        except Exception as exc:
            rospy.logwarn('Cannot pause Gazebo for VLM epoch: %r', exc)

    def _unpause(self):
        if not bool(self.root.get('pause_gazebo_during_epoch', False)):
            return
        try:
            rospy.wait_for_service(str(self.root.get('unpause_service', '/gazebo/unpause_physics')), timeout=2.0)
            self.unpause_srv()
        except Exception as exc:
            rospy.logwarn('Cannot unpause Gazebo after VLM epoch: %r', exc)

    @staticmethod
    def _wait_for(predicate, timeout_sec: float) -> bool:
        deadline = now_wall() + float(timeout_sec)
        while now_wall() < deadline and not rospy.is_shutdown():
            if predicate():
                return True
            time.sleep(0.03)
        return predicate()

    def _collect_local_reports(self, event: Dict[str, Any], participants: List[str]) -> Dict[str, Dict[str, Any]]:
        """Request local VLM reports in sequential or legacy-parallel mode.

        Sequential dispatch is the default for cloud endpoints.  A request is
        sent to exactly one robot and the coordinator waits for that robot's
        report before proceeding, preventing image-request queueing at a
        single API key/model endpoint.
        """
        epoch = event.get('epoch_id')
        if not participants:
            return {}
        cfg = self.root.get('local_dispatch', {})
        mode = str(cfg.get('mode', 'sequential')).lower()
        ordered = self._ordered_participants(participants)
        per_timeout = float(cfg.get(
            'per_robot_response_timeout_sec',
            self.root.get('local_response_timeout_sec', 22.0)))
        delay = max(0.0, float(cfg.get('inter_request_delay_sec', 0.0)))

        if mode == 'parallel' or len(ordered) <= 1:
            self.local_req_pub.publish(compact_json({
                'epoch_id': epoch,
                'reason': event.get('reason'),
                'details': event.get('details', {}),
                'participants': ordered,
                'query': event.get('query', {}),
                'dispatch_mode': 'parallel',
            }))
            self._wait_for(lambda: all(p in self.local_reports for p in ordered), per_timeout)
        else:
            for index, participant in enumerate(ordered):
                self.status_pub.publish('LOCAL_PERCEPTION:%s:%s:%d/%d' % (
                    epoch, participant, index + 1, len(ordered)))
                rospy.loginfo('VLM epoch %s dispatches local perception serially to %s (%d/%d).',
                              epoch, participant, index + 1, len(ordered))
                self.local_req_pub.publish(compact_json({
                    'epoch_id': epoch,
                    'reason': event.get('reason'),
                    'details': event.get('details', {}),
                    'participants': [participant],
                    'query': event.get('query', {}),
                    'dispatch_mode': 'sequential',
                    'dispatch_index': index,
                    'dispatch_total': len(ordered),
                }))
                got = self._wait_for(lambda p=participant: p in self.local_reports, per_timeout)
                if not got:
                    rospy.logwarn('VLM epoch %s local perception timed out for %s after %.1fs.',
                                  epoch, participant, per_timeout)
                if delay > 0.0 and index + 1 < len(ordered):
                    time.sleep(delay)

        with self.lock:
            return dict(self.local_reports)

    def _request_map_fallback(self, event: Dict[str, Any], reports: Dict[str, Dict[str, Any]],
                              participants: List[str], failure_reason: str) -> bool:
        """Ask the deterministic fallback selector for new candidate endpoints.

        The fallback selector does not publish a route. It emits a normal
        endpoint-only plan which then goes through the validator and the
        post-selection A* route planner.
        """
        if not bool(self.root.get('backend_fallback', {}).get('enabled', True)):
            return False
        epoch = event.get('epoch_id')
        with self.lock:
            self.plan_result = None
        self.status_pub.publish('MAP_FALLBACK:%s:%s' % (failure_reason, epoch))
        self.fallback_req_pub.publish(compact_json({
            'epoch_id': epoch,
            'reason': event.get('reason'),
            'failure_reason': failure_reason,
            'details': event.get('details', {}),
            'participants': participants,
            'local_reports': reports,
        }))
        timeout = float(self.root.get('backend_fallback', {}).get('response_timeout_sec', 12.0)) + \
                  float(self.root.get('validator_timeout_sec', 6.0))
        return self._wait_for(lambda: self.plan_result is not None, timeout)

    def _finish(self, status: str):
        self._unpause()

        finished_event = None
        next_event = None

        with self.lock:
            finished_event = (
                dict(self.current_event)
                if isinstance(
                    self.current_event,
                    dict,
                )
                else None
            )

            self.active = False
            self.current_epoch = None
            self.current_event = None
            self.local_reports = {}
            self.plan_result = None

            next_event = (
                None
                if self.task_finished
                else self._pop_next_event_locked()
            )

            if next_event is not None:
                self._start_event_locked(next_event)

        self.epoch_pub.publish(False)
        self.status_pub.publish(status)

        # 颜色事件的路径或 fallback 已完成生成后，才解除观察保持。
        if finished_event is not None:
            self._release_color_hold_after_delay(
                finished_event,
                status,
            )

        # 颜色事件通常具有最高优先级，会在当前 epoch 完成后立即启动。
        if next_event is not None:
            rospy.loginfo(
                'Starting queued VLM event %s from %s.',
                next_event.get('reason'),
                next_event.get('robot_id'),
            )

            threading.Thread(
                target=self._run_epoch,
                args=(next_event,),
                daemon=True,
            ).start()

    @staticmethod
    def _is_local_failure(report: Dict[str, Any]) -> bool:
        status = str(report.get('status', 'OK')).upper()
        return status != 'OK' or bool(report.get('backend_error'))

    def _run_epoch(self, event: Dict[str, Any]):
        epoch = event.get('epoch_id')

        # local VLM 只更新触发机器人的新语义信息；
        # Central VLM 始终对全部机器人进行联合目标分配。
        local_participants = self._participants(event)
        planning_robots = list(self.all_robot_ids)

        pause_enabled = bool(self.root.get('pause_gazebo_during_epoch', False))

        self.epoch_pub.publish(True)
        self.status_pub.publish(('PAUSE_AND_SNAPSHOT:' if pause_enabled else 'SNAPSHOT_NO_PHYSICS_PAUSE:') + str(epoch))
        self._pause()
        try:
            reports = self._collect_local_reports(event, local_participants)
            missing = [p for p in local_participants if p not in reports]
            failed = [
                p for p in local_participants
                if p in reports and self._is_local_failure(reports[p])
                ]
            if missing:
                rospy.logwarn('VLM epoch %s local perception timeout; missing %s.', epoch, missing)
            if failed:
                rospy.logwarn('VLM epoch %s local perception returned failures from %s.', epoch, failed)
            if ((missing and bool(self.root.get('skip_central_on_local_timeout', True))) or
                    (failed and bool(self.root.get('skip_central_on_local_failure', True)))):
                reason = 'LOCAL_TIMEOUT' if missing else 'LOCAL_BACKEND_FAILURE'
                if self._request_map_fallback(event, reports, planning_robots, reason):
                    with self.lock:
                        result = dict(self.plan_result or {})
                    self._finish('MAP_FALLBACK_RESUMED:%s:%s' % (epoch, result.get('status', 'UNKNOWN')))
                else:
                    self._finish('%s_RESUMED:%s' % (reason, epoch))
                return
            if missing:
                rospy.logwarn('VLM epoch %s continues with partial local reports.', epoch)
            self.status_pub.publish('CENTRAL_PLANNING:%s' % epoch)
            self.central_req_pub.publish(compact_json({
                'epoch_id': epoch,
                'reason': event.get('reason'),
                'details': event.get('details', {}),

                # 触发本次语义更新的机器人。
                'local_participants': local_participants,

                # Central VLM 必须始终对三台机器人联合输出任务。
                'planning_robots': planning_robots,
                'planning_scope': 'GLOBAL_JOINT_REALLOCATION',
                # Preserve the exact query snapshot used by Local VLM.
                'query': event.get('query', {}),

                'local_reports': reports,
            }))
            plan_timeout = float(self.root.get('central_response_timeout_sec', 22.0)) + float(self.root.get('validator_timeout_sec', 6.0))
            ok = self._wait_for(lambda: self.plan_result is not None, plan_timeout)
            if not ok:
                if self._request_map_fallback(event, reports, planning_robots, 'CENTRAL_TIMEOUT'):
                    with self.lock:
                        result = dict(self.plan_result or {})
                    self._finish('MAP_FALLBACK_RESUMED:%s:%s' % (epoch, result.get('status', 'UNKNOWN')))
                else:
                    self._finish('CENTRAL_TIMEOUT_RESUMED:%s' % epoch)
                return
            with self.lock:
                result = dict(self.plan_result or {})
            if str(result.get('status', '')).upper() == 'SKIPPED_BACKEND_ERROR':
                if self._request_map_fallback(event, reports, planning_robots, 'CENTRAL_BACKEND_FAILURE'):
                    with self.lock:
                        result = dict(self.plan_result or {})
                    self._finish('MAP_FALLBACK_RESUMED:%s:%s' % (epoch, result.get('status', 'UNKNOWN')))
                else:
                    self._finish('CENTRAL_BACKEND_ERROR_RESUMED:%s' % epoch)
                return
            self._finish('RESUMED:%s:%s' % (epoch, result.get('status', 'UNKNOWN')))
        except Exception as exc:
            rospy.logerr('VLM epoch %s failed: %r', epoch, exc)
            self._finish('FAILED_RESUMED:%s' % epoch)


def main():
    rospy.init_node('vlm_sync_coordinator')
    SyncCoordinator()
    rospy.spin()


if __name__ == '__main__':
    main()
