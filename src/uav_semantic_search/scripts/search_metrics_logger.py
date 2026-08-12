#!/usr/bin/env python3
"""Passive experiment logger for the Human--AI VLM search stack.

The node does not publish robot goals and therefore does not change the search
method.  It records the paper-facing metrics requested for one trial:

* success (and the single-trial success-rate representation, 0 or 100 percent);
* task motion time, accumulated while at least one robot is moving,
  including motion that overlaps with an active VLM epoch;
* team route length, the sum of all robot trajectories;
* explored-map coverage when the current query target is first confirmed;
* region conflict rate (RCR) for eligible autonomous exploration pairs;
* pre-validation constraint violation rate (CVR) for raw assignments.

Target success is latched for the current query when the local VLM reports
``CONFIRMED``, or ``LIKELY`` above the semantic-priority threshold, or any
localized target evidence above the stricter completion threshold.  UAV route
length is three-dimensional; UGV route length is planar.  Coverage is the union
of known cells in the aerial and ground occupancy grids divided by the
configured rectangular map area.
"""
from __future__ import annotations

import csv
import json
import math
import os
import threading
import time
from typing import Any, Dict, List, Optional, Set, Tuple

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger, TriggerResponse


def _safe_json(text: str, default: Any) -> Any:
    try:
        return json.loads(text)
    except (TypeError, ValueError):
        return default


class SearchMetricsLogger:
    """Record one manually started Human--AI VLM search trial."""

    def __init__(self) -> None:
        self.lock = threading.RLock()
        vehicles = list(rospy.get_param('/vehicles', []))
        ground_robots = list(rospy.get_param('/ground_robots', []))
        self.robots = vehicles + ground_robots
        self.robot_types = {
            str(item['name']): str(item.get('type', 'uav')).lower()
            for item in self.robots
        }

        experiment = dict(rospy.get_param('/experiment', {}))
        self.map_id = str(experiment.get('map_id', 'unknown_map'))
        requested_run_name = str(rospy.get_param('~run_name', '')).strip()
        stamp = time.strftime('%Y%m%d_%H%M%S', time.localtime())
        self.run_name = requested_run_name or '%s_%s' % (self.map_id, stamp)

        output_root = os.path.expanduser(
            str(rospy.get_param(
                '~output_dir',
                '~/harp_sar_ws/experiment_results',
            ))
        )
        self.run_dir = os.path.join(output_root, self.run_name)
        suffix = 2
        while os.path.exists(self.run_dir):
            self.run_dir = os.path.join(
                output_root,
                '%s_%02d' % (self.run_name, suffix),
            )
            suffix += 1
        os.makedirs(self.run_dir)

        self.period = max(0.05, float(rospy.get_param('~period_sec', 0.10)))
        self.motion_speed_threshold = max(
            0.0,
            float(rospy.get_param('~motion_speed_threshold_mps', 0.03)),
        )
        # 一条位姿消息判定机器人正在运动后，将该运动状态短暂保持。
        # 该时间应略大于位姿消息的发布周期，防止两个位姿消息之间被误判为静止。
        self.motion_state_hold_sec = max(
            self.period,
            float(rospy.get_param('~motion_state_hold_sec', 0.30)),
        )
        self.min_route_step = max(
            0.0,
            float(rospy.get_param('~min_route_step_m', 0.002)),
        )
        self.max_route_step = max(
            self.min_route_step,
            float(rospy.get_param('~max_route_step_m', 1.0)),
        )
        candidate_cfg = dict(
            rospy.get_param(
                '/vlm_semantic_search/candidate_builder',
                {},
            )
        )
        self.ablation = dict(rospy.get_param(
            '/vlm_semantic_search/ablation', {}))
        self.success_confidence = float(rospy.get_param(
            '~success_confidence',
            candidate_cfg.get('confirmed_target_completion_confidence', 0.80),
        ))
        self.likely_success_confidence = float(rospy.get_param(
            '~likely_success_confidence',
            candidate_cfg.get('semantic_priority_target_confidence', 0.75),
        ))

        self.poses: Dict[str, PoseStamped] = {}

        # 每台机器人上一条已处理位姿及其墙钟接收时间
        self.previous_xyz: Dict[str, Tuple[float, float, float]] = {}
        self.previous_pose_wall: Dict[str, float] = {}

        # 该时间之前认为机器人仍处于运动状态
        self.robot_moving_until: Dict[str, float] = {
            str(item['name']): 0.0 for item in self.robots
        }

        self.route_lengths = {
            str(item['name']): 0.0 for item in self.robots
        }
        self.uav_map: Optional[OccupancyGrid] = None
        self.ground_map: Optional[OccupancyGrid] = None
        self.epoch_active = False
        self.last_sync_status = 'IDLE'
        self.query_version = 0
        self.query_id = ''

        self.created_wall = time.monotonic()
        self.started_wall: Optional[float] = None
        self.last_tick_wall = self.created_wall
        self.task_motion_time_sec = 0.0
        self.llm_query_time_sec = 0.0
        self.success = False
        self.success_reason = ''
        self.success_wall: Optional[float] = None
        self.coverage_at_success: Optional[Dict[str, float]] = None
        self.finalized = False

        # Mechanism metrics are computed immediately from the unmodified VLM
        # plan. Validator rejection, repair and fallback never enter CVR/RCR.
        self.processed_metric_epochs: Set[str] = set()
        self.rcr_conflicting_pairs = 0
        self.rcr_eligible_pairs = 0
        self.cvr_violating_decisions = 0
        self.cvr_raw_decisions = 0
        self.raw_invalid_decisions = 0
        self.cvr_reason_counts: Dict[str, int] = {}
        self.task_finished_pub = rospy.Publisher(
            '/experiment/task_finished',
            String,
            queue_size=1,
            latch=True,
        )

        self.time_series_path = os.path.join(self.run_dir, 'time_series.csv')
        self.time_series_file = open(
            self.time_series_path,
            'w',
            newline='',
        )
        self.time_series_writer = csv.writer(self.time_series_file)
        self.time_series_writer.writerow([
            'wall_elapsed_sec',
            'task_motion_time_sec',
            'llm_epoch_active',
            'llm_query_time_sec',
            'any_robot_moving',
            'team_route_length_m',
            'explored_area_m2',
            'map_total_area_m2',
            'map_coverage_percent',
            'success',
            'query_version',
            'sync_status',
        ] + [
            '%s_route_length_m' % name for name in self.route_lengths
        ])
        self.time_series_file.flush()

        self.decision_metrics_path = os.path.join(
            self.run_dir, 'decision_metrics.csv'
        )
        self.decision_metrics_file = open(
            self.decision_metrics_path, 'w', newline='', encoding='utf-8'
        )
        self.decision_metrics_writer = csv.writer(self.decision_metrics_file)
        self.decision_metrics_writer.writerow([
            'epoch_id', 'event_reason', 'source',
            'raw_assignment_count', 'constraint_violation_count',
            'cvr_evaluable_assignment_count', 'raw_invalid_assignment_count',
            'constraint_violation_reasons',
            'rcr_eligible_pair_count', 'rcr_conflicting_pair_count',
        ])
        self.decision_metrics_file.flush()

        rospy.Subscriber(
            '/global_map_2d',
            OccupancyGrid,
            self._uav_map_cb,
            queue_size=1,
        )
        rospy.Subscriber(
            '/ugv0/ground_map_2d',
            OccupancyGrid,
            self._ground_map_cb,
            queue_size=1,
        )
        rospy.Subscriber(
            '/vlm/epoch_active',
            Bool,
            self._epoch_cb,
            queue_size=10,
        )
        rospy.Subscriber(
            '/vlm/sync_status',
            String,
            self._sync_status_cb,
            queue_size=10,
        )
        rospy.Subscriber(
            '/vlm/target_query',
            String,
            self._query_cb,
            queue_size=5,
        )
        rospy.Subscriber(
            '/semantic_overlay/summary',
            String,
            self._semantic_summary_cb,
            queue_size=10,
        )
        rospy.Subscriber(
            '/vlm/central_plan', String, self._central_plan_cb, queue_size=20
        )
        for robot in self.robots:
            name = str(robot['name'])
            topic = str(
                robot.get('global_pose_topic', '/%s/global_pose' % name)
            )
            rospy.Subscriber(
                topic,
                PoseStamped,
                lambda msg, rid=name: self._pose_cb(rid, msg),
                queue_size=30,
            )

        self.finish_service = rospy.Service(
            '/experiment_metrics/finish',
            Trigger,
            self._finish_service_cb,
        )
        rospy.Timer(rospy.Duration(self.period), self._tick)
        rospy.on_shutdown(self._on_shutdown)
        rospy.loginfo(
            'Human-AI search metrics logger ready: %s',
            self.run_dir,
        )

    def _uav_map_cb(self, msg: OccupancyGrid) -> None:
        with self.lock:
            self.uav_map = msg

    def _ground_map_cb(self, msg: OccupancyGrid) -> None:
        with self.lock:
            self.ground_map = msg

    def _epoch_cb(self, msg: Bool) -> None:
        with self.lock:
            self.epoch_active = bool(msg.data)

    def _sync_status_cb(self, msg: String) -> None:
        with self.lock:
            self.last_sync_status = str(msg.data)

    def _query_cb(self, msg: String) -> None:
        query = _safe_json(msg.data, {})
        if not isinstance(query, dict):
            return
        with self.lock:
            self.query_version = int(
                query.get('query_version', self.query_version)
            )
            self.query_id = str(query.get('query_id', self.query_id))

    @staticmethod
    def _epoch_id(payload: Dict[str, Any]) -> str:
        value = payload.get('epoch_id')
        return str(value) if value is not None else ''

    def _central_plan_cb(self, msg: String) -> None:
        payload = _safe_json(msg.data, {})
        if not isinstance(payload, dict):
            return
        epoch_id = self._epoch_id(payload)
        if not epoch_id:
            return
        with self.lock:
            if self.finalized or epoch_id in self.processed_metric_epochs:
                return
            self._record_raw_plan_metrics(payload)

    @staticmethod
    def _candidate_region(candidate: Dict[str, Any]) -> Optional[Tuple[str, str]]:
        if str(candidate.get('task_type', '')).upper() != 'EXPLORE':
            return None
        region = candidate.get('topology_region_id')
        confidence = str(candidate.get('topology_confidence', 'LOW')).upper()
        unique_unassigned = bool(region and ':UNASSIGNED:F' in str(region))
        if not region or (confidence not in ('HIGH', 'MEDIUM')
                          and not unique_unassigned):
            return None
        return (
            str(candidate.get('topology_layer', 'topology')),
            str(region),
        )

    @staticmethod
    def _is_human_directed(candidate: Dict[str, Any]) -> bool:
        if str(candidate.get('source', '')).upper() in ('HUMAN', 'HRI'):
            return True
        if str(candidate.get('task_type', '')).upper() == 'HUMAN':
            return True
        if candidate.get('human_priority_regions'):
            return True
        return bool(float(candidate.get('human_priority_score', 0.0) or 0.0) > 0.0)

    @classmethod
    def _robot_explore_regions(
        cls, catalog: List[Dict[str, Any]], robot_id: str,
    ) -> Set[Tuple[str, str]]:
        return {
            region for candidate in catalog
            if isinstance(candidate, dict)
            and str(candidate.get('robot_id', '')) == robot_id
            and not cls._is_human_directed(candidate)
            for region in [cls._candidate_region(candidate)]
            if region is not None
        }

    @classmethod
    def _rcr_counts(
        cls,
        assignments: List[Dict[str, Any]],
        catalog_by_id: Dict[str, Dict[str, Any]],
        catalog: List[Dict[str, Any]],
    ) -> Tuple[int, int]:
        selected = []
        for item in assignments:
            if not isinstance(item, dict):
                continue
            candidate = catalog_by_id.get(str(item.get('candidate_id', '')))
            if not isinstance(candidate, dict) or cls._is_human_directed(candidate):
                continue
            region = cls._candidate_region(candidate)
            if region is not None:
                selected.append((str(item.get('robot_id', '')), region))

        eligible = 0
        conflicts = 0
        for index, (robot_i, selected_i) in enumerate(selected):
            regions_i = cls._robot_explore_regions(catalog, robot_i)
            for robot_j, selected_j in selected[index + 1:]:
                if robot_i == robot_j:
                    continue
                regions_j = cls._robot_explore_regions(catalog, robot_j)
                # A pair is evaluable only when a conflict-free regional
                # assignment was available to that pair at this epoch.
                has_diverse_choice = any(
                    region_i != region_j
                    for region_i in regions_i for region_j in regions_j
                )
                if not has_diverse_choice:
                    continue
                eligible += 1
                conflicts += int(selected_i == selected_j)
        return conflicts, eligible

    @staticmethod
    def _raw_assignment_count(plan_envelope: Dict[str, Any]) -> int:
        plan = plan_envelope.get('plan', {})
        assignments = plan.get('assignments', []) if isinstance(plan, dict) else []
        return sum(isinstance(item, dict) for item in assignments) \
            if isinstance(assignments, list) else 0

    @staticmethod
    def _action_priority(candidate: Dict[str, Any]) -> int:
        if int(candidate.get('priority_tier', 99)) == 0:
            return 0
        task = str(candidate.get('task_type', '')).upper()
        candidate_class = str(candidate.get('candidate_class', '')).upper()
        if task in ('EXPLORE', 'HRI_REGION_SEARCH') or candidate_class == 'FRONTIER':
            return 1
        if task == 'QUERY_RESCAN' or 'RESCAN' in candidate_class:
            return 2
        if task in ('INSPECT', 'GROUND_VERIFY', 'AERIAL_INSPECT'):
            return 3
        if task in ('HOVER_AND_SCAN', 'SCAN_IN_PLACE', 'HOLD'):
            return 4
        return 50

    @staticmethod
    def _candidate_query_version(candidate: Dict[str, Any]) -> Optional[int]:
        for source in (candidate, candidate.get('semantic_anchor', {})):
            if isinstance(source, dict) and source.get('query_version') is not None:
                try:
                    return int(source.get('query_version'))
                except (TypeError, ValueError):
                    return None
        return None

    @classmethod
    def _constraint_reasons(
        cls, assignment: Dict[str, Any], candidate: Dict[str, Any],
        catalog: List[Dict[str, Any]], current_query_version: int,
    ) -> List[str]:
        reasons: List[str] = []
        robot_id = str(assignment.get('robot_id', ''))
        if str(candidate.get('robot_id', '')) != robot_id:
            reasons.append('candidate_robot_mismatch')
            return reasons

        task = str(candidate.get('task_type', '')).upper()
        candidate_version = cls._candidate_query_version(candidate)
        if (task in ('INSPECT', 'GROUND_VERIFY', 'AERIAL_INSPECT')
                and candidate_version is not None
                and candidate_version != current_query_version):
            reasons.append('stale_target_verification_after_query_switch')

        robot_options = [
            option for option in catalog
            if str(option.get('robot_id', '')) == robot_id
        ]
        if robot_options:
            best_priority = min(cls._action_priority(option) for option in robot_options)
            if cls._action_priority(candidate) > best_priority:
                reasons.append('lower_priority_action_while_better_option_available')

        confirmed = any(
            int(option.get('priority_tier', 99)) == 0
            and (
                str(option.get('target_state', '')).upper() == 'CONFIRMED'
                or float(option.get('target_confidence', 0.0) or 0.0) >= 0.80
            )
            for option in robot_options
        )
        if confirmed and task in (
                'EXPLORE', 'QUERY_RESCAN', 'INSPECT', 'GROUND_VERIFY',
                'AERIAL_INSPECT', 'HRI_REGION_SEARCH'):
            reasons.append('search_after_confirmed_target')
        return sorted(set(reasons))

    def _record_raw_plan_metrics(self, envelope: Dict[str, Any]) -> None:
        epoch_id = self._epoch_id(envelope)
        catalog = [item for item in envelope.get('candidate_catalog', [])
                   if isinstance(item, dict)]
        catalog_by_id = {str(item['id']): item for item in catalog if item.get('id')}
        plan = envelope.get('plan', {})
        assignments = [item for item in plan.get('assignments', [])
                       if isinstance(item, dict)] if isinstance(plan, dict) else []
        query = envelope.get('query', {})
        query_version = int(query.get('query_version', self.query_version)) \
            if isinstance(query, dict) else self.query_version

        evaluable: List[Dict[str, Any]] = []
        reason_counts: Dict[str, int] = {}
        violation_count = 0
        invalid_count = 0
        for assignment in assignments:
            robot_id = str(assignment.get('robot_id', ''))
            candidate = catalog_by_id.get(str(assignment.get('candidate_id', '')))
            if candidate is None or robot_id not in self.robot_types:
                invalid_count += 1
                continue
            evaluable.append(assignment)
            reasons = self._constraint_reasons(
                assignment, candidate, catalog, query_version)
            if reasons:
                violation_count += 1
                for reason in reasons:
                    reason_counts[reason] = reason_counts.get(reason, 0) + 1

        conflicts, eligible = self._rcr_counts(
            evaluable, catalog_by_id, catalog)
        raw_count = len(assignments)
        self.cvr_raw_decisions += len(evaluable)
        self.cvr_violating_decisions += violation_count
        self.raw_invalid_decisions += invalid_count
        for reason, count in reason_counts.items():
            self.cvr_reason_counts[reason] = (
                self.cvr_reason_counts.get(reason, 0) + count
            )
        self.rcr_conflicting_pairs += conflicts
        self.rcr_eligible_pairs += eligible
        self.processed_metric_epochs.add(epoch_id)
        self.decision_metrics_writer.writerow([
            epoch_id,
            str(envelope.get('event_reason', '')),
            str(envelope.get('source', 'VLM')),
            raw_count,
            violation_count,
            len(evaluable),
            invalid_count,
            json.dumps(reason_counts, ensure_ascii=False, sort_keys=True),
            eligible,
            conflicts,
        ])
        self.decision_metrics_file.flush()

    def _pose_cb(self, robot: str, msg: PoseStamped) -> None:
        now_wall = time.monotonic()

        with self.lock:
            if self.finalized:
                return

            self.poses[robot] = msg

            p = msg.pose.position
            xyz = (float(p.x), float(p.y), float(p.z))

            previous = self.previous_xyz.get(robot)
            previous_wall = self.previous_pose_wall.get(robot)

            # 每条新位姿消息只处理一次
            self.previous_xyz[robot] = xyz
            self.previous_pose_wall[robot] = now_wall

            if previous is None or previous_wall is None:
                return

            pose_dt = now_wall - previous_wall
            if pose_dt <= 0.0:
                return

            dx = xyz[0] - previous[0]
            dy = xyz[1] - previous[1]

            if self.robot_types.get(robot) == 'ugv':
                # UGV只计算平面运动
                distance = math.hypot(dx, dy)
            else:
                # UAV计算三维运动
                dz = xyz[2] - previous[2]
                distance = math.sqrt(
                    dx * dx + dy * dy + dz * dz
                )

            # 超过最大步长通常表示定位跳变，不计入轨迹
            if distance > self.max_route_step:
                rospy.logwarn_throttle(
                    5.0,
                    '%s pose jump ignored: %.3f m over %.3f s'
                    % (robot, distance, pose_dt),
                )
                self.robot_moving_until[robot] = now_wall
                return

            # 小于最小步长，认为是定位抖动或静止
            if distance < self.min_route_step:
                self.robot_moving_until[robot] = now_wall
                return

            # 有效轨迹长度在位姿回调中累计
            self.route_lengths[robot] += distance

            speed = distance / pose_dt

            if speed >= self.motion_speed_threshold:
                self.robot_moving_until[robot] = (
                    now_wall + self.motion_state_hold_sec
                )
            else:
                self.robot_moving_until[robot] = now_wall

    def _semantic_summary_cb(self, msg: String) -> None:
        summary = _safe_json(msg.data, {})
        if not isinstance(summary, dict):
            return
        with self.lock:
            if self.finalized or self.success:
                return
            summary_query = summary.get('query', {})
            if isinstance(summary_query, dict):
                summary_version = int(
                    summary_query.get('query_version', self.query_version)
                )
                if summary_version != self.query_version:
                    return

            for obj in summary.get('objects', []):
                if not isinstance(obj, dict):
                    continue
                if str(obj.get('label', '')) != 'target_candidate':
                    continue
                object_version = int(
                    obj.get('query_version', self.query_version)
                )
                if object_version != self.query_version:
                    continue
                state = str(obj.get('target_state', '')).upper()
                confidence = float(
                    obj.get(
                        'target_confidence',
                        obj.get('confidence', 0.0),
                    ) or 0.0
                )
                confirmed_state = state == 'CONFIRMED'
                likely_confirmed = (
                    state == 'LIKELY'
                    and confidence >= self.likely_success_confidence
                )
                high_confidence_confirmed = (
                    confidence >= self.success_confidence
                )
                if confirmed_state or likely_confirmed or high_confidence_confirmed:
                    if confirmed_state:
                        reason = 'state=CONFIRMED'
                    elif likely_confirmed:
                        reason = (
                            'state=LIKELY,confidence=%.3f>=%.3f'
                            % (confidence, self.likely_success_confidence)
                        )
                    else:
                        reason = (
                            'confidence=%.3f>=%.3f'
                            % (confidence, self.success_confidence)
                        )
                    self._mark_success_locked(reason, obj)
                    return

    def _publish_task_finished_locked(
        self,
        finish_reason: str,
        evidence: Optional[Dict[str, Any]] = None,
    ) -> None:
        message = {
            'finished': True,
            'success': bool(self.success),
            'finish_reason': str(finish_reason),
            'success_reason': str(self.success_reason),
            'query_id': self.query_id,
            'query_version': int(self.query_version),
            'evidence': evidence if isinstance(evidence, dict) else {},
            'stamp_sec': rospy.Time.now().to_sec(),
        }
        self.task_finished_pub.publish(
            json.dumps(message, ensure_ascii=False, separators=(',', ':'))
        )

    def _mark_success_locked(
        self,
        reason: str,
        evidence: Dict[str, Any],
    ) -> None:
        if self.finalized or self.success:
            return
        self.success = True
        self.success_wall = time.monotonic()
        self.success_reason = str(reason)
        self.coverage_at_success = self._coverage_locked()
        self._publish_task_finished_locked('target_confirmed', evidence)
        rospy.logwarn(
            'Target search succeeded for query v%d (%s); task stop latched.',
            self.query_version,
            self.success_reason,
        )
        # Finalize outside the subscriber's call stack so that all accepted
        # values are already visible to the periodic loop.
        threading.Thread(
            target=self._finalize,
            args=('target_confirmed',),
            daemon=True,
        ).start()

    @staticmethod
    def _map_signature(
        msg: OccupancyGrid,
    ) -> Tuple[int, int, float, float, float]:
        info = msg.info
        return (
            int(info.width),
            int(info.height),
            float(info.resolution),
            float(info.origin.position.x),
            float(info.origin.position.y),
        )

    def _coverage_locked(self) -> Dict[str, float]:
        maps = [
            msg for msg in (self.uav_map, self.ground_map)
            if msg is not None and msg.data
        ]
        if not maps:
            return {
                'explored_cells': 0,
                'total_cells': 0,
                'explored_area_m2': 0.0,
                'map_total_area_m2': 0.0,
                'coverage_percent': 0.0,
            }

        reference = maps[0]
        signature = self._map_signature(reference)
        known = np.asarray(reference.data, dtype=np.int16) >= 0
        for msg in maps[1:]:
            if self._map_signature(msg) != signature:
                rospy.logwarn_throttle(
                    10.0,
                    'Cannot union UAV/UGV coverage: OccupancyGrid metadata differ.',
                )
                continue
            known = np.logical_or(
                known,
                np.asarray(msg.data, dtype=np.int16) >= 0,
            )
        explored_cells = int(np.count_nonzero(known))
        total_cells = int(known.size)
        resolution = float(reference.info.resolution)
        cell_area = resolution * resolution
        return {
            'explored_cells': explored_cells,
            'total_cells': total_cells,
            'explored_area_m2': explored_cells * cell_area,
            'map_total_area_m2': total_cells * cell_area,
            'coverage_percent': (
                100.0 * explored_cells / float(total_cells)
                if total_cells else 0.0
            ),
        }

    def _sample_motion_locked(self, now_wall: float) -> bool:
        dt = max(0.0, now_wall - self.last_tick_wall)

        # 团队运动时间取所有机器人运动时间的并集：
        # 只要至少一台机器人正在运动，团队就处于运动状态。
        any_moving = any(
            now_wall <= moving_until
            for moving_until in self.robot_moving_until.values()
        )

        if any_moving and self.started_wall is None:
            self.started_wall = now_wall

        if self.started_wall is not None:
            # VLM查询时间独立累计
            if self.epoch_active:
                self.llm_query_time_sec += dt

            # 运动时间也独立累计。
            # 即使VLM查询正在进行，只要机器人运动，就必须累计。
            if any_moving:
                self.task_motion_time_sec += dt

        self.last_tick_wall = now_wall
        return any_moving

    def _tick(self, _event: Any) -> None:
        with self.lock:
            if self.finalized:
                return
            now_wall = time.monotonic()
            any_moving = self._sample_motion_locked(now_wall)
            coverage = self._coverage_locked()
            wall_elapsed = (
                now_wall - self.started_wall
                if self.started_wall is not None else 0.0
            )
            row = [
                wall_elapsed,
                self.task_motion_time_sec,
                int(self.epoch_active),
                self.llm_query_time_sec,
                int(any_moving),
                sum(self.route_lengths.values()),
                coverage['explored_area_m2'],
                coverage['map_total_area_m2'],
                coverage['coverage_percent'],
                int(self.success),
                self.query_version,
                self.last_sync_status,
            ] + [
                self.route_lengths[name] for name in self.route_lengths
            ]
            self.time_series_writer.writerow(row)
            self.time_series_file.flush()

    def _summary_locked(self, finish_reason: str) -> Dict[str, Any]:
        final_coverage = self._coverage_locked()
        success_coverage = self.coverage_at_success
        finish_wall = self.success_wall or time.monotonic()
        wall_elapsed = (
            finish_wall - self.started_wall
            if self.started_wall is not None else 0.0
        )
        return {
            'run_name': self.run_name,
            'map_id': self.map_id,
            'query_id': self.query_id,
            'query_version': self.query_version,
            'finish_reason': finish_reason,
            'success': int(self.success),
            'success_rate_percent': 100.0 if self.success else 0.0,
            'task_motion_time_sec': self.task_motion_time_sec,
            'team_route_length_m': sum(self.route_lengths.values()),
            'coverage_at_success_percent': (
                success_coverage['coverage_percent']
                if success_coverage is not None else None
            ),
            'explored_area_at_success_m2': (
                success_coverage['explored_area_m2']
                if success_coverage is not None else None
            ),
            'map_total_area_m2': final_coverage['map_total_area_m2'],
            'per_robot_route_length_m': dict(self.route_lengths),
            'llm_query_time_sec': self.llm_query_time_sec,
            'wall_elapsed_from_first_motion_sec': wall_elapsed,
            'final_coverage_percent': final_coverage['coverage_percent'],
            'success_reason': self.success_reason,
            'enable_topology_planning': bool(
                self.ablation.get('enable_topology_planning', True)),
            'enable_state_action_gate': bool(
                self.ablation.get('enable_state_action_gate', True)),
            'region_conflict_rate_percent': (
                100.0 * self.rcr_conflicting_pairs / self.rcr_eligible_pairs
                if self.rcr_eligible_pairs else None
            ),
            'rcr_conflicting_pairs': self.rcr_conflicting_pairs,
            'rcr_eligible_pairs': self.rcr_eligible_pairs,
            'constraint_violation_rate_percent': (
                100.0 * self.cvr_violating_decisions / self.cvr_raw_decisions
                if self.cvr_raw_decisions else None
            ),
            'cvr_violating_decisions': self.cvr_violating_decisions,
            'cvr_raw_decisions': self.cvr_raw_decisions,
            'raw_invalid_decisions': self.raw_invalid_decisions,
            'cvr_reason_counts': dict(self.cvr_reason_counts),
            'decision_metric_epochs': len(self.processed_metric_epochs),
            'metric_definitions': {
                'task_motion_time_sec': (
                    'Union wall-clock duration in which at least one robot '
                    'is moving. Motion time is accumulated independently '
                    'of overlapping VLM decision epochs.'
                ),
                'team_route_length_m': (
                    'Sum of 3-D UAV trajectories and planar UGV trajectories.'
                ),
                'coverage_at_success_percent': (
                    'Union of known UAV/UGV occupancy cells divided by all '
                    'configured map cells, sampled at first target confirmation.'
                ),
                'region_conflict_rate_percent': (
                    'Conflicting raw-VLM EXPLORE robot pairs divided by eligible '
                    'raw autonomous EXPLORE pairs. A pair is eligible only when '
                    'both selected candidates have topology labels and a '
                    'conflict-free regional assignment existed in the same '
                    'candidate catalog. Human-directed candidates are excluded.'
                ),
                'constraint_violation_rate_percent': (
                    'Evaluable raw-VLM assignments violating the deterministic '
                    'state-action rules divided by all evaluable raw assignments. '
                    'Validator outcomes and invalid IDs are excluded.'
                ),
            },
        }

    def _finalize(self, finish_reason: str) -> None:
        with self.lock:
            if self.finalized:
                return
            # Capture the last sub-period pose increment before freezing the
            # summary. This keeps the success-trigger sample consistent with
            # the 0.1 s periodic trajectory integration.
            self._sample_motion_locked(time.monotonic())
            self.finalized = True
            summary = self._summary_locked(finish_reason)
            summary_json = os.path.join(self.run_dir, 'summary.json')
            with open(summary_json, 'w', encoding='utf-8') as stream:
                json.dump(summary, stream, ensure_ascii=False, indent=2)
                stream.write('\n')

            summary_csv = os.path.join(self.run_dir, 'summary.csv')
            fields = [
                'run_name',
                'map_id',
                'success',
                'success_rate_percent',
                'task_motion_time_sec',
                'team_route_length_m',
                'coverage_at_success_percent',
                'explored_area_at_success_m2',
                'map_total_area_m2',
                'llm_query_time_sec',
                'wall_elapsed_from_first_motion_sec',
                'final_coverage_percent',
                'region_conflict_rate_percent',
                'rcr_conflicting_pairs',
                'rcr_eligible_pairs',
                'constraint_violation_rate_percent',
                'cvr_violating_decisions',
                'cvr_raw_decisions',
                'raw_invalid_decisions',
                'enable_topology_planning',
                'enable_state_action_gate',
                'decision_metric_epochs',
                'finish_reason',
                'success_reason',
            ]
            with open(summary_csv, 'w', newline='', encoding='utf-8') as stream:
                writer = csv.DictWriter(stream, fieldnames=fields)
                writer.writeheader()
                writer.writerow({key: summary.get(key) for key in fields})
            try:
                self.time_series_file.flush()
                self.time_series_file.close()
            except Exception:
                pass
            try:
                self.decision_metrics_file.flush()
                self.decision_metrics_file.close()
            except Exception:
                pass
            rospy.loginfo(
                'Experiment metrics finalized: success=%d, motion_time=%.3fs, '
                'team_route=%.3fm, coverage_at_success=%s, RCR=%s, CVR=%s, dir=%s',
                summary['success'],
                summary['task_motion_time_sec'],
                summary['team_route_length_m'],
                (
                    '%.3f%%' % summary['coverage_at_success_percent']
                    if summary['coverage_at_success_percent'] is not None
                    else 'N/A'
                ),
                (
                    '%.3f%%' % summary['region_conflict_rate_percent']
                    if summary['region_conflict_rate_percent'] is not None
                    else 'N/A'
                ),
                (
                    '%.3f%%' % summary['constraint_violation_rate_percent']
                    if summary['constraint_violation_rate_percent'] is not None
                    else 'N/A'
                ),
                self.run_dir,
            )

    def _finish_service_cb(self, _request: Any) -> TriggerResponse:
        with self.lock:
            if not self.finalized:
                self._publish_task_finished_locked('manual_finish_service')
        self._finalize('manual_finish_service')
        return TriggerResponse(
            success=True,
            message='Metrics finalized in %s' % self.run_dir,
        )

    def _on_shutdown(self) -> None:
        self._finalize(
            'target_confirmed' if self.success else 'ros_shutdown_before_success'
        )


def main() -> None:
    rospy.init_node('search_metrics_logger')
    SearchMetricsLogger()
    rospy.spin()


if __name__ == '__main__':
    main()
