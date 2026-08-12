#!/usr/bin/env python3
"""Centralized VLM task organization and safe-candidate selection for Stage-5."""
from __future__ import annotations

import os
import sys
import time
import threading
from typing import Any, Dict, List, Optional

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String

_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if _SCRIPT_DIR not in sys.path:
    sys.path.insert(0, _SCRIPT_DIR)

from vlm_candidate_builder import build_candidates
from vlm_common import (OpenAICompatibleVisionClient, compact_json, grid_from_msg, map_summary,
                        mock_central_plan, pose_to_dict, safe_json_loads)


CENTRAL_SYSTEM_PROMPT = """You are the centralized high-level planner for heterogeneous indoor search.

You receive safe, geometrically validated candidate waypoints for UAVs and UGVs.
Return ONLY one valid JSON object.

You must never invent a coordinate or candidate ID.
Assign at most one candidate per robot.
Use only candidate IDs that belong to the assigned robot.

Use semantic reports, the fused semantic overlay, target query, robot positions,
candidate geometry, information_gain, risk, and path_length_m to coordinate
the whole team.

TEAM EXPLORATION POLICY:

1. In EXPLORE mode, maximize team-level unknown-space coverage rather than
   minimizing the travel distance of an individual robot.

2. For EXPLORE candidates, prioritize larger information_gain and lower risk.
   Treat path_length_m as a secondary cost only. Do not choose a frontier merely
   because it is the closest candidate.

3. Avoid repeatedly assigning very near frontiers when a farther candidate
   reveals a larger unexplored region or advances exploration into a new area.

4. When both UAVs have feasible EXPLORE candidates, assign spatially separated
   goals whenever possible. Avoid sending both UAVs toward the same corridor
   segment, room entrance, or nearby frontier cluster.

5. Prefer complementary roles:
   UAVs should cover distinct aerial exploration directions; the UGV should
   inspect or verify regions that benefit from ground-level sensing.

6. Do NOT prioritize INSPECT or GROUND_VERIFY over EXPLORE unless the candidate
   has target_confidence >= 0.75. Ordinary scene objects with low or zero
   target_confidence should not interrupt high-value frontier exploration.

7. Use HOVER_AND_SCAN, SCAN_IN_PLACE, or HOLD only when that robot has no safe
   moving candidate, or when a high-confidence semantic verification reason
   explicitly requires stationary observation.

CANDIDATE PRIORITY POLICY:

Candidates include priority_tier:
0 = suspected target candidate;
1 = exploration frontier;
2 = ordinary semantic object inspection;
3 = scan or hold.

Follow this priority strictly:

1. If a valid tier-0 suspected target exists, assign exactly one suitable robot
   to verify the highest-confidence target. Prefer UGV GROUND_VERIFY when
   available. Do not send multiple robots to the same semantic_anchor unless
   cross-view verification is explicitly necessary.

2. After target verification has been assigned, allocate remaining robots to
   tier-1 EXPLORE frontiers. Prefer larger frontier_utility, lower risk, and
   spatially separated frontier goals.

3. Never select tier-2 ordinary INSPECT candidates while the same robot has a
   feasible tier-0 target candidate or tier-1 EXPLORE candidate.

4. Select tier-2 INSPECT only when target candidates are absent and no feasible
   exploration frontier remains for that robot.

5. Select tier-3 scan or hold only when no safe moving candidate is available.

Schema:
{
  "mission_mode":"EXPLORE|VERIFY|INSPECT|OVERWATCH",
  "assignments":[
    {
      "robot_id":"...",
      "role":"AERIAL_SCOUT|GROUND_SCOUT|GROUND_VERIFY|OVERWATCH|ACTIVE_SCAN|HOLD",
      "candidate_id":"must exist and belong to robot_id",
      "task_type":"must match candidate",
      "reason":"short"
    }
  ],
  "plan_valid_for_sec":12,
  "reason":"short"
}
"""


class CentralVLMPlanner:
    def __init__(self):
        self.root = rospy.get_param('/vlm_semantic_search')
        self.lock = threading.RLock()
        self.robots = list(rospy.get_param('/vehicles', [])) + list(rospy.get_param('/ground_robots', []))
        self.maps: Dict[str, Optional[OccupancyGrid]] = {'uav': None, 'ugv': None}

        self.poses: Dict[str, Optional[PoseStamped]] = {
            r['name']: None for r in self.robots
        }

        self.execution_status: Dict[str, str] = {
            r['name']: 'UNKNOWN' for r in self.robots
        }

        self.last_dispatch: Dict[str, Dict[str, Any]] = {
            r['name']: {} for r in self.robots
        }

        self.overlay: Dict[str, Any] = {
            'version': 0,
            'objects': [],
            'latest_reports': []
        }

        self.query: Dict[str, Any] = dict(self.root.get('target_query', {}))
        self.client = OpenAICompatibleVisionClient(self.root['backend'])
        self.pub = rospy.Publisher('/vlm/central_plan', String, queue_size=5)

        rospy.Subscriber('/global_map_2d', OccupancyGrid, lambda msg: self._map_cb('uav', msg), queue_size=2)
        rospy.Subscriber('/ugv0/ground_map_2d', OccupancyGrid, lambda msg: self._map_cb('ugv', msg), queue_size=2)

        for robot in self.robots:
            rospy.Subscriber(
                robot['global_pose_topic'],
                PoseStamped,
                lambda msg, n=robot['name']: self._pose_cb(n, msg),
                queue_size=20
            )

            rospy.Subscriber(
                '/%s/mission/status' % robot['name'],
                String,
                lambda msg, n=robot['name']: self._status_cb(n, msg),
                queue_size=20
            )

        rospy.Subscriber(
            '/vlm/goal_dispatch',
            String,
            self._dispatch_cb,
            queue_size=30
        )

        rospy.Subscriber('/semantic_overlay/summary', String, self._overlay_cb, queue_size=5)
        rospy.Subscriber('/vlm/target_query', String, self._query_cb, queue_size=5)
        rospy.Subscriber('/vlm/central_plan_request', String, self._request_cb, queue_size=10)

    def _map_cb(self, key, msg):
        with self.lock:
            self.maps[key] = msg

    def _pose_cb(self, name, msg):
        with self.lock:
            self.poses[name] = msg

    def _status_cb(self, name, msg):
        with self.lock:
            self.execution_status[name] = str(msg.data)

    def _dispatch_cb(self, msg):
        value = safe_json_loads(msg.data, None)
        if not isinstance(value, dict):
            return

        robot_id = str(value.get('robot_id', ''))
        if robot_id not in self.last_dispatch:
            return

        with self.lock:
            self.last_dispatch[robot_id] = value

    def _overlay_cb(self, msg):
        parsed = safe_json_loads(msg.data, None)
        if isinstance(parsed, dict):
            with self.lock:
                self.overlay = parsed

    def _query_cb(self, msg):
        parsed = safe_json_loads(msg.data, None)
        if isinstance(parsed, dict):
            with self.lock:
                self.query = parsed

    def _request_cb(self, msg):
        request = safe_json_loads(msg.data, None)
        if not isinstance(request, dict):
            return
        threading.Thread(target=self._plan, args=(request,), daemon=True).start()

    def _prompt(self, request: Dict[str, Any], candidates: List[Dict[str, Any]], robots: List[Dict[str, Any]],
                overlay: Dict[str, Any], query: Dict[str, Any]) -> str:
        compact_candidates = []
        for c in candidates:
            compact_candidates.append({
                'id': c['id'],
                'robot_id': c['robot_id'],
                'robot_type': c['robot_type'],
                'task_type': c['task_type'],
                'goal': c['goal'],

                'path_length_m': c.get('path_length_m'),
                'information_gain': c.get('information_gain'),
                'risk': c.get('risk'),

                'semantic_anchor': c.get('semantic_anchor'),

                # 让 Central VLM 能区分“高置信目标验证”
                # 和“仅仅是普通场景物体”。
                'target_confidence': c.get('target_confidence', 0.0),
                'semantic_confidence': c.get('semantic_confidence', 0.0),

                'priority_tier': c.get('priority_tier'),
                'candidate_class': c.get('candidate_class'),

                'target_confidence': c.get('target_confidence', 0.0),
                'target_state': c.get('target_state', 'NONE'),

                'frontier_utility': c.get('frontier_utility', 0.0),
                'frontier_length_m': c.get('frontier_length_m', 0.0),
            })

        return compact_json({
            'event': {
                'epoch_id': request.get('epoch_id'),
                'reason': request.get('reason'),
                'details': request.get('details', {})},
            'planning_scope': request.get(
                'planning_scope',
                'GLOBAL_JOINT_REALLOCATION'),
            'planning_robots': request.get(
                'planning_robots',
                [robot.get('id') for robot in robots]),
            'target_query': query,
            'robots': robots,
            'semantic_overlay': overlay,
            'candidate_catalog': compact_candidates,
            'instruction': (
                'Select feasible candidate IDs only. Maximize team-level unknown-space '
                'coverage, prioritize high information_gain and low risk EXPLORE frontiers, '
                'avoid short-distance-only choices, spatially separate UAV exploration '
                'goals, and prioritize INSPECT or GROUND_VERIFY only when '
                'target_confidence >= 0.75. Do not output raw coordinates.'),
        })

    @staticmethod
    def _merge_epoch_reports(overlay: Dict[str, Any], reports: Any) -> Dict[str, Any]:
        """Make just-finished local reports visible to this planning epoch.

        Gazebo is paused during a synchronous epoch, so ROS simulated-time timers
        do not advance.  This merge avoids depending on the overlay publisher's
        periodic timer before the central planner sees fresh semantic evidence.
        """
        merged = dict(overlay)
        objects = list(merged.get('objects', []))
        latest = {str(x.get('robot_id', 'unknown')): x for x in merged.get('latest_reports', [])
                  if isinstance(x, dict)}
        if not isinstance(reports, dict):
            return merged
        for rid, report in reports.items():
            if not isinstance(report, dict):
                continue
            latest[str(rid)] = {
                'robot_id': str(rid),
                'scene_summary': report.get('scene_summary', ''),
                'target_evidence': report.get('target_evidence', {}),
                'requested_follow_up': report.get('requested_follow_up', 'none'),
                'query_version': report.get('query_version', 0),
            }
            for entity in report.get('entities', []) if isinstance(report.get('entities', []), list) else []:
                if not isinstance(entity, dict) or not entity.get('position_map'):
                    continue
                obj = dict(entity)
                obj['observed_by'] = [str(rid)]
                obj['last_epoch_id'] = report.get('epoch_id')
                obj['target_confidence'] = 0.0
                objects.append(obj)
            target = report.get('target_evidence', {})
            if isinstance(target, dict) and target.get('position_map') and float(target.get('confidence', 0.0)) > 0.0:
                objects.append({
                    'object_id': 'epoch_target_%s_%s' % (report.get('epoch_id', ''), rid),
                    'label': 'target_candidate',
                    'category': 'query_target',
                    'confidence': float(target.get('confidence', 0.0)),
                    'target_confidence': float(target.get('confidence', 0.0)),
                    'position_map': target.get('position_map'),
                    'observed_by': [str(rid)],
                    'last_epoch_id': report.get('epoch_id'),
                })
        merged['objects'] = objects[-40:]
        merged['latest_reports'] = list(latest.values())
        merged['epoch_local_reports'] = reports
        return merged

    # def _call(self, prompt: str, candidates: List[Dict[str, Any]], robots: List[Dict[str, Any]],
    #           query: Dict[str, Any], overlay: Dict[str, Any]) -> Dict[str, Any]:
    #     mode = str(self.root['backend'].get('mode', 'mock')).lower()
    #     if mode == 'mock':
    #         return mock_central_plan(candidates, robots, query, overlay)
    #     try:
    #         return self.client.complete_json(CENTRAL_SYSTEM_PROMPT, prompt, None)
    #     except Exception as exc:
    #         if bool(self.root['backend'].get('allow_mock_fallback', True)):
    #             out = mock_central_plan(candidates, robots, query, overlay)
    #             out['backend_error'] = repr(exc)
    #             return out
    #         raise
    def _call(self, prompt: str, candidates: List[Dict[str, Any]],
            robots: List[Dict[str, Any]], query: Dict[str, Any],
            overlay: Dict[str, Any]) -> Dict[str, Any]:
        """Call Central VLM with bounded retry and exponential backoff."""

        mode = str(self.root['backend'].get('mode', 'mock')).lower()

        # Mock 模式不需要重试。
        if mode == 'mock':
            return mock_central_plan(candidates, robots, query, overlay)

        retry_cfg = self.root.get('central_retry', {})

        enabled = bool(retry_cfg.get('enabled', True))
        max_retries = int(retry_cfg.get('max_retries', 2)) if enabled else 0

        # 总尝试次数 = 1 次初始调用 + max_retries 次重试。
        max_attempts = 1 + max(0, max_retries)

        attempt_timeout = float(
            retry_cfg.get(
                'attempt_timeout_sec',
                self.root['backend'].get('timeout_sec', 25.0)
            )
        )

        total_deadline = float(
            retry_cfg.get(
                'total_deadline_sec',
                attempt_timeout * max_attempts + 5.0
            )
        )

        initial_backoff = float(retry_cfg.get('initial_backoff_sec', 1.0))
        backoff_multiplier = float(retry_cfg.get('backoff_multiplier', 2.0))

        deadline = time.monotonic() + total_deadline
        last_error = None

        for attempt_index in range(max_attempts):
            attempt_no = attempt_index + 1
            remaining = deadline - time.monotonic()

            if remaining <= 0.0:
                break

            # 当前尝试的 timeout 不允许超过总 deadline 剩余时间。
            current_timeout = max(0.1, min(attempt_timeout, remaining))

            try:
                response = self.client.complete_json(
                    CENTRAL_SYSTEM_PROMPT,
                    prompt,
                    None,
                    timeout_sec=current_timeout,
                )

                rospy.loginfo(
                    'Central VLM succeeded on attempt %d/%d.',
                    attempt_no,
                    max_attempts,
                )
                return response

            except Exception as exc:
                last_error = exc
                error_text = str(exc)

                # 400/401/403 通常是配置或鉴权问题，重试没有意义。
                non_retryable = (
                    'VLM HTTP 400' in error_text or
                    'VLM HTTP 401' in error_text or
                    'VLM HTTP 403' in error_text
                )

                if non_retryable or attempt_no >= max_attempts:
                    break

                backoff = initial_backoff * (
                    backoff_multiplier ** attempt_index
                )

                remaining_after_error = deadline - time.monotonic()
                if remaining_after_error <= 0.0:
                    break

                sleep_time = min(backoff, remaining_after_error)

                rospy.logwarn(
                    'Central VLM attempt %d/%d failed: %r. '
                    'Retrying in %.1f s.',
                    attempt_no,
                    max_attempts,
                    exc,
                    sleep_time,
                )

                time.sleep(sleep_time)

        # 所有重试均失败后，保持原有 fallback 行为。
        if bool(self.root['backend'].get('allow_mock_fallback', True)):
            out = mock_central_plan(candidates, robots, query, overlay)
            out['backend_error'] = repr(last_error)
            return out

        raise RuntimeError(
            'Central VLM failed after %d attempt(s): %r'
            % (max_attempts, last_error)
        )

    def _plan(self, request: Dict[str, Any]):
        with self.lock:
            maps = dict(self.maps)
            poses = dict(self.poses)
            execution_status = dict(self.execution_status)
            last_dispatch = {
                robot_id: dict(value)
                for robot_id, value in self.last_dispatch.items()
            }
            overlay = dict(self.overlay)
            query = dict(self.query)
        overlay = self._merge_epoch_reports(overlay, request.get('local_reports', {}))
        racer = rospy.get_param('/racer_stage3', {})
        heterogeneous = rospy.get_param('/heterogeneous_fuel', {})
        candidates = build_candidates(self.robots, maps, poses, overlay, racer, heterogeneous,
                                      self.root['candidate_builder'])
        
        robot_context = [{
            'id': r['name'],
            'type': r.get('type'),
            'pose_map': pose_to_dict(poses.get(r['name'])),
            'map_layer': (
                'ugv_ground'
                if r.get('type') == 'ugv'
                else 'uav_flight'
            ),
            # 当前执行状态，例如 tracking_astar_route、route_reached_hold。
            'execution_status': execution_status.get(r['name'], 'UNKNOWN'),
            # 当前执行任务、旧 candidate、旧目标点。
            'current_assignment': last_dispatch.get(r['name'], {}),
        } for r in self.robots]

        prompt = self._prompt(request, candidates, robot_context, overlay, query)
        try:
            response = self._call(prompt, candidates, robot_context, query, overlay)
            status = 'PLANNED'
        except Exception as exc:
            # Always publish an envelope. The coordinator can then close the epoch
            # without waiting for an additional central timeout, and the validator
            # preserves the robots' previous commands instead of issuing fallback
            # motion based on a failed semantic decision.
            response = {
                'mission_mode': 'HOLD', 'assignments': [],
                'reason': 'central VLM backend unavailable', 'backend_error': repr(exc),
            }
            status = 'BACKEND_ERROR'
        plan = {
            'epoch_id': request.get('epoch_id'),
            'status': status,
            'query': query,
            'event_reason': request.get('reason'),
            'plan': response,
            'candidate_catalog': candidates,
            'robot_context': robot_context,
            'map_summary': {'uav': map_summary(grid_from_msg(maps.get('uav'))), 'ugv': map_summary(grid_from_msg(maps.get('ugv')))},
        }
        self.pub.publish(compact_json(plan))
        if status == 'PLANNED':
            rospy.loginfo('Central VLM plan produced for epoch %s with %d candidates.', request.get('epoch_id'), len(candidates))
        else:
            rospy.logwarn('Central VLM backend failed for epoch %s: %s', request.get('epoch_id'), response.get('backend_error'))


def main():
    rospy.init_node('central_vlm_planner')
    CentralVLMPlanner()
    rospy.spin()


if __name__ == '__main__':
    main()
