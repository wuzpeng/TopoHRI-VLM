#!/usr/bin/env python3
"""Geometry-only fallback target selector for VLM backend failures.

This node is deliberately not an alternative route generator.  It selects a
safe *candidate endpoint* from the same FUEL-style candidate catalog used by
the VLM.  The selected endpoint is sent through the normal validator and then
to astar_route_planner.py, which recomputes a fresh A* route from the current
robot pose and current map.
"""
from __future__ import annotations

import math
import os
import sys
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
from vlm_common import compact_json, safe_json_loads


class VLMMapFallbackPlanner:
    def __init__(self):
        self.root = rospy.get_param('/vlm_semantic_search')
        self.cfg = self.root.get('backend_fallback', {})
        self.robots = list(rospy.get_param('/vehicles', [])) + list(rospy.get_param('/ground_robots', []))
        self.robot_by_name = {str(r['name']): r for r in self.robots}
        self.lock = threading.RLock()
        self.maps: Dict[str, Optional[OccupancyGrid]] = {'uav': None, 'ugv': None}
        self.poses: Dict[str, Optional[PoseStamped]] = {str(r['name']): None for r in self.robots}
        self.overlay: Dict[str, Any] = {'version': 0, 'objects': [], 'latest_reports': []}
        self.query: Dict[str, Any] = dict(self.root.get('target_query', {}))
        self.human_context: Dict[str, Any] = {'context_version': 0, 'priority_regions': []}
        self.last_dispatch: Dict[str, Dict[str, Any]] = {}
        self.pub = rospy.Publisher('/vlm/central_plan', String, queue_size=10)

        rospy.Subscriber('/global_map_2d', OccupancyGrid, lambda msg: self._map_cb('uav', msg), queue_size=2)
        rospy.Subscriber('/ugv0/ground_map_2d', OccupancyGrid, lambda msg: self._map_cb('ugv', msg), queue_size=2)
        for robot in self.robots:
            rospy.Subscriber(robot['global_pose_topic'], PoseStamped,
                             lambda msg, n=str(robot['name']): self._pose_cb(n, msg), queue_size=20)
        rospy.Subscriber('/semantic_overlay/summary', String, self._overlay_cb, queue_size=5)
        rospy.Subscriber('/vlm/target_query', String, self._query_cb, queue_size=5)
        rospy.Subscriber('/hri/shared_context', String, self._human_context_cb, queue_size=5)
        rospy.Subscriber('/vlm/goal_dispatch', String, self._dispatch_cb, queue_size=30)
        rospy.Subscriber('/vlm/backend_fallback_request', String, self._request_cb, queue_size=10)
        rospy.loginfo('Geometry-only VLM backend fallback planner ready.')

    def _map_cb(self, key: str, msg: OccupancyGrid) -> None:
        with self.lock:
            self.maps[key] = msg

    def _pose_cb(self, name: str, msg: PoseStamped) -> None:
        with self.lock:
            self.poses[name] = msg

    def _overlay_cb(self, msg: String) -> None:
        value = safe_json_loads(msg.data, None)
        if isinstance(value, dict):
            with self.lock:
                self.overlay = value

    def _query_cb(self, msg: String) -> None:
        value = safe_json_loads(msg.data, None)
        if isinstance(value, dict):
            with self.lock:
                self.query = value

    def _human_context_cb(self, msg: String) -> None:
        value = safe_json_loads(msg.data, None)
        if isinstance(value, dict):
            with self.lock:
                self.human_context = value

    def _dispatch_cb(self, msg: String) -> None:
        value = safe_json_loads(msg.data, None)
        if not isinstance(value, dict):
            return
        robot_id = str(value.get('robot_id', ''))
        if robot_id:
            with self.lock:
                self.last_dispatch[robot_id] = value

    def _request_cb(self, msg: String) -> None:
        request = safe_json_loads(msg.data, None)
        if isinstance(request, dict):
            threading.Thread(target=self._plan, args=(request,), daemon=True).start()

    @staticmethod
    def _distance(a: Dict[str, Any], b: Dict[str, Any]) -> float:
        try:
            return math.hypot(float(a['x']) - float(b['x']), float(a['y']) - float(b['y']))
        except Exception:
            return float('inf')

    def _choose(self, robot_id: str, catalog: List[Dict[str, Any]], reason: str, failed_candidate_id: str = ''):
        last = self.last_dispatch.get(robot_id, {})
        last_goal = last.get('goal', {}) if isinstance(last, dict) else {}
        has_valid_last_goal = (
            isinstance(last_goal, dict)
            and 'x' in last_goal
            and 'y' in last_goal
        )
        min_disp = float(self.cfg.get('minimum_goal_displacement_m', 1.0))
        blocked_case = 'BLOCKED' in str(reason).upper() or 'LOCAL_' in str(reason).upper()
        options = [
            c for c in catalog
            if str(c.get('robot_id')) == robot_id
            and str(c.get('id')) != str(failed_candidate_id)
        ]
        moving_types = {'EXPLORE', 'INSPECT', 'GROUND_VERIFY', 'AERIAL_INSPECT', 'HRI_REGION_SEARCH', 'QUERY_RESCAN'}
        moving = [c for c in options if str(c.get('task_type')) in moving_types]
        if moving:
            options = moving

        def key(candidate: Dict[str, Any]):
            tier = int(
                candidate.get(
                    "priority_tier",
                    99,
                )
            )

            target_confidence = float(
                candidate.get(
                    "target_confidence",
                    0.0,
                ) or 0.0
            )

            frontier_utility = float(
                candidate.get(
                    "frontier_utility",
                    candidate.get(
                        "information_gain",
                        0.0,
                    ),
                ) or 0.0
            )

            same_id = int(
                candidate.get("id")
                == last.get("candidate_id")
            )

            too_close = int(
                bool(blocked_case)
                and has_valid_last_goal
                and self._distance(
                    candidate.get("goal", {}),
                    last_goal,
                ) < min_disp
            )

            candidate_class = str(candidate.get('candidate_class', '')).upper()
            is_hri_region = int(
                str(candidate.get('task_type', '')).upper() == 'HRI_REGION_SEARCH'
                or candidate_class in ('HRI_REGION_SEARCH', 'HRI_REGION_PERIMETER_SCAN')
            )

            return (
                too_close,
                tier,
                -is_hri_region,
                same_id,
                -target_confidence,
                -float(
                    candidate.get(
                        "human_priority_score",
                        0.0,
                    ) or 0.0
                ),
                -frontier_utility,
                float(
                    candidate.get(
                        "risk",
                        0.0,
                    ) or 0.0
                ),
                float(
                    candidate.get(
                        "path_length_m",
                        1e9,
                    ) or 1e9
                ),
            )

        options.sort(key=key)
        return options[0] if options else None

    def _plan(self, request: Dict[str, Any]) -> None:
        if not bool(self.cfg.get('enabled', True)):
            return
        with self.lock:
            maps = dict(self.maps)
            poses = dict(self.poses)
            overlay = dict(self.overlay)
            query = dict(self.query)
            human_context = dict(self.human_context)
        racer = rospy.get_param('/racer_stage3', {})
        hetero = rospy.get_param('/heterogeneous_fuel', {})
        catalog = build_candidates(self.robots, maps, poses, overlay, racer, hetero,
                                   self.root.get('candidate_builder', {}), hri_context=human_context)
        requested = request.get('participants', [])
        if not isinstance(requested, list) or not requested:
            requested = [request.get('robot_id')]
        participants = [str(x) for x in requested if str(x) in self.robot_by_name]
        if not participants and request.get('robot_id') == 'all':
            participants = [str(r['name']) for r in self.robots]
        reason = str(request.get('failure_reason', request.get('reason', 'BACKEND_FAILURE')))
        failed_candidate_id = str(request.get('failed_candidate_id', ''))
        assignments = []
        for robot_id in participants:
            candidate = self._choose(
                robot_id,
                catalog,
                reason,
                failed_candidate_id=failed_candidate_id,
            )
            if candidate is None:
                continue
            assignments.append({
                'robot_id': robot_id,
                'candidate_id': candidate['id'],
                'task_type': candidate.get('task_type', 'EXPLORE'),
                'role': 'MAP_FALLBACK',
                'reason': 'deterministic FUEL-style fallback after %s' % reason,
            })
        envelope = {
            'epoch_id': request.get('epoch_id'),
            'status': 'PLANNED',
            'query': query,
            'human_context': human_context,
            'event_reason': 'MAP_FALLBACK_%s' % reason,
            'source': 'MAP_FALLBACK',
            'plan': {
                'mission_mode': 'MAP_FALLBACK',
                'assignments': assignments,
                'reason': 'geometric endpoint fallback; A* route is computed after candidate selection',
            },
            'candidate_catalog': catalog,
        }
        self.pub.publish(compact_json(envelope))
        rospy.logwarn('Map fallback plan produced for epoch %s: %d assignment(s), reason=%s.',
                      request.get('epoch_id'), len(assignments), reason)


def main() -> None:
    rospy.init_node('vlm_map_fallback_planner')
    VLMMapFallbackPlanner()
    rospy.spin()


if __name__ == '__main__':
    main()
