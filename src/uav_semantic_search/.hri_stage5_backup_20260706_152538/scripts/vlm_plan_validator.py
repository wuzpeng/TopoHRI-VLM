#!/usr/bin/env python3
"""Validate endpoint-only VLM selections and request fresh A* route planning.

The VLM selects a safe candidate ID.  This validator verifies the candidate
against the current geometry, then asks ``astar_route_planner.py`` to compute a
new route from the current robot pose.  It never forwards a VLM path and it
never permits a VLM to inject arbitrary coordinates.
"""
from __future__ import annotations

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

from frontier_core import astar, build_passable_mask, nearest_passable
from vlm_common import compact_json, grid_from_msg, safe_json_loads


_MOVING_TASKS = {'EXPLORE', 'INSPECT', 'GROUND_VERIFY', 'AERIAL_INSPECT'}


class VLMPlanValidator:
    def __init__(self):
        self.root = rospy.get_param('/vlm_semantic_search')
        self.cfg = self.root.get('validator', {})
        self.robots = list(rospy.get_param('/vehicles', [])) + list(rospy.get_param('/ground_robots', []))
        self.robot_by_name = {str(r['name']): r for r in self.robots}
        self.lock = threading.RLock()
        self.poses: Dict[str, Optional[PoseStamped]] = {str(r['name']): None for r in self.robots}
        self.maps: Dict[str, Optional[OccupancyGrid]] = {'uav': None, 'ugv': None}
        self.pub = rospy.Publisher('/vlm/validated_plan', String, queue_size=5)
        self.dispatch_pub = rospy.Publisher('/vlm/goal_dispatch', String, queue_size=20)
        self.route_req_pub = rospy.Publisher(
            str(self.root.get('route_planner', {}).get('request_topic', '/vlm/route_request')),
            String, queue_size=20)
        for robot in self.robots:
            rospy.Subscriber(robot['global_pose_topic'], PoseStamped,
                             lambda msg, n=str(robot['name']): self._pose_cb(n, msg), queue_size=20)
        rospy.Subscriber('/global_map_2d', OccupancyGrid, lambda msg: self._map_cb('uav', msg), queue_size=2)
        rospy.Subscriber('/ugv0/ground_map_2d', OccupancyGrid, lambda msg: self._map_cb('ugv', msg), queue_size=2)
        rospy.Subscriber('/vlm/central_plan', String, self._plan_cb, queue_size=10)
        rospy.loginfo('VLM plan validator ready: validates endpoints, then requests fresh A* routes.')

    def _pose_cb(self, name: str, msg: PoseStamped) -> None:
        with self.lock:
            self.poses[name] = msg

    def _map_cb(self, key: str, msg: OccupancyGrid) -> None:
        with self.lock:
            self.maps[key] = msg

    def _plan_cb(self, msg: String) -> None:
        plan = safe_json_loads(msg.data, None)
        if isinstance(plan, dict):
            threading.Thread(target=self._validate, args=(plan,), daemon=True).start()

    @staticmethod
    def _task_priority(candidate: Dict[str, Any]) -> int:
        """Use candidate tier instead of raw task type."""

        if "priority_tier" in candidate:
            return int(
                candidate.get("priority_tier", 99,))

        # Backward-compatible fallback for old candidate catalogs.
        task = str(candidate.get("task_type", "",))

        if task == "EXPLORE":
            return 1

        if task == "GROUND_VERIFY":
            return 0

        if task in (
            "INSPECT",
            "AERIAL_INSPECT",
        ):
            return 2

        if task in (
            "HOVER_AND_SCAN",
            "SCAN_IN_PLACE",
        ):
            return 3

        return 99

    def _candidate_safe_now(self, robot: Dict[str, Any], candidate: Dict[str, Any]) -> bool:
        if not bool(self.cfg.get('recheck_astar', True)):
            return True
        map_key = 'ugv' if robot.get('type') == 'ugv' else 'uav'
        with self.lock:
            map_msg = self.maps.get(map_key)
            pose = self.poses.get(str(robot['name']))
        grid = grid_from_msg(map_msg)
        if grid is None or pose is None:
            return False
        racer = rospy.get_param('/racer_stage3', {})
        hetero = rospy.get_param('/heterogeneous_fuel', {})
        occupied = int(racer.get('occupied_threshold', 65))
        if robot.get('type') == 'ugv':
            inflation = float(hetero.get('ugv_obstacle_inflation_m', 0.54))
            near = int(hetero.get('ugv_nearest_free_search_cells', 20))
            max_exp = int(hetero.get('ugv_astar_max_expansions', 30000))
        else:
            inflation = float(racer.get('obstacle_inflation_m', 0.55))
            near = int(racer.get('nearest_free_search_cells', 25))
            max_exp = int(racer.get('astar_max_expansions', 30000))
        passable, _ = build_passable_mask(grid, occupied, inflation)
        start = grid.world_to_cell(pose.pose.position.x, pose.pose.position.y)
        goal = grid.world_to_cell(float(candidate['goal']['x']), float(candidate['goal']['y']))
        if start is None or goal is None:
            return False
        start = nearest_passable(passable, start, near)
        goal = nearest_passable(passable, goal, near)
        return start is not None and goal is not None and astar(passable, start, goal, max_exp) is not None

    def _fallback_for(self, catalog: List[Dict[str, Any]], robot_id: str,
                      moving_only: bool = False) -> Optional[Dict[str, Any]]:
        robot = self.robot_by_name.get(robot_id)
        if robot is None:
            return None
        options = [c for c in catalog if str(c.get('robot_id')) == robot_id]
        if moving_only:
            moving = [c for c in options if str(c.get('task_type')) in _MOVING_TASKS]
            if moving:
                options = moving
        options.sort(key=lambda c: (self._task_priority(c),
                                    -float(c.get('information_gain', 0.0)),
                                    float(c.get('risk', 0.0)),
                                    float(c.get('path_length_m', 1e9))))
        for candidate in options:
            if self._candidate_safe_now(robot, candidate):
                return candidate
        return None
    
    def _has_higher_priority_option(
            self,
            catalog: List[Dict[str, Any]],
            robot_id: str,
            candidate: Dict[str, Any]
    ) -> bool:
        """Reject ordinary INSPECT when target/frontier remains feasible."""

        candidate_tier = self._task_priority(
            candidate
        )

        # Tier 0 target 与 Tier 1 frontier 本身不受本规则限制。
        if candidate_tier <= 1:
            return False

        robot = self.robot_by_name.get(robot_id)

        if robot is None:
            return False

        for option in catalog:
            if str(option.get("robot_id")) != robot_id:
                continue

            option_tier = self._task_priority(option)

            if option_tier > 1:
                continue

            if self._candidate_safe_now(
                robot,
                option,
            ):
                return True

        return False

    def _ensure_initial_uav_explore(self, event_reason: str, catalog: List[Dict[str, Any]],
                                    accepted: List[Dict[str, Any]]) -> None:
        if not bool(self.cfg.get('require_initial_uav_explore', True)):
            return
        if str(event_reason) != 'INITIAL_CONTEXT_READY':
            return
        for item in accepted:
            robot = self.robot_by_name.get(item['robot_id'])
            if robot and robot.get('type') == 'uav' and str(item.get('task_type')) == 'EXPLORE':
                return
        uav_ids = [str(r['name']) for r in self.robots if r.get('type') == 'uav']
        assigned = {item['robot_id'] for item in accepted}
        # Prefer an unassigned UAV. If both are assigned only to scan/hold, replace
        # one such assignment with a feasible exploration candidate.
        order = [rid for rid in uav_ids if rid not in assigned] + [rid for rid in uav_ids if rid in assigned]
        for robot_id in order:
            candidate = self._fallback_for(catalog, robot_id, moving_only=True)
            if candidate is None or str(candidate.get('task_type')) != 'EXPLORE':
                continue
            accepted[:] = [item for item in accepted if item['robot_id'] != robot_id]
            accepted.append({
                'robot_id': robot_id,
                'candidate_id': candidate['id'],
                'candidate': candidate,
                'role': 'INITIAL_AERIAL_EXPLORER',
                'task_type': 'EXPLORE',
            })
            rospy.loginfo('Validator enforces initial UAV exploration: %s -> %s.', robot_id, candidate['id'])
            return

    def _validate(self, envelope: Dict[str, Any]) -> None:
        epoch_id = envelope.get('epoch_id')
        if str(envelope.get('status', 'PLANNED')).upper() != 'PLANNED':
            output = {
                'epoch_id': epoch_id,
                'status': 'SKIPPED_BACKEND_ERROR',
                'accepted': [],
                'rejected': [{'reason': 'central_vlm_backend_error',
                              'detail': (envelope.get('plan') or {}).get('backend_error', '')}],
            }
            self.pub.publish(compact_json(output))
            rospy.logwarn('VLM plan validator skipped epoch %s because central VLM did not produce a plan.', epoch_id)
            return

        catalog = {c.get('id'): c for c in envelope.get('candidate_catalog', [])
                   if isinstance(c, dict) and c.get('id')}
        raw_plan = envelope.get('plan') or {}
        assignments = raw_plan.get('assignments', []) if isinstance(raw_plan, dict) else []
        used = set()
        assigned_robots = set()
        accepted: List[Dict[str, Any]] = []
        rejected: List[Dict[str, Any]] = []
        for assignment in assignments if isinstance(assignments, list) else []:
            if not isinstance(assignment, dict):
                continue
            robot_id = str(assignment.get('robot_id', ''))
            candidate_id = assignment.get('candidate_id')
            robot = self.robot_by_name.get(robot_id)
            candidate = catalog.get(candidate_id)
            reason = None
            if robot is None:
                reason = 'unknown_robot'
            elif candidate is None:
                reason = 'unknown_candidate'
            elif str(candidate.get('robot_id')) != robot_id:
                reason = 'candidate_robot_mismatch'
            elif robot_id in assigned_robots:
                reason = 'duplicate_robot_assignment'  
            elif bool(self.cfg.get('reject_duplicate_candidate_assignment', True)) and candidate_id in used:
                reason = 'duplicate_candidate'
            elif (bool(self.cfg.get("enforce_task_priority", True,))
                and self._has_higher_priority_option(list(catalog.values()), robot_id, candidate,)):
                reason = 'lower_priority_inspection_while_target_or_frontier_available'
            elif not self._candidate_safe_now(robot, candidate):
                reason = 'candidate_no_longer_reachable'
            if reason is not None:
                rejected.append({'assignment': assignment, 'reason': reason})
                continue
            used.add(candidate_id)
            assigned_robots.add(robot_id)
            accepted.append({
                'robot_id': robot_id,
                'candidate_id': candidate_id,
                'candidate': candidate,
                'role': assignment.get('role', ''),
                'task_type': assignment.get('task_type', candidate.get('task_type')),
            })

        self._ensure_initial_uav_explore(str(envelope.get('event_reason', '')), list(catalog.values()), accepted)

        source = str(envelope.get('source', 'VLM'))
        # A normal VLM plan may be completed for omitted robots. A backend
        # recovery request, however, only reroutes its affected participant(s);
        # unaffected robots retain their current validated route.
        complete_unassigned = (
            bool(self.cfg.get('force_joint_assignment', False))
            or source != 'MAP_FALLBACK'
            or str(envelope.get('event_reason', '')) == 'INITIAL_CONTEXT_READY'
        )
        if complete_unassigned and bool(self.cfg.get('assign_unassigned_robots', True)) and \
                str(self.cfg.get('fallback_mode', 'safe_nearest_candidate')) == 'safe_nearest_candidate':
            assigned = {item['robot_id'] for item in accepted}
            for robot_id in self.robot_by_name:
                if robot_id in assigned:
                    continue
                candidate = self._fallback_for(list(catalog.values()), robot_id, moving_only=True)
                if candidate is None:
                    candidate = self._fallback_for(list(catalog.values()), robot_id, moving_only=False)
                if candidate is not None:
                    accepted.append({
                        'robot_id': robot_id,
                        'candidate_id': candidate['id'],
                        'candidate': candidate,
                        'role': 'SAFE_ENDPOINT_FALLBACK',
                        'task_type': candidate.get('task_type'),
                    })

        for item in accepted:
            candidate = item['candidate']
            dispatch = {
                'epoch_id': epoch_id,
                'robot_id': item['robot_id'],
                'candidate_id': item['candidate_id'],
                'task_type': item.get('task_type', candidate.get('task_type', 'EXPLORE')),
                'role': item.get('role', ''),
                'goal': candidate.get('goal', {}),
                'source': source,
            }
            # Goal dispatch is task metadata only. The route planner will compute
            # and publish a current-map A* Path; no direct final-goal command is
            # published here.
            self.dispatch_pub.publish(compact_json(dispatch))
            self.route_req_pub.publish(compact_json(dispatch))

        status = 'VALID' if accepted else 'NO_VALID_ASSIGNMENT'
        output = {
            'epoch_id': epoch_id,
            'status': status,
            'accepted': [{
                'robot_id': a['robot_id'], 'candidate_id': a['candidate_id'],
                'role': a['role'], 'task_type': a['task_type']
            } for a in accepted],
            'rejected': rejected,
            'route_mode': 'POST_SELECTION_ASTAR',
        }
        self.pub.publish(compact_json(output))
        rospy.loginfo('VLM plan validator epoch %s: %d accepted, %d rejected; requested fresh A* routes.',
                      epoch_id, len(accepted), len(rejected))


def main() -> None:
    rospy.init_node('vlm_plan_validator')
    VLMPlanValidator()
    rospy.spin()


if __name__ == '__main__':
    main()
