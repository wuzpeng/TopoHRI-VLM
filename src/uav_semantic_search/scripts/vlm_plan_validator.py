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


_MOVING_TASKS = {
    'EXPLORE',
    'INSPECT',
    'GROUND_VERIFY',
    'AERIAL_INSPECT',
    'HRI_REGION_SEARCH',
    'QUERY_RESCAN',
}


class VLMPlanValidator:
    def __init__(self):
        self.root = rospy.get_param('/vlm_semantic_search')
        self.cfg = self.root.get('validator', {})
        ablation = dict(self.root.get('ablation', {}))
        self.topology_planning_enabled = bool(
            ablation.get('enable_topology_planning', True))
        self.state_action_gate_enabled = bool(
            ablation.get('enable_state_action_gate', True))
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

        if task in ("EXPLORE", "HRI_REGION_SEARCH", "QUERY_RESCAN"):
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

    @staticmethod
    def _is_hri_region_candidate(candidate: Dict[str, Any]) -> bool:
        return (
            str(candidate.get('task_type', '')).upper() == 'HRI_REGION_SEARCH'
            or str(candidate.get('candidate_class', '')).upper() in (
                'HRI_REGION_SEARCH',
                'HRI_REGION_PERIMETER_SCAN',
            )
        )

    @classmethod
    def _action_priority(cls, candidate: Dict[str, Any]) -> int:
        """确定性状态—动作优先级，数值越小优先级越高。"""

        # 当前 query 的目标验证候选
        if cls._task_priority(candidate) == 0:
            return 0

        # 人类主动设置的优先区域
        if cls._is_hri_region_candidate(candidate):
            return 1

        task = str(candidate.get('task_type', '')).upper()
        candidate_class = str(
            candidate.get('candidate_class', '')
        ).upper()

        # 可达 Frontier
        if task == 'EXPLORE' or candidate_class == 'FRONTIER':
            return 2

        # 已探索区域重扫
        if task == 'QUERY_RESCAN' or 'RESCAN' in candidate_class:
            return 3

        # 普通语义检查
        if task in (
            'INSPECT',
            'GROUND_VERIFY',
            'AERIAL_INSPECT',
        ):
            return 4

        # 原地动作
        if task in (
            'HOVER_AND_SCAN',
            'SCAN_IN_PLACE',
            'HOLD',
        ):
            return 5

        return 50 + cls._task_priority(candidate)

    def _has_feasible_hri_region_option(
            self,
            catalog: List[Dict[str, Any]],
            robot_id: str,
    ) -> bool:
        robot = self.robot_by_name.get(robot_id)
        if robot is None:
            return False
        for option in catalog:
            if str(option.get('robot_id')) != robot_id:
                continue
            if not self._is_hri_region_candidate(option):
                continue
            if self._candidate_safe_now(robot, option):
                return True
        return False

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

    @staticmethod
    def _topology_region(candidate: Dict[str, Any]) -> Optional[tuple]:
        if str(candidate.get('task_type', '')).upper() != 'EXPLORE':
            return None
        region = candidate.get('topology_region_id')
        confidence = str(candidate.get('topology_confidence', 'LOW')).upper()
        # A LOW-confidence UNASSIGNED label is still unique to one frontier
        # cluster (Fxxx), so identical per-robot copies must not be duplicated.
        is_unique_unassigned = bool(region and ':UNASSIGNED:F' in str(region))
        if not region or (confidence not in ('HIGH', 'MEDIUM')
                          and not is_unique_unassigned):
            return None
        return str(candidate.get('topology_layer', 'topology')), str(region)

    @classmethod
    def _occupied_topology_regions(cls, accepted: List[Dict[str, Any]]) -> set:
        occupied = set()
        for item in accepted:
            region = cls._topology_region(item.get('candidate', {}))
            if region is not None:
                occupied.add(region)
        return occupied

    def _fallback_for(self, catalog: List[Dict[str, Any]], robot_id: str,
                      moving_only: bool = False,
                      accepted: Optional[List[Dict[str, Any]]] = None) -> Optional[Dict[str, Any]]:
        robot = self.robot_by_name.get(robot_id)
        if robot is None:
            return None
        options = [c for c in catalog if str(c.get('robot_id')) == robot_id]
        if moving_only:
            moving = [c for c in options if str(c.get('task_type')) in _MOVING_TASKS]
            if moving:
                options = moving
        occupied_topology = self._occupied_topology_regions(accepted or [])
        options.sort(key=lambda c: (
            self._action_priority(c),
            -int(self._is_hri_region_candidate(c)),
            -float(c.get('human_priority_score', 0.0) or 0.0),
            int(self._topology_region(c) in occupied_topology
                if self._topology_region(c) is not None else False),
            -float(c.get('frontier_utility', c.get('information_gain', 0.0)) or 0.0),
            -float(c.get('information_gain', 0.0) or 0.0),
            float(c.get('risk', 0.0) or 0.0),
            float(c.get('path_length_m', 1e9) or 1e9),
        ))
        for candidate in options:
            if accepted is not None and not self._within_human_region_capacity(
                    candidate, accepted):
                continue
            if (accepted is not None
                    and self.topology_planning_enabled
                    and bool(self.cfg.get('enforce_topology_region_diversity', True))
                    and not self._within_topology_region_capacity(
                        candidate, accepted, catalog, robot_id)):
                continue
            if self._candidate_safe_now(robot, candidate):
                return candidate
        return None
    
    def _has_higher_priority_option(
            self,
            catalog: List[Dict[str, Any]],
            robot_id: str,
            candidate: Dict[str, Any],
    ) -> bool:
        """如果该机器人存在更高优先级且仍安全可达的候选，则拒绝当前动作。"""

        candidate_priority = self._action_priority(candidate)

        robot = self.robot_by_name.get(robot_id)

        if robot is None:
            return False

        for option in catalog:
            if str(option.get('robot_id')) != robot_id:
                continue

            # 只检查严格高于当前动作的候选
            if self._action_priority(option) >= candidate_priority:
                continue

            if self._candidate_safe_now(robot, option):
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
            candidate = self._fallback_for(
                catalog, robot_id, moving_only=True, accepted=accepted)
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

    @staticmethod
    def _within_human_region_capacity(candidate: Dict[str, Any], accepted: List[Dict[str, Any]]) -> bool:
        """Return False when an assignment would exceed a region's robot cap."""
        regions = candidate.get('human_priority_regions', [])
        if not isinstance(regions, list) or not regions:
            return True
        existing_ids = []
        for item in accepted:
            other = item.get('candidate', {}) if isinstance(item, dict) else {}
            if isinstance(other, dict):
                for region in other.get('human_priority_regions', []):
                    if isinstance(region, dict) and region.get('region_id'):
                        existing_ids.append(str(region['region_id']))
        for region in regions:
            if not isinstance(region, dict) or not region.get('region_id'):
                continue
            region_id = str(region['region_id'])
            try:
                cap = max(1, int(region.get('max_robots', 1)))
            except (TypeError, ValueError):
                cap = 1
            if existing_ids.count(region_id) >= cap:
                return False
        return True

    @staticmethod
    def _goal_separation_m(a: Dict[str, Any], b: Dict[str, Any]) -> float:
        try:
            ag, bg = a['goal'], b['goal']
            dx = float(ag['x']) - float(bg['x'])
            dy = float(ag['y']) - float(bg['y'])
            return (dx * dx + dy * dy) ** 0.5
        except (KeyError, TypeError, ValueError):
            return 0.0

    def _has_unused_topology_alternative(
            self, catalog: List[Dict[str, Any]], robot_id: str,
            accepted: List[Dict[str, Any]]) -> bool:
        robot = self.robot_by_name.get(robot_id)
        if robot is None:
            return False
        occupied = self._occupied_topology_regions(accepted)
        for option in catalog:
            if str(option.get('robot_id')) != robot_id:
                continue
            region = self._topology_region(option)
            if region is None or region in occupied:
                continue
            if self._candidate_safe_now(robot, option):
                return True
        return False

    def _within_topology_region_capacity(
            self, candidate: Dict[str, Any], accepted: List[Dict[str, Any]],
            catalog: List[Dict[str, Any]], robot_id: str) -> bool:
        """Prefer one robot per skeleton branch, with a safe small-map fallback."""
        region = self._topology_region(candidate)
        if region is None:
            return True
        same_region = [item for item in accepted
                       if self._topology_region(item.get('candidate', {})) == region]
        capacity = max(1, int(self.cfg.get('topology_region_default_capacity', 1)))
        if len(same_region) < capacity:
            return True
        if self._has_unused_topology_alternative(catalog, robot_id, accepted):
            return False
        if not bool(self.cfg.get('topology_allow_shared_region_if_no_alternative', True)):
            return False
        minimum = float(self.cfg.get('topology_shared_region_min_goal_separation_m', 2.2))
        return all(self._goal_separation_m(candidate, item.get('candidate', {})) >= minimum
                   for item in same_region)

    @staticmethod
    def _candidate_query_version(candidate: Dict[str, Any]) -> Optional[int]:
        for key in ('query_version',):
            if key in candidate:
                try:
                    return int(candidate.get(key))
                except (TypeError, ValueError):
                    return None
        anchor = candidate.get('semantic_anchor')
        if isinstance(anchor, dict):
            for key in ('query_version', 'old_query_version'):
                if key in anchor:
                    try:
                        return int(anchor.get(key))
                    except (TypeError, ValueError):
                        return None
        return None

    def _is_stale_target_verification(self, candidate: Dict[str, Any], current_query_version: int) -> bool:
        task = str(candidate.get('task_type', '')).upper()
        cclass = str(candidate.get('candidate_class', '')).upper()
        if task not in ('AERIAL_INSPECT', 'GROUND_VERIFY') and cclass != 'TARGET':
            return False
        qv = self._candidate_query_version(candidate)
        return qv is not None and int(qv) != int(current_query_version)

    def _validate(self, envelope: Dict[str, Any]) -> None:
        epoch_id = envelope.get('epoch_id')
        current_query_version = int((envelope.get('query') or {}).get('query_version', -1))
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
            elif (bool(self.cfg.get('reject_stale_target_verification', True))
                  and self._is_stale_target_verification(candidate, current_query_version)):
                reason = 'stale_target_verification_after_query_switch'
            elif (self.state_action_gate_enabled
                and bool(self.cfg.get("enforce_task_priority", True,))
                and self._has_higher_priority_option(list(catalog.values()), robot_id, candidate,)):
                reason = 'lower_priority_action_while_better_option_available'
            elif (bool(self.cfg.get('enforce_hri_region_priority', True))
                  and int(candidate.get('priority_tier', 99)) > 0
                  and not self._is_hri_region_candidate(candidate)
                  and self._has_feasible_hri_region_option(list(catalog.values()), robot_id)):
                reason = 'human_priority_region_candidate_available'
            elif not self._within_human_region_capacity(candidate, accepted):
                reason = 'human_priority_region_capacity_exceeded'
            elif (self.topology_planning_enabled
                  and bool(self.cfg.get('enforce_topology_region_diversity', True))
                  and not self._within_topology_region_capacity(
                      candidate, accepted, list(catalog.values()), robot_id)):
                reason = 'topology_region_already_assigned'
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
                candidate = self._fallback_for(
                    list(catalog.values()), robot_id,
                    moving_only=True, accepted=accepted)
                if candidate is None:
                    candidate = self._fallback_for(
                        list(catalog.values()), robot_id,
                        moving_only=False, accepted=accepted)
                if candidate is not None and not self._within_human_region_capacity(candidate, accepted):
                    candidate = next((c for c in list(catalog.values())
                                      if str(c.get('robot_id')) == robot_id
                                      and self._within_human_region_capacity(c, accepted)
                                      and self._within_topology_region_capacity(
                                          c, accepted, list(catalog.values()), robot_id)
                                      and self._candidate_safe_now(self.robot_by_name[robot_id], c)), None)
                if (candidate is not None
                        and self.topology_planning_enabled
                        and bool(self.cfg.get('enforce_topology_region_diversity', True))
                        and not self._within_topology_region_capacity(
                            candidate, accepted, list(catalog.values()), robot_id)):
                    candidate = next((c for c in list(catalog.values())
                                      if str(c.get('robot_id')) == robot_id
                                      and self._within_human_region_capacity(c, accepted)
                                      and self._within_topology_region_capacity(
                                          c, accepted, list(catalog.values()), robot_id)
                                      and self._candidate_safe_now(
                                          self.robot_by_name[robot_id], c)), None)
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
                'query_version': current_query_version,
                'event_reason': envelope.get('event_reason', ''),
                'topology_region_id': candidate.get('topology_region_id'),
                'topology_layer': candidate.get('topology_layer'),
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
                'role': a['role'], 'task_type': a['task_type'],
                'topology_region_id': a['candidate'].get('topology_region_id')
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
