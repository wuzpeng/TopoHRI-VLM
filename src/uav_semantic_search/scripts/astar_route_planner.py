#!/usr/bin/env python3
"""Deterministic post-selection A* route planner for Stage-5.

The VLM and deterministic backend fallback select only a safe candidate endpoint.
This node always recomputes the executable route *after* that endpoint has been
selected, using the latest robot pose and latest robot-specific occupancy map.
It therefore never accepts VLM-generated coordinates or VLM-generated paths.
"""
from __future__ import annotations

import math
import os
import sys
import threading
from typing import Any, Dict, List, Optional, Tuple

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from std_msgs.msg import String
from tf.transformations import quaternion_from_euler

_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if _SCRIPT_DIR not in sys.path:
    sys.path.insert(0, _SCRIPT_DIR)

from frontier_core import astar, build_passable_mask, nearest_passable, simplify_path
from vlm_common import compact_json, grid_from_msg, safe_json_loads


class AStarRoutePlanner:
    def __init__(self):
        self.root = rospy.get_param('/vlm_semantic_search')
        self.cfg = self.root.get('route_planner', {})
        self.robots = list(rospy.get_param('/vehicles', [])) + list(rospy.get_param('/ground_robots', []))
        self.robot_by_name = {str(r['name']): r for r in self.robots}
        self.lock = threading.RLock()
        self.maps: Dict[str, Optional[OccupancyGrid]] = {'uav': None, 'ugv': None}
        self.poses: Dict[str, Optional[PoseStamped]] = {str(r['name']): None for r in self.robots}
        self.current_query_version = int(self.root.get('target_query', {}).get('query_version', 0))
        self.path_pubs = {
            str(r['name']): rospy.Publisher(self._path_topic(r), Path, queue_size=3, latch=True)
            for r in self.robots
        }
        self.result_pub = rospy.Publisher(str(self.cfg.get('result_topic', '/vlm/route_result')), String, queue_size=20)
        self.fallback_req_pub = rospy.Publisher('/vlm/backend_fallback_request', String, queue_size=10)
        rospy.Subscriber('/global_map_2d', OccupancyGrid, lambda msg: self._map_cb('uav', msg), queue_size=2)
        rospy.Subscriber('/ugv0/ground_map_2d', OccupancyGrid, lambda msg: self._map_cb('ugv', msg), queue_size=2)
        for robot in self.robots:
            rospy.Subscriber(robot['global_pose_topic'], PoseStamped,
                             lambda msg, n=str(robot['name']): self._pose_cb(n, msg), queue_size=20)
        rospy.Subscriber(str(self.cfg.get('request_topic', '/vlm/route_request')),
                         String, self._request_cb, queue_size=20)
        rospy.Subscriber('/vlm/target_query', String, self._query_cb, queue_size=5)
        rospy.Subscriber('/vlm/query_switch_cancel', String, self._cancel_cb, queue_size=10)
        rospy.loginfo('A* route planner ready: VLM selections are endpoint-only; routes are replanned from current map/pose.')

    @staticmethod
    def _path_topic(robot: Dict[str, Any]) -> str:
        return str(robot.get('planned_path_topic', '/%s/search/planned_path' % robot['name']))

    def _map_cb(self, key: str, msg: OccupancyGrid) -> None:
        with self.lock:
            self.maps[key] = msg

    def _pose_cb(self, robot_id: str, msg: PoseStamped) -> None:
        with self.lock:
            self.poses[robot_id] = msg

    def _query_cb(self, msg: String) -> None:
        query = safe_json_loads(msg.data, None)
        if not isinstance(query, dict):
            return
        with self.lock:
            self.current_query_version = int(query.get('query_version', self.current_query_version))

    def _cancel_cb(self, msg: String) -> None:
        notice = safe_json_loads(msg.data, None)
        if not isinstance(notice, dict):
            return
        with self.lock:
            self.current_query_version = int(notice.get('new_query_version', self.current_query_version))
        rospy.logwarn('A* route planner dropped stale route requests for target switch to query v%d.', self.current_query_version)

    def _request_cb(self, msg: String) -> None:
        request = safe_json_loads(msg.data, None)
        if isinstance(request, dict):
            threading.Thread(target=self._plan, args=(request,), daemon=True).start()

    @staticmethod
    def _profile(robot: Dict[str, Any]) -> Tuple[int, float, int, int]:
        racer = rospy.get_param('/racer_stage3', {})
        heterogeneous = rospy.get_param('/heterogeneous_fuel', {})
        occupied = int(racer.get('occupied_threshold', 65))
        if robot.get('type') == 'ugv':
            return (
                occupied,
                float(heterogeneous.get('ugv_obstacle_inflation_m', 0.54)),
                int(heterogeneous.get('ugv_nearest_free_search_cells', 20)),
                int(heterogeneous.get('ugv_astar_max_expansions', 30000)),
            )
        return (
            occupied,
            float(racer.get('obstacle_inflation_m', 0.55)),
            int(racer.get('nearest_free_search_cells', 25)),
            int(racer.get('astar_max_expansions', 30000)),
        )

    @staticmethod
    def _pose_from_cell(grid, cell, z: float, yaw: float) -> PoseStamped:
        x, y = grid.cell_to_world(cell)
        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = rospy.Time.now()
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = float(z)
        q = quaternion_from_euler(0.0, 0.0, float(yaw))
        msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w = q
        return msg

    def _make_path(self, robot: Dict[str, Any], goal: Dict[str, Any]) -> Tuple[Optional[Path], str]:
        robot_id = str(robot['name'])
        key = 'ugv' if robot.get('type') == 'ugv' else 'uav'
        with self.lock:
            map_msg = self.maps.get(key)
            pose = self.poses.get(robot_id)
        grid = grid_from_msg(map_msg)
        if grid is None:
            return None, 'map_unavailable'
        if pose is None:
            return None, 'pose_unavailable'
        if not isinstance(goal, dict) or 'x' not in goal or 'y' not in goal:
            return None, 'invalid_selected_goal'

        occupied, inflation, near, max_exp = self._profile(robot)
        passable, _ = build_passable_mask(grid, occupied, inflation)
        start = grid.world_to_cell(pose.pose.position.x, pose.pose.position.y)
        end = grid.world_to_cell(float(goal['x']), float(goal['y']))
        if start is None or end is None:
            return None, 'goal_outside_current_map'
        start = nearest_passable(passable, start, near)
        end = nearest_passable(passable, end, near)
        if start is None or end is None:
            return None, 'no_nearby_passable_cell'
        cells = astar(passable, start, end, max_exp)
        if not cells:
            return None, 'no_current_astar_route'
        try:
            cells = simplify_path(cells, passable)
        except Exception:
            pass
        if not cells:
            return None, 'empty_simplified_route'

        z = float(goal.get('z', pose.pose.position.z))
        path = Path()
        path.header.frame_id = 'map'
        path.header.stamp = rospy.Time.now()
        final_yaw = float(goal.get('yaw_rad', 0.0) or 0.0)
        for i, cell in enumerate(cells):
            if i + 1 < len(cells):
                x0, y0 = grid.cell_to_world(cell)
                x1, y1 = grid.cell_to_world(cells[i + 1])
                yaw = math.atan2(y1 - y0, x1 - x0)
            else:
                yaw = final_yaw
            path.poses.append(self._pose_from_cell(grid, cell, z, yaw))
        return path, 'ok'

    def _plan(self, request: Dict[str, Any]) -> None:
        robot_id = str(request.get('robot_id', ''))
        robot = self.robot_by_name.get(robot_id)
        output = {
            'epoch_id': request.get('epoch_id'),
            'robot_id': robot_id,
            'candidate_id': request.get('candidate_id'),
            'source': request.get('source', 'VLM'),
            'task_type': request.get('task_type', 'EXPLORE'),
            'query_version': request.get('query_version'),
        }
        try:
            request_query_version = int(request.get('query_version', self.current_query_version))
        except (TypeError, ValueError):
            request_query_version = int(self.current_query_version)
        with self.lock:
            active_query_version = int(self.current_query_version)
        if request_query_version != active_query_version:
            output.update({
                'status': 'ROUTE_REJECTED',
                'reason': 'stale_query_route_request',
                'active_query_version': active_query_version,
            })
            self.result_pub.publish(compact_json(output))
            rospy.logwarn('A* route rejected stale request for %s candidate=%s query v%d; active v%d.',
                          robot_id, request.get('candidate_id'), request_query_version, active_query_version)
            return
        if robot is None:
            output.update({'status': 'ROUTE_REJECTED', 'reason': 'unknown_robot'})
            self.result_pub.publish(compact_json(output))
            return
        path, reason = self._make_path(robot, request.get('goal', {}))

        # A target switch may happen while A* is computing.  Re-check before
        # publishing the path because Path messages do not carry query metadata.
        with self.lock:
            active_query_version = int(self.current_query_version)
        if request_query_version != active_query_version:
            output.update({
                'status': 'ROUTE_REJECTED',
                'reason': 'stale_query_route_result',
                'active_query_version': active_query_version,
            })
            self.result_pub.publish(compact_json(output))
            rospy.logwarn('A* route result discarded for %s candidate=%s query v%d; active v%d.',
                          robot_id, request.get('candidate_id'), request_query_version, active_query_version)
            return
        
        if path is None:
            output.update({'status': 'ROUTE_REJECTED', 'reason': reason})
            self.result_pub.publish(compact_json(output))

            rospy.logwarn('A* route rejected for %s candidate=%s: %s.', robot_id,
                        request.get('candidate_id'), reason)

            # 只在原始 VLM 任务失败时尝试一次几何候选回退。
            # MAP_FALLBACK 自己失败时不再递归触发，避免无限循环。
            if str(request.get('source', 'VLM')) != 'MAP_FALLBACK':
                self.fallback_req_pub.publish(compact_json({
                    'epoch_id': request.get('epoch_id'),
                    'robot_id': robot_id,
                    'participants': [robot_id],
                    'failure_reason': 'ROUTE_REJECTED_%s' % reason,
                    'failed_candidate_id': request.get('candidate_id'),
                }))

            return
        
        self.path_pubs[robot_id].publish(path)
        length = 0.0
        for a, b in zip(path.poses[:-1], path.poses[1:]):
            pa, pb = a.pose.position, b.pose.position
            length += math.hypot(pb.x - pa.x, pb.y - pa.y)
        output.update({
            'status': 'ROUTE_READY',
            'reason': 'ok',
            'waypoint_count': len(path.poses),
            'path_length_m': round(length, 3),
        })
        self.result_pub.publish(compact_json(output))
        rospy.loginfo('A* route ready for %s candidate=%s with %d waypoint(s).',
                      robot_id, request.get('candidate_id'), len(path.poses))



def main() -> None:
    rospy.init_node('astar_route_planner')
    AStarRoutePlanner()
    rospy.spin()


if __name__ == '__main__':
    main()
