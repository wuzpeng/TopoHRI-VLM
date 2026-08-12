#!/usr/bin/env python3
"""Centralized RACER-inspired cooperative exploration manager (Stage-3A/3B).

Design scope
------------
This node implements the operational baseline used before a VLM planner is added:
  1. extract clustered frontiers from the shared 2.5D occupancy map;
  2. score safe viewpoints using unknown-space gain, A* path cost, risk, hgrid
     ownership, and duplicate-coverage penalties;
  3. jointly assign different frontiers to two UAVs while balancing path load;
  4. convert each A* path into sequential map-frame mission waypoints;
  5. replan when a path completes, becomes stale, or a confirmed target ends search.

It deliberately reuses /uavX/mission/goal and waypoint_executor.py. It does NOT
replace PX4, MAVROS, mapping, or semantic sensing.
"""
from __future__ import annotations

# catkin_install_python creates executable wrappers in devel/lib.  A helper
# module must be imported from this source/install directory rather than from
# an executable wrapper with the same filename.
import os
import sys
_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if _SCRIPT_DIR not in sys.path:
    sys.path.insert(0, _SCRIPT_DIR)

import math
import threading
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from std_msgs.msg import Bool, String
from visualization_msgs.msg import Marker, MarkerArray

from uav_semantic_search.msg import TargetHypothesisArray
from frontier_core import (
    FrontierCluster,
    GridMap,
    astar,
    extract_frontier_clusters,
    nearest_passable,
    occupancy_grid_from_flat,
    path_length_m,
    simplify_path,
)


@dataclass
class VehicleState:
    name: str
    altitude: float
    pose: Optional[PoseStamped] = None
    reached: bool = False
    takeoff_ready: bool = False
    active: bool = False
    task_id: str = ''
    cluster_id: int = -1
    zone: Optional[Tuple[int, int]] = None
    queued_cells: List[Tuple[int, int]] = field(default_factory=list)
    goal_index: int = 0
    goal_sent_time: rospy.Time = field(default_factory=lambda: rospy.Time(0))
    estimated_path_m: float = 0.0
    last_completed_xy: Optional[Tuple[float, float]] = None


@dataclass
class CandidatePlan:
    vehicle: str
    cluster: FrontierCluster
    path_cells: List[Tuple[int, int]]
    path_cost_m: float
    utility: float


class AutonomousSearchManager:
    def __init__(self):
        self.cfg = rospy.get_param('/racer_stage3')
        self.map_frame = rospy.get_param('/map/frame_id', 'map')
        self.vehicle_cfgs = {item['name']: item for item in rospy.get_param('/vehicles', [])}
        if len(self.vehicle_cfgs) < 1:
            raise RuntimeError('Missing /vehicles configuration.')

        self.lock = threading.RLock()
        self.map_msg: Optional[OccupancyGrid] = None
        self.last_map_stamp = rospy.Time(0)
        self.start_time = rospy.Time.now()
        # Timing for the map bootstrap begins only after *all* UAVs report that
        # they have climbed to and held their individual takeoff heights.
        self.all_takeoff_ready_time = rospy.Time(0)
        self.last_plan_time = rospy.Time(0)
        self.mission_finished = False
        self.confirmed_target_seen = False
        self.latest_clusters: List[FrontierCluster] = []
        self.zone_claims: Dict[Tuple[int, int], str] = {}
        self.known_cells_at_last_plan = 0

        self.vehicles: Dict[str, VehicleState] = {}
        self.goal_pubs = {}
        self.path_pubs = {}
        for name, vehicle in self.vehicle_cfgs.items():
            altitude = float(vehicle.get('takeoff_height', 1.8))
            self.vehicles[name] = VehicleState(name=name, altitude=altitude)
            self.goal_pubs[name] = rospy.Publisher('/%s/mission/goal' % name, PoseStamped, queue_size=3, latch=True)
            self.path_pubs[name] = rospy.Publisher('/%s/search/planned_path' % name, Path, queue_size=1, latch=True)
            rospy.Subscriber(vehicle['global_pose_topic'], PoseStamped,
                             lambda msg, robot=name: self._pose_cb(robot, msg), queue_size=20)
            rospy.Subscriber('/%s/mission/reached' % name, Bool,
                             lambda msg, robot=name: self._reached_cb(robot, msg), queue_size=10)
            rospy.Subscriber('/%s/mission/takeoff_ready' % name, Bool,
                             lambda msg, robot=name: self._takeoff_ready_cb(robot, msg), queue_size=2)

        rospy.Subscriber('/global_map_2d', OccupancyGrid, self._map_cb, queue_size=1)
        rospy.Subscriber('/semantic_map/confirmed_targets', TargetHypothesisArray,
                         self._confirmed_cb, queue_size=5)

        self.marker_pub = rospy.Publisher('/search/racer_markers', MarkerArray, queue_size=1, latch=True)
        self.status_pub = rospy.Publisher('/search/status', String, queue_size=10, latch=True)
        self.assignment_pub = rospy.Publisher('/search/assigned_goals', MarkerArray, queue_size=1, latch=True)

        tick_hz = max(1.0, float(self.cfg.get('manager_tick_hz', 2.0)))
        rospy.Timer(rospy.Duration(1.0 / tick_hz), self._tick)
        rospy.loginfo('Centralized RACER-inspired exploration manager is ready for %s.', list(self.vehicles))
        self.status_pub.publish('WAIT_READY')

    def _pose_cb(self, robot: str, msg: PoseStamped):
        with self.lock:
            self.vehicles[robot].pose = msg

    def _map_cb(self, msg: OccupancyGrid):
        with self.lock:
            self.map_msg = msg
            self.last_map_stamp = msg.header.stamp

    def _reached_cb(self, robot: str, msg: Bool):
        if not msg.data:
            return
        with self.lock:
            state = self.vehicles[robot]
            if not state.active:
                return
            # waypoint_executor reports one true event for every current mission goal.
            self._advance_path_locked(state)

    def _takeoff_ready_cb(self, robot: str, msg: Bool):
        with self.lock:
            state = self.vehicles[robot]
            changed = (state.takeoff_ready != bool(msg.data))
            state.takeoff_ready = bool(msg.data)
            if not state.takeoff_ready:
                self.all_takeoff_ready_time = rospy.Time(0)
            if changed:
                rospy.loginfo('%s takeoff-ready=%s.', robot, state.takeoff_ready)

    def _confirmed_cb(self, msg: TargetHypothesisArray):
        if not bool(self.cfg.get('stop_on_confirmed_target', True)):
            return
        if not msg.hypotheses:
            return
        with self.lock:
            self.confirmed_target_seen = True
            self.mission_finished = True
            self.status_pub.publish('TARGET_CONFIRMED')
            rospy.logwarn('Confirmed semantic target received: autonomous exploration stops assigning new goals.')

    def _grid_locked(self) -> Optional[GridMap]:
        if self.map_msg is None or not self.map_msg.data:
            return None
        msg = self.map_msg
        return occupancy_grid_from_flat(
            msg.data, msg.info.width, msg.info.height, msg.info.resolution,
            msg.info.origin.position.x, msg.info.origin.position.y,
            msg.header.frame_id or self.map_frame)

    def _known_cells(self, grid: GridMap) -> int:
        return int((grid.data >= 0).sum())

    def _ready_locked(self, grid: Optional[GridMap], now: rospy.Time) -> Tuple[bool, str]:
        if grid is None:
            return False, 'WAIT_MAP'
        if any(v.pose is None for v in self.vehicles.values()):
            return False, 'WAIT_POSES'
        if any(not v.takeoff_ready for v in self.vehicles.values()):
            # Critical safety gate: no frontier assignment until every UAV has
            # completed a purely vertical takeoff and a short hover-settle period.
            return False, 'WAIT_TAKEOFF'

        if self.all_takeoff_ready_time.is_zero():
            self.all_takeoff_ready_time = now
            rospy.loginfo('All UAVs are takeoff-ready. Starting post-takeoff map bootstrap.')

        delay = float(self.cfg.get(
            'post_takeoff_map_bootstrap_sec',
            self.cfg.get('startup_delay_sec', 8.0)))
        if (now - self.all_takeoff_ready_time).to_sec() < delay:
            return False, 'MAP_BOOTSTRAP_AFTER_TAKEOFF'
        if self._known_cells(grid) < int(self.cfg['min_known_cells']):
            return False, 'MAP_BOOTSTRAP'
        return True, 'EXPLORE'

    def _vehicle_start_cell(self, grid: GridMap, passable, state: VehicleState):
        if state.pose is None:
            return None
        position = state.pose.pose.position
        cell = grid.world_to_cell(position.x, position.y)
        if cell is None:
            return None
        return nearest_passable(passable, cell, max_radius_cells=int(self.cfg['nearest_free_search_cells']))

    def _zone_owner(self, grid: GridMap, cluster: FrontierCluster) -> Optional[str]:
        # A central hgrid analogue: a zone naturally belongs to the closest robot,
        # unless it has an active claim from another robot.
        if cluster.zone in self.zone_claims:
            return self.zone_claims[cluster.zone]
        wx, wy = grid.cell_to_world(cluster.viewpoint_cell)
        best_name = None
        best_dist = float('inf')
        for name, state in self.vehicles.items():
            if state.pose is None:
                continue
            p = state.pose.pose.position
            dist = math.hypot(wx - p.x, wy - p.y)
            if dist < best_dist:
                best_dist = dist
                best_name = name
        return best_name

    def _recently_completed_penalty(self, grid: GridMap, cluster: FrontierCluster, state: VehicleState) -> float:
        if state.last_completed_xy is None:
            return 0.0
        wx, wy = grid.cell_to_world(cluster.viewpoint_cell)
        distance = math.hypot(wx - state.last_completed_xy[0], wy - state.last_completed_xy[1])
        radius = max(0.1, float(self.cfg['revisit_radius_m']))
        return float(self.cfg['revisit_penalty']) if distance < radius else 0.0

    def _candidate_plan(self, grid: GridMap, passable, state: VehicleState,
                        cluster: FrontierCluster) -> Optional[CandidatePlan]:
        start = self._vehicle_start_cell(grid, passable, state)
        if start is None:
            return None
        path = astar(passable, start, cluster.viewpoint_cell,
                     max_expansions=int(self.cfg['astar_max_expansions']))
        if path is None or len(path) < 2:
            return None
        cost = path_length_m(path, grid.resolution)
        if cost > float(self.cfg['max_assignment_path_m']):
            return None

        owner = self._zone_owner(grid, cluster)
        zone_bonus = float(self.cfg['zone_owner_bonus']) if owner == state.name else -float(self.cfg['zone_foreign_penalty'])
        recently_completed = self._recently_completed_penalty(grid, cluster, state)
        utility = (
            float(self.cfg['gain_weight']) * cluster.information_gain +
            float(self.cfg['frontier_weight']) * cluster.frontier_length_m -
            float(self.cfg['path_cost_weight']) * cost -
            float(self.cfg['risk_weight']) * cluster.risk +
            zone_bonus - recently_completed
        )
        return CandidatePlan(state.name, cluster, path, cost, utility)

    def _choose_assignments_locked(self, grid: GridMap, clusters: List[FrontierCluster], passable) -> List[CandidatePlan]:
        idle = [v for v in self.vehicles.values() if not v.active]
        if not idle or not clusters:
            return []
        candidate_by_robot: Dict[str, List[CandidatePlan]] = {}
        for state in idle:
            plans = []
            for cluster in clusters:
                plan = self._candidate_plan(grid, passable, state, cluster)
                if plan is not None:
                    plans.append(plan)
            candidate_by_robot[state.name] = sorted(plans, key=lambda p: p.utility, reverse=True)

        if len(idle) == 1:
            return candidate_by_robot[idle[0].name][:1]

        first, second = idle[0], idle[1]
        best_pair = None
        best_value = -float('inf')
        separation = float(self.cfg['min_assignment_separation_m'])
        for pa in candidate_by_robot[first.name]:
            ax, ay = grid.cell_to_world(pa.cluster.viewpoint_cell)
            for pb in candidate_by_robot[second.name]:
                if pa.cluster.cluster_id == pb.cluster.cluster_id:
                    continue
                bx, by = grid.cell_to_world(pb.cluster.viewpoint_cell)
                if math.hypot(ax - bx, ay - by) < separation:
                    continue
                imbalance = abs(pa.path_cost_m - pb.path_cost_m)
                score = pa.utility + pb.utility - float(self.cfg['load_balance_weight']) * imbalance
                if score > best_value:
                    best_value = score
                    best_pair = (pa, pb)
        if best_pair is not None:
            return [best_pair[0], best_pair[1]]

        # Fallback when only one non-overlapping frontier is reachable.
        all_plans = []
        for plans in candidate_by_robot.values():
            all_plans.extend(plans)
        return [max(all_plans, key=lambda p: p.utility)] if all_plans else []

    @staticmethod
    def _make_goal(frame: str, x: float, y: float, z: float) -> PoseStamped:
        msg = PoseStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = frame
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        msg.pose.orientation.w = 1.0
        return msg

    def _publish_path_locked(self, state: VehicleState, grid: GridMap):
        path_msg = Path()
        path_msg.header.stamp = rospy.Time.now()
        path_msg.header.frame_id = grid.frame_id
        for cell in state.queued_cells:
            x, y = grid.cell_to_world(cell)
            path_msg.poses.append(self._make_goal(grid.frame_id, x, y, state.altitude))
        self.path_pubs[state.name].publish(path_msg)

    def _publish_next_goal_locked(self, state: VehicleState, grid: GridMap):
        if state.goal_index >= len(state.queued_cells):
            self._finish_task_locked(state)
            return
        x, y = grid.cell_to_world(state.queued_cells[state.goal_index])
        goal = self._make_goal(grid.frame_id, x, y, state.altitude)
        self.goal_pubs[state.name].publish(goal)
        state.reached = False
        state.goal_sent_time = rospy.Time.now()
        self.status_pub.publish('%s -> %s waypoint %d/%d' % (
            state.name, state.task_id, state.goal_index + 1, len(state.queued_cells)))
        rospy.loginfo('%s RACER-style goal [%0.2f, %0.2f, %0.2f] for %s.',
                      state.name, x, y, state.altitude, state.task_id)

    def _activate_plan_locked(self, plan: CandidatePlan, grid: GridMap):
        state = self.vehicles[plan.vehicle]
        simplified = simplify_path(plan.path_cells, self._current_passable)
        # Remove the start cell; keep a sparse chain of safe turn points.
        queued = simplified[1:]
        if not queued:
            return
        state.active = True
        state.cluster_id = plan.cluster.cluster_id
        state.zone = plan.cluster.zone
        state.task_id = 'frontier_%03d' % plan.cluster.cluster_id
        state.queued_cells = queued
        state.goal_index = 0
        state.estimated_path_m = plan.path_cost_m
        self.zone_claims[plan.cluster.zone] = state.name
        self._publish_path_locked(state, grid)
        self._publish_next_goal_locked(state, grid)

    def _advance_path_locked(self, state: VehicleState):
        grid = self._grid_locked()
        if grid is None:
            return
        state.goal_index += 1
        if state.goal_index < len(state.queued_cells):
            self._publish_next_goal_locked(state, grid)
        else:
            self._finish_task_locked(state)

    def _finish_task_locked(self, state: VehicleState):
        grid = self._grid_locked()
        if grid is not None and state.queued_cells:
            last_cell = state.queued_cells[-1]
            state.last_completed_xy = grid.cell_to_world(last_cell)
        rospy.loginfo('%s completed %s.', state.name, state.task_id)
        state.active = False
        state.task_id = ''
        state.cluster_id = -1
        state.zone = None
        state.queued_cells = []
        state.goal_index = 0
        state.estimated_path_m = 0.0
        state.goal_sent_time = rospy.Time(0)

    def _timeout_active_goals_locked(self, now: rospy.Time):
        timeout = float(self.cfg['mission_goal_timeout_sec'])
        if timeout <= 0:
            return
        for state in self.vehicles.values():
            if not state.active or state.goal_sent_time.is_zero():
                continue
            if (now - state.goal_sent_time).to_sec() > timeout:
                rospy.logwarn('%s goal timeout in %s; releasing the task for replanning.', state.name, state.task_id)
                self._finish_task_locked(state)

    def _task_markers_locked(self, grid: GridMap, clusters: List[FrontierCluster]) -> MarkerArray:
        markers = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)
        stamp = rospy.Time.now()

        # Frontier viewpoints in blue.
        for cluster in clusters:
            x, y = grid.cell_to_world(cluster.viewpoint_cell)
            m = Marker()
            m.header.stamp = stamp
            m.header.frame_id = grid.frame_id
            m.ns = 'racer_frontiers'
            m.id = 1000 + int(cluster.cluster_id)
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = x
            m.pose.position.y = y
            m.pose.position.z = 0.18
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 0.22
            m.color.r, m.color.g, m.color.b, m.color.a = 0.05, 0.55, 1.0, 0.65
            markers.markers.append(m)

        colours = {'uav0': (0.1, 0.9, 1.0), 'uav1': (1.0, 0.25, 0.9)}
        for idx, state in enumerate(self.vehicles.values()):
            if not state.active or not state.queued_cells:
                continue
            x, y = grid.cell_to_world(state.queued_cells[-1])
            r, g, b = colours.get(state.name, (1.0, 1.0, 0.1))
            m = Marker()
            m.header.stamp = stamp
            m.header.frame_id = grid.frame_id
            m.ns = 'racer_assignments'
            m.id = 2000 + idx
            m.type = Marker.CYLINDER
            m.action = Marker.ADD
            m.pose.position.x = x
            m.pose.position.y = y
            m.pose.position.z = state.altitude
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = 0.50
            m.scale.z = 0.12
            m.color.r, m.color.g, m.color.b, m.color.a = r, g, b, 0.90
            markers.markers.append(m)

            label = Marker()
            label.header = m.header
            label.ns = 'racer_assignment_labels'
            label.id = 2100 + idx
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose = m.pose
            label.pose.position.z += 0.45
            label.scale.z = 0.28
            label.color.r = label.color.g = label.color.b = label.color.a = 1.0
            label.text = '%s\n%s\n%.1f m' % (state.name, state.task_id, state.estimated_path_m)
            markers.markers.append(label)
        return markers

    def _should_plan_locked(self, now: rospy.Time, grid: GridMap) -> bool:
        if self.mission_finished:
            return False
        if all(v.active for v in self.vehicles.values()):
            return False
        if self.last_plan_time.is_zero():
            return True
        if (now - self.last_plan_time).to_sec() >= float(self.cfg['replan_period_sec']):
            return True
        known = self._known_cells(grid)
        change_threshold = int(self.cfg['map_change_cells_for_replan'])
        return abs(known - self.known_cells_at_last_plan) >= change_threshold

    def _tick(self, _event):
        now = rospy.Time.now()
        with self.lock:
            grid = self._grid_locked()
            ready, state = self._ready_locked(grid, now)
            if not ready:
                self.status_pub.publish(state)
                return
            if self.mission_finished:
                self.status_pub.publish('TARGET_CONFIRMED' if self.confirmed_target_seen else 'MISSION_COMPLETE')
                return
            self._timeout_active_goals_locked(now)
            if not self._should_plan_locked(now, grid):
                self.marker_pub.publish(self._task_markers_locked(grid, self.latest_clusters))
                return

            try:
                clusters, passable = extract_frontier_clusters(
                    grid,
                    occupied_threshold=int(self.cfg['occupied_threshold']),
                    inflation_radius_m=float(self.cfg['obstacle_inflation_m']),
                    min_frontier_length_m=float(self.cfg['min_frontier_length_m']),
                    gain_radius_m=float(self.cfg['gain_radius_m']),
                    min_clearance_m=float(self.cfg['min_clearance_m']),
                    hgrid_size_m=float(self.cfg['hgrid_size_m']),
                    sample_stride=int(self.cfg['frontier_sample_stride']),
                )
            except Exception as exc:
                rospy.logwarn_throttle(3.0, 'Stage-3 frontier planning failed: %r', exc)
                self.status_pub.publish('PLANNING_ERROR')
                return

            self.latest_clusters = clusters
            self._current_passable = passable
            if not clusters:
                known_fraction = float(self._known_cells(grid)) / float(max(1, grid.width * grid.height))
                if (not any(v.active for v in self.vehicles.values()) and
                        known_fraction >= float(self.cfg.get('min_completion_known_fraction', 0.85))):
                    self.mission_finished = True
                    self.status_pub.publish('MISSION_COMPLETE_NO_FRONTIER')
                    rospy.loginfo('No valid frontier remains after %.1f%% map coverage. Stage-3 exploration is complete.',
                                  100.0 * known_fraction)
                else:
                    self.status_pub.publish('WAIT_FRONTIER: known=%.1f%%' % (100.0 * known_fraction))
                return

            assignments = self._choose_assignments_locked(grid, clusters, passable)
            for plan in assignments:
                if not self.vehicles[plan.vehicle].active:
                    self._activate_plan_locked(plan, grid)
            self.last_plan_time = now
            self.known_cells_at_last_plan = self._known_cells(grid)
            marker_array = self._task_markers_locked(grid, clusters)
            self.marker_pub.publish(marker_array)
            self.assignment_pub.publish(marker_array)
            self.status_pub.publish('EXPLORE: %d frontiers, %d new assignments' % (len(clusters), len(assignments)))
            rospy.loginfo('Stage-3 plan: %d frontiers; assignments=%s.', len(clusters),
                          [(p.vehicle, p.cluster.cluster_id, round(p.path_cost_m, 1), round(p.utility, 1))
                           for p in assignments])


def main():
    rospy.init_node('autonomous_search_manager')
    AutonomousSearchManager()
    rospy.spin()


if __name__ == '__main__':
    main()
