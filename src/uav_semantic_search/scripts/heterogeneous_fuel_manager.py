#!/usr/bin/env python3
"""Centralized FUEL-style heterogeneous explorer for two UAVs and one Go2 proxy.

Design:
  * UAVs retain the existing fixed-height exploration layer and plan on
    /global_map_2d, which is built only from UAV LiDAR.
  * The navigation-level Go2 plans separately on /ugv0/ground_map_2d, built from
    its low-mounted LiDAR and low-obstacle height band.
  * All agents use the same frontier -> safe viewpoint -> 2D A* -> waypoint chain
    loop, but each group uses its own traversability map and clearance constraints.

This is a centralized, FUEL-inspired operational baseline. It does not claim to
replicate the full original FUEL trajectory optimizer or Go2 gait dynamics.
"""
from __future__ import annotations

import math
import os
import sys
import threading
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if _SCRIPT_DIR not in sys.path:
    sys.path.insert(0, _SCRIPT_DIR)

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from std_msgs.msg import Bool, String
from visualization_msgs.msg import Marker, MarkerArray

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
from uav_semantic_search.msg import TargetHypothesisArray


@dataclass
class AgentState:
    name: str
    kind: str                         # 'uav' or 'ugv'
    map_key: str                      # 'uav' or 'ugv'
    height: float
    pose: Optional[PoseStamped] = None
    takeoff_ready: bool = False
    blocked: bool = False
    blocked_since: rospy.Time = field(default_factory=lambda: rospy.Time(0))
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
class Candidate:
    agent: str
    cluster: FrontierCluster
    path: List[Tuple[int, int]]
    cost_m: float
    utility: float


class HeterogeneousFUELManager:
    def __init__(self):
        self.base = rospy.get_param('/racer_stage3')
        self.cfg = rospy.get_param('/heterogeneous_fuel')
        self.map_frame = rospy.get_param('/map/frame_id', 'map')
        self.lock = threading.RLock()
        self.finished = False
        self.confirmed_target = False
        self.maps: Dict[str, Optional[OccupancyGrid]] = {'uav': None, 'ugv': None}
        self.last_plan_time = {'uav': rospy.Time(0), 'ugv': rospy.Time(0)}
        self.known_at_plan = {'uav': 0, 'ugv': 0}
        self.zone_claims: Dict[str, Dict[Tuple[int, int], str]] = {'uav': {}, 'ugv': {}}
        self.uav_ready_time = rospy.Time(0)
        self.ugv_ready_time = rospy.Time(0)

        # The Go2 launch condition is a one-shot startup gate, not a persistent
        # dependency on the UAV mission state. Once every UAV has reached and
        # held its fixed takeoff altitude, the gate remains open for the rest of
        # the mission, even after UAV frontier exploration is exhausted.
        self.ugv_start_gate_open = not bool(
            self.cfg.get('ugv_wait_for_uav_takeoff', True))

        self.agents: Dict[str, AgentState] = {}
        self.goal_pubs = {}
        self.path_pubs = {}
        self.group_cfg = {'uav': self._uav_profile(), 'ugv': self._ugv_profile()}

        for vehicle in rospy.get_param('/vehicles', []):
            name = vehicle['name']
            self._register_agent(
                name=name, kind='uav', map_key='uav', height=float(vehicle.get('takeoff_height', 1.8)),
                pose_topic=vehicle['global_pose_topic'], goal_topic=vehicle['mission_goal_topic'],
                reached_topic='/%s/mission/reached' % name, path_topic='/%s/search/planned_path' % name,
                takeoff_topic='/%s/mission/takeoff_ready' % name,
            )

        for robot in rospy.get_param('/ground_robots', []):
            name = robot['name']
            self._register_agent(
                name=name, kind='ugv', map_key='ugv', height=float(robot.get('base_height_m', 0.34)),
                pose_topic=robot['global_pose_topic'], goal_topic=robot['mission_goal_topic'],
                reached_topic=robot['mission_reached_topic'], path_topic=robot['planned_path_topic'],
                blocked_topic=robot['blocked_topic'],
            )

        if not self.agents:
            raise RuntimeError('No UAV or ground robot configured.')

        rospy.Subscriber(self.cfg.get('uav_map_topic', '/global_map_2d'), OccupancyGrid,
                         lambda m: self._map_cb('uav', m), queue_size=1)
        # The first configured ground robot determines the map topic for this baseline.
        ground_robots = rospy.get_param('/ground_robots', [])
        if ground_robots:
            rospy.Subscriber(ground_robots[0]['ground_map_topic'], OccupancyGrid,
                             lambda m: self._map_cb('ugv', m), queue_size=1)
        rospy.Subscriber('/semantic_map/confirmed_targets', TargetHypothesisArray,
                         self._confirmed_cb, queue_size=5)

        self.marker_pub = rospy.Publisher('/search/heterogeneous_fuel_markers', MarkerArray, queue_size=1, latch=True)
        self.status_pub = rospy.Publisher('/search/heterogeneous_status', String, queue_size=10, latch=True)
        self.status_pub.publish('WAIT_READY')
        rate = max(1.0, float(self.cfg.get('manager_tick_hz', 2.0)))
        rospy.Timer(rospy.Duration(1.0 / rate), self._tick)
        rospy.loginfo('Heterogeneous FUEL-style manager ready for %s.', list(self.agents))

    def _uav_profile(self):
        return {
            'occupied_threshold': int(self.base['occupied_threshold']),
            'obstacle_inflation_m': float(self.base['obstacle_inflation_m']),
            'min_clearance_m': float(self.base['min_clearance_m']),
            'min_frontier_length_m': float(self.base['min_frontier_length_m']),
            'gain_radius_m': float(self.base['gain_radius_m']),
            'sample_stride': int(self.base['frontier_sample_stride']),
            'hgrid_size_m': float(self.base['hgrid_size_m']),
            'max_path_m': float(self.base['max_assignment_path_m']),
            'nearest_free_cells': int(self.base['nearest_free_search_cells']),
            'astar_max_expansions': int(self.base['astar_max_expansions']),
            'gain_weight': float(self.base['gain_weight']),
            'frontier_weight': float(self.base['frontier_weight']),
            'path_cost_weight': float(self.base['path_cost_weight']),
            'risk_weight': float(self.base['risk_weight']),
            'revisit_radius_m': float(self.base['revisit_radius_m']),
            'revisit_penalty': float(self.base['revisit_penalty']),
            'zone_owner_bonus': float(self.base['zone_owner_bonus']),
            'zone_foreign_penalty': float(self.base['zone_foreign_penalty']),
            'min_separation_m': float(self.base['min_assignment_separation_m']),
            'load_balance_weight': float(self.base['load_balance_weight']),
            'min_known': int(self.cfg.get('uav_map_bootstrap_known_cells', self.base['min_known_cells'])),
        }

    def _ugv_profile(self):
        return {
            'occupied_threshold': int(self.base['occupied_threshold']),
            'obstacle_inflation_m': float(self.cfg['ugv_obstacle_inflation_m']),
            'min_clearance_m': float(self.cfg['ugv_min_clearance_m']),
            'min_frontier_length_m': float(self.cfg['ugv_min_frontier_length_m']),
            'gain_radius_m': float(self.cfg['ugv_gain_radius_m']),
            'sample_stride': int(self.cfg['ugv_frontier_sample_stride']),
            'hgrid_size_m': float(self.cfg['ugv_hgrid_size_m']),
            'max_path_m': float(self.cfg['ugv_max_assignment_path_m']),
            'nearest_free_cells': int(self.cfg['ugv_nearest_free_search_cells']),
            'astar_max_expansions': int(self.cfg['ugv_astar_max_expansions']),
            'gain_weight': float(self.cfg['ugv_gain_weight']),
            'frontier_weight': float(self.cfg['ugv_frontier_weight']),
            'path_cost_weight': float(self.cfg['ugv_path_cost_weight']),
            'risk_weight': float(self.cfg['ugv_risk_weight']),
            'revisit_radius_m': float(self.cfg['ugv_revisit_radius_m']),
            'revisit_penalty': float(self.cfg['ugv_revisit_penalty']),
            'zone_owner_bonus': 0.0,
            'zone_foreign_penalty': 0.0,
            'min_separation_m': 0.0,
            'load_balance_weight': 0.0,
            'min_known': int(self.cfg['ugv_map_bootstrap_known_cells']),
        }

    def _register_agent(self, name, kind, map_key, height, pose_topic, goal_topic,
                        reached_topic, path_topic, takeoff_topic=None, blocked_topic=None):
        state = AgentState(name=name, kind=kind, map_key=map_key, height=height,
                           takeoff_ready=(kind == 'ugv'))
        self.agents[name] = state
        self.goal_pubs[name] = rospy.Publisher(goal_topic, PoseStamped, queue_size=3, latch=True)
        self.path_pubs[name] = rospy.Publisher(path_topic, Path, queue_size=1, latch=True)
        rospy.Subscriber(pose_topic, PoseStamped, lambda m, n=name: self._pose_cb(n, m), queue_size=20)
        rospy.Subscriber(reached_topic, Bool, lambda m, n=name: self._reached_cb(n, m), queue_size=10)
        if takeoff_topic:
            rospy.Subscriber(takeoff_topic, Bool, lambda m, n=name: self._takeoff_cb(n, m), queue_size=2)
        if blocked_topic:
            rospy.Subscriber(blocked_topic, Bool, lambda m, n=name: self._blocked_cb(n, m), queue_size=10)

    def _pose_cb(self, name, msg):
        with self.lock:
            self.agents[name].pose = msg

    def _takeoff_cb(self, name, msg):
        with self.lock:
            self.agents[name].takeoff_ready = bool(msg.data)
            if not msg.data:
                # Before the initial release, a lost takeoff certification must
                # reset the startup timers. After the one-shot Go2 launch gate
                # has opened, UAV completion, hovering, or a later transient
                # takeoff-ready update must not freeze ground exploration.
                self.uav_ready_time = rospy.Time(0)
                if not self.ugv_start_gate_open:
                    self.ugv_ready_time = rospy.Time(0)

    def _blocked_cb(self, name, msg):
        with self.lock:
            state = self.agents[name]
            blocked = bool(msg.data)
            if blocked and not state.blocked:
                state.blocked_since = rospy.Time.now()
                rospy.logwarn('%s reports front translation blockage; allow local escape rotation first.', name)
            elif not blocked:
                state.blocked_since = rospy.Time(0)
            state.blocked = blocked

    def _reached_cb(self, name, msg):
        if not msg.data:
            return
        with self.lock:
            state = self.agents[name]
            if state.active:
                self._advance_locked(state)

    def _map_cb(self, key, msg):
        with self.lock:
            self.maps[key] = msg

    def _confirmed_cb(self, msg):
        if not bool(self.cfg.get('stop_on_confirmed_target', True)) or not msg.hypotheses:
            return
        with self.lock:
            self.finished = True
            self.confirmed_target = True
            for state in self.agents.values():
                if state.active:
                    self._finish_locked(state, self._grid_locked(state.map_key))
                else:
                    self._publish_hold_locked(state)
            self.status_pub.publish('TARGET_CONFIRMED_HOLD')
            rospy.logwarn('Confirmed target received: FUEL manager holds all agents and stops assigning new tasks.')

    def _grid_locked(self, key):
        msg = self.maps.get(key)
        if msg is None or not msg.data:
            return None
        return occupancy_grid_from_flat(msg.data, msg.info.width, msg.info.height, msg.info.resolution,
                                        msg.info.origin.position.x, msg.info.origin.position.y,
                                        msg.header.frame_id or self.map_frame)

    @staticmethod
    def _known_cells(grid):
        return int((grid.data >= 0).sum())

    def _group_agents(self, key):
        return [state for state in self.agents.values() if state.map_key == key]

    def _all_uavs_takeoff_ready_locked(self):
        """Return True only when every configured UAV passed its takeoff gate."""
        uavs = [state for state in self.agents.values() if state.kind == 'uav']
        return bool(uavs) and all(state.takeoff_ready for state in uavs)

    def _group_ready_locked(self, key, now):
        # Go2 waits for the UAV takeoff gate only once, at mission startup.
        # Crucially, this is not checked as a continuous dependency after the
        # ground robot has been released: UAV completion is a local group state
        # and must never terminate or suspend unfinished ground exploration.
        if (key == 'ugv' and
                bool(self.cfg.get('ugv_wait_for_uav_takeoff', True)) and
                not self.ugv_start_gate_open):
            if not self._all_uavs_takeoff_ready_locked():
                self.ugv_ready_time = rospy.Time(0)
                return False, 'UGV_WAIT_UAV_TAKEOFF', self._grid_locked(key)

            self.ugv_start_gate_open = True
            self.ugv_ready_time = rospy.Time(0)
            rospy.loginfo(
                'All UAVs passed the initial fixed-altitude gate; Go2 ground '
                'exploration is permanently released from the startup gate.')

        grid = self._grid_locked(key)
        profile = self.group_cfg[key]
        group = self._group_agents(key)
        if not group:
            return False, 'NO_%s' % key.upper(), grid
        if grid is None:
            return False, '%s_WAIT_MAP' % key.upper(), None
        if any(state.pose is None for state in group):
            return False, '%s_WAIT_POSE' % key.upper(), grid

        if key == 'uav':
            if any(not state.takeoff_ready for state in group):
                return False, 'UAV_WAIT_TAKEOFF', grid
            if self.uav_ready_time.is_zero():
                self.uav_ready_time = now
                rospy.loginfo('All UAVs takeoff-ready; start FUEL map bootstrap.')
            delay = float(self.cfg.get('post_takeoff_map_bootstrap_sec', 6.0))
            if (now - self.uav_ready_time).to_sec() < delay:
                return False, 'UAV_MAP_BOOTSTRAP', grid

        if key == 'ugv':
            if self.ugv_ready_time.is_zero():
                self.ugv_ready_time = now
                rospy.loginfo(
                    'All UAVs takeoff-ready; Go2 remains stationary and starts '
                    'its synchronized ground-map bootstrap scan.')

            release_delay = float(
                self.cfg.get('ugv_post_uav_takeoff_delay_sec', 0.0))
            if (now - self.ugv_ready_time).to_sec() < release_delay:
                return False, 'UGV_WAIT_UAV_STABLE', grid

            scan_delay = float(self.cfg.get('ugv_initial_scan_sec', 6.0))
            if (now - self.ugv_ready_time).to_sec() < scan_delay:
                return False, 'UGV_INITIAL_SCAN', grid

        if self._known_cells(grid) < profile['min_known']:
            return False, '%s_MAP_BOOTSTRAP' % key.upper(), grid
        return True, '%s_EXPLORE' % key.upper(), grid

    def _start_cell(self, grid, passable, state, profile):
        if state.pose is None:
            return None
        p = state.pose.pose.position
        cell = grid.world_to_cell(p.x, p.y)
        return None if cell is None else nearest_passable(passable, cell, int(profile['nearest_free_cells']))

    def _zone_owner(self, key, grid, cluster, group):
        if cluster.zone in self.zone_claims[key]:
            return self.zone_claims[key][cluster.zone]
        wx, wy = grid.cell_to_world(cluster.viewpoint_cell)
        best_name, best_distance = None, float('inf')
        for state in group:
            if state.pose is None:
                continue
            p = state.pose.pose.position
            distance = math.hypot(wx - p.x, wy - p.y)
            if distance < best_distance:
                best_name, best_distance = state.name, distance
        return best_name

    def _candidate(self, key, grid, passable, state, cluster, profile, group):
        start = self._start_cell(grid, passable, state, profile)
        if start is None:
            return None
        path = astar(passable, start, cluster.viewpoint_cell, int(profile['astar_max_expansions']))
        if path is None or len(path) < 2:
            return None
        cost = path_length_m(path, grid.resolution)
        if cost > profile['max_path_m']:
            return None
        revisit = 0.0
        if state.last_completed_xy is not None:
            wx, wy = grid.cell_to_world(cluster.viewpoint_cell)
            if math.hypot(wx - state.last_completed_xy[0], wy - state.last_completed_xy[1]) < profile['revisit_radius_m']:
                revisit = profile['revisit_penalty']
        owner = self._zone_owner(key, grid, cluster, group)
        zone_term = profile['zone_owner_bonus'] if owner == state.name else -profile['zone_foreign_penalty']
        utility = (profile['gain_weight'] * cluster.information_gain +
                   profile['frontier_weight'] * cluster.frontier_length_m -
                   profile['path_cost_weight'] * cost -
                   profile['risk_weight'] * cluster.risk + zone_term - revisit)
        return Candidate(state.name, cluster, path, cost, utility)

    def _inside_ground_boundary(self, x, y):
        """Return whether a UGV candidate lies inside the configured mission ROI."""
        boundary = self.cfg.get('ground_exploration_boundary', {})

        if not boundary or not bool(boundary.get('enabled', True)):
            return True

        margin = float(boundary.get('margin_m', 0.0))
        return (
            float(boundary['x_min']) + margin
            <= x <=
            float(boundary['x_max']) - margin
            and
            float(boundary['y_min']) + margin
            <= y <=
            float(boundary['y_max']) - margin
        )

    def _restrict_ground_passable(self, grid, passable):
        """Prevent 2D A* from routing a ground robot outside the operational ROI."""
        boundary = self.cfg.get('ground_exploration_boundary', {})

        if not boundary or not bool(boundary.get('enabled', True)):
            return passable

        bounded = passable.copy()
        for y in range(grid.height):
            for x in range(grid.width):
                if not bounded[y, x]:
                    continue
                wx, wy = grid.cell_to_world((x, y))
                if not self._inside_ground_boundary(wx, wy):
                    bounded[y, x] = False
        return bounded

    def _extract_clusters(self, key, grid):
        profile = self.group_cfg[key]
        return extract_frontier_clusters(
            grid, occupied_threshold=profile['occupied_threshold'],
            inflation_radius_m=profile['obstacle_inflation_m'],
            min_frontier_length_m=profile['min_frontier_length_m'],
            gain_radius_m=profile['gain_radius_m'], min_clearance_m=profile['min_clearance_m'],
            hgrid_size_m=profile['hgrid_size_m'], sample_stride=profile['sample_stride'])

    def _choose(self, key, grid, clusters, passable):
        profile = self.group_cfg[key]
        group = self._group_agents(key)
        idle = [state for state in group if not state.active]
        if not idle or not clusters:
            return []
        all_candidates: Dict[str, List[Candidate]] = {}
        for state in idle:
            candidates = [c for cluster in clusters
                          if (c := self._candidate(key, grid, passable, state, cluster, profile, group)) is not None]
            all_candidates[state.name] = sorted(candidates, key=lambda item: item.utility, reverse=True)
        if key == 'ugv' or len(idle) == 1:
            candidates = all_candidates[idle[0].name]
            return candidates[:1]
        # Joint assignment for exactly the two aerial robots; preserves FUEL/RACER-style
        # duplicate avoidance and path-load balancing on the shared UAV layer.
        a, b = idle[0], idle[1]
        best_pair, best_score = None, -float('inf')
        for ca in all_candidates[a.name]:
            ax, ay = grid.cell_to_world(ca.cluster.viewpoint_cell)
            for cb in all_candidates[b.name]:
                if ca.cluster.cluster_id == cb.cluster.cluster_id:
                    continue
                bx, by = grid.cell_to_world(cb.cluster.viewpoint_cell)
                if math.hypot(ax - bx, ay - by) < profile['min_separation_m']:
                    continue
                score = ca.utility + cb.utility - profile['load_balance_weight'] * abs(ca.cost_m - cb.cost_m)
                if score > best_score:
                    best_pair, best_score = (ca, cb), score
        if best_pair is not None:
            return list(best_pair)
        flat = [candidate for values in all_candidates.values() for candidate in values]
        return [max(flat, key=lambda item: item.utility)] if flat else []

    @staticmethod
    def _goal(frame, x, y, z):
        msg = PoseStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = frame
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        msg.pose.orientation.w = 1.0
        return msg

    def _publish_path_locked(self, state, grid):
        msg = Path()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = grid.frame_id
        for cell in state.queued_cells:
            x, y = grid.cell_to_world(cell)
            msg.poses.append(self._goal(grid.frame_id, x, y, state.height))
        self.path_pubs[state.name].publish(msg)

    def _send_next_locked(self, state, grid):
        if state.goal_index >= len(state.queued_cells):
            self._finish_locked(state, grid)
            return
        x, y = grid.cell_to_world(state.queued_cells[state.goal_index])
        self.goal_pubs[state.name].publish(self._goal(grid.frame_id, x, y, state.height))
        state.goal_sent_time = rospy.Time.now()
        rospy.loginfo('%s FUEL-style goal [%.2f, %.2f, %.2f], %s %d/%d.', state.name, x, y,
                      state.height, state.task_id, state.goal_index + 1, len(state.queued_cells))

    def _publish_hold_locked(self, state):
        """Replace any stale waypoint by the agent's current map-frame position."""
        if state.pose is None:
            return
        p = state.pose.pose.position
        frame = state.pose.header.frame_id or self.map_frame
        self.goal_pubs[state.name].publish(self._goal(frame, p.x, p.y, state.height))

    def _activate_locked(self, candidate, grid, passable):
        state = self.agents[candidate.agent]
        simplified = simplify_path(candidate.path, passable)
        queued = simplified[1:]
        if not queued:
            return
        state.active = True
        # Keep the physical front-gate state reported by the controller. It may
        # clear naturally after an in-place escape rotation; forcing it false here
        # creates an inconsistent high-level state.
        state.task_id = '%s_frontier_%03d' % (state.kind, candidate.cluster.cluster_id)
        state.cluster_id = candidate.cluster.cluster_id
        state.zone = candidate.cluster.zone
        state.queued_cells = queued
        state.goal_index = 0
        state.estimated_path_m = candidate.cost_m
        self.zone_claims[state.map_key][candidate.cluster.zone] = state.name
        self._publish_path_locked(state, grid)
        self._send_next_locked(state, grid)

    def _advance_locked(self, state):
        grid = self._grid_locked(state.map_key)
        if grid is None:
            return
        state.goal_index += 1
        if state.goal_index < len(state.queued_cells):
            self._send_next_locked(state, grid)
        else:
            self._finish_locked(state, grid)

    def _finish_locked(self, state, grid):
        if not state.active:
            return
        # Record the actual stop position. The original code recorded the final
        # queued waypoint even when a task was cancelled by a blockage or timeout.
        if state.pose is not None:
            p = state.pose.pose.position
            state.last_completed_xy = (p.x, p.y)
        elif grid is not None and state.queued_cells:
            state.last_completed_xy = grid.cell_to_world(state.queued_cells[-1])
        rospy.loginfo('%s completed/released %s.', state.name, state.task_id)
        self._publish_hold_locked(state)
        state.active = False
        state.task_id = ''
        state.cluster_id = -1
        state.zone = None
        state.queued_cells = []
        state.goal_index = 0
        state.goal_sent_time = rospy.Time(0)
        state.estimated_path_m = 0.0

    def _timeout_locked(self, now):
        timeout = float(self.cfg.get('mission_goal_timeout_sec', 70.0))
        for state in self.agents.values():
            if not state.active or state.goal_sent_time.is_zero():
                continue
            if (now - state.goal_sent_time).to_sec() > timeout:
                rospy.logwarn('%s task timeout; release for replanning.', state.name)
                self._finish_locked(state, self._grid_locked(state.map_key))

    def _release_persistently_blocked_locked(self, now):
        """Escalate only a *persistent* UGV blockage to high-level replanning.

        A single front-sector hit is often caused by the robot initially facing a
        wall in a corridor. The local executor first gets a chance to rotate away.
        Only if the gate remains asserted for the configured duration is the
        assigned task released for a new frontier/path decision.
        """
        delay = float(self.cfg.get('ugv_persistent_block_replan_sec', 2.5))
        for state in self._group_agents('ugv'):
            if (not state.active or not state.blocked or
                    state.blocked_since.is_zero()):
                continue
            if (now - state.blocked_since).to_sec() >= delay:
                rospy.logwarn(
                    '%s remains front-blocked for %.1fs; release %s for replanning.',
                    state.name, delay, state.task_id)
                self._finish_locked(state, self._grid_locked(state.map_key))
                # Keep the physical blocked state; only the controller may clear it.
                state.blocked_since = now

    def _needs_plan(self, key, now, grid):
        group = self._group_agents(key)
        if not any(not state.active for state in group):
            return False
        elapsed = self.last_plan_time[key].is_zero() or (now - self.last_plan_time[key]).to_sec() >= float(self.cfg.get('replan_period_sec', 3.0))
        known = self._known_cells(grid)
        change = abs(known - self.known_at_plan[key]) >= int(self.base.get('map_change_cells_for_replan', 80))
        return elapsed or change

    def _marker_array(self, views):
        markers = MarkerArray()
        clear = Marker(); clear.action = Marker.DELETEALL; markers.markers.append(clear)
        stamp = rospy.Time.now()
        colour = {'uav': (0.08, 0.55, 1.0), 'ugv': (0.10, 0.95, 0.30)}
        for key, grid, clusters in views:
            r, g, b = colour[key]
            z = float(self.cfg.get('marker_z_uav_m' if key == 'uav' else 'marker_z_ugv_m', 0.25))
            for cluster in clusters:
                x, y = grid.cell_to_world(cluster.viewpoint_cell)
                m = Marker(); m.header.stamp = stamp; m.header.frame_id = grid.frame_id
                m.ns = '%s_fuel_frontiers' % key; m.id = (0 if key == 'uav' else 10000) + cluster.cluster_id
                m.type = Marker.SPHERE; m.action = Marker.ADD; m.pose.position.x = x; m.pose.position.y = y; m.pose.position.z = z; m.pose.orientation.w = 1.0
                m.scale.x = m.scale.y = m.scale.z = 0.20; m.color.r = r; m.color.g = g; m.color.b = b; m.color.a = 0.65
                markers.markers.append(m)
        active_colours = {'uav0': (0.10, 0.90, 1.0), 'uav1': (1.0, 0.25, 0.90), 'ugv0': (0.15, 1.0, 0.25)}
        for idx, state in enumerate(self.agents.values()):
            if not state.active or not state.queued_cells:
                continue
            grid = self._grid_locked(state.map_key)
            if grid is None:
                continue
            x, y = grid.cell_to_world(state.queued_cells[-1])
            r, g, b = active_colours.get(state.name, (1.0, 1.0, 0.1))
            m = Marker(); m.header.stamp = stamp; m.header.frame_id = grid.frame_id
            m.ns = 'heterogeneous_assignments'; m.id = 20000 + idx; m.type = Marker.CYLINDER; m.action = Marker.ADD
            m.pose.position.x = x; m.pose.position.y = y; m.pose.position.z = state.height; m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = 0.48; m.scale.z = 0.12; m.color.r = r; m.color.g = g; m.color.b = b; m.color.a = 0.90
            markers.markers.append(m)
        return markers

    def _tick(self, _event):
        now = rospy.Time.now()
        with self.lock:
            if self.finished:
                self.status_pub.publish('TARGET_CONFIRMED_HOLD' if self.confirmed_target else 'MISSION_COMPLETE')
                return
            self._timeout_locked(now)
            self._release_persistently_blocked_locked(now)
            views = []
            summary = []
            for key in ('uav', 'ugv'):
                ready, state_text, grid = self._group_ready_locked(key, now)
                if not ready:
                    summary.append(state_text)
                    continue
                try:
                    clusters, passable = self._extract_clusters(key, grid)
                except Exception as exc:
                    rospy.logwarn_throttle(3.0, '%s frontier extraction error: %r', key, exc)
                    summary.append('%s_ERROR' % key.upper())
                    continue
                if key == 'ugv':
                    # Do not treat the open exterior/map canvas west of the corridor
                    # as an indoor frontier, and do not let A* route Go2 through it.
                    passable = self._restrict_ground_passable(grid, passable)
                    clusters = [
                        cluster for cluster in clusters
                        if self._inside_ground_boundary(*grid.cell_to_world(cluster.viewpoint_cell))
                        and passable[cluster.viewpoint_cell[1], cluster.viewpoint_cell[0]]
                    ]
                views.append((key, grid, clusters))

                # Frontier exhaustion is a group-local condition. In particular,
                # UAV completion must not set self.finished or inhibit UGV
                # replanning. self.finished remains reserved for a confirmed
                # target / global mission-termination event.
                if not clusters and all(
                        not state.active for state in self._group_agents(key)):
                    if key == 'uav':
                        summary.append('UAV_COMPLETE_NO_FRONTIER')
                    else:
                        summary.append('UGV_NO_REACHABLE_FRONTIER')
                else:
                    summary.append('%s: %d frontiers' % (
                        key.upper(), len(clusters)))

                if self._needs_plan(key, now, grid):
                    assignments = self._choose(key, grid, clusters, passable)
                    for candidate in assignments:
                        self._activate_locked(candidate, grid, passable)
                    self.last_plan_time[key] = now
                    self.known_at_plan[key] = self._known_cells(grid)
                    rospy.loginfo('%s FUEL plan: %d frontiers, %d new assignments.', key.upper(), len(clusters), len(assignments))
            self.marker_pub.publish(self._marker_array(views))
            self.status_pub.publish(' | '.join(summary) if summary else 'WAIT_READY')


def main():
    rospy.init_node('heterogeneous_fuel_manager')
    HeterogeneousFUELManager()
    rospy.spin()


if __name__ == '__main__':
    main()