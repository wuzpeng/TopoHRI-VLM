#!/usr/bin/env python3
"""PX4 OFFBOARD executor with post-selection A* route waypoint tracking.

Stage-5 normal operation receives a map-frame ``nav_msgs/Path`` from the
geometry route planner. The VLM therefore selects only an endpoint candidate;
this executor follows the current-map A* waypoints. The legacy ``mission_goal``
input remains available for manual tests and non-Stage-5 components.
"""
from __future__ import annotations

import copy
import json
import math
import threading
import time
from typing import List, Optional

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
from nav_msgs.msg import Path
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool, String
import sensor_msgs.point_cloud2 as pc2
from tf.transformations import euler_from_quaternion, quaternion_from_euler


class WaypointExecutor:
    def __init__(self):
        self.vehicle = rospy.get_param('~vehicle')
        self.autostart = bool(rospy.get_param('~autostart', True))
        self.arrival_tolerance = float(rospy.get_param('~arrival_tolerance', 0.35))
        self.request_period = float(rospy.get_param('~request_period', 1.5))

        vehicles = {item['name']: item for item in rospy.get_param('/vehicles', [])}
        if self.vehicle not in vehicles:
            raise RuntimeError('Unknown vehicle: %s' % self.vehicle)
        self.cfg = vehicles[self.vehicle]
        self.takeoff_height = float(self.cfg.get('takeoff_height', 1.8))
        self.takeoff_tolerance = float(self.cfg.get('takeoff_tolerance_m', 0.18))
        self.takeoff_hold_sec = float(self.cfg.get('takeoff_hold_sec', 2.0))
        runtime = rospy.get_param('/experiment_runtime', {})
        yaw_cfg = dict(runtime.get('uav_yaw_control', {}))
        yaw_cfg.update(self.cfg.get('yaw_control', {}) or {})
        self.yaw_follow_enabled = bool(yaw_cfg.get('enabled', True))
        self.max_yaw_rate = max(0.05, float(yaw_cfg.get('max_yaw_rate_rad_s', 0.55)))
        self.yaw_tolerance = max(0.0, float(yaw_cfg.get('yaw_tolerance_rad', 0.08)))
        self.rotate_before_translate = bool(
            yaw_cfg.get('rotate_before_translate', True))
        self.rotate_before_move_threshold = max(
            self.yaw_tolerance,
            float(yaw_cfg.get('rotate_before_move_threshold_rad', 0.30)))
        self.final_yaw_tolerance = max(
            self.yaw_tolerance,
            float(yaw_cfg.get('final_yaw_tolerance_rad', 0.14)))
        self.final_observation_dwell_sec = max(
            0.0, float(yaw_cfg.get('final_observation_dwell_sec', 1.0)))
        self.path_yaw_lookahead_m = max(
            0.10, float(yaw_cfg.get('path_yaw_lookahead_m', 1.0)))
        self.final_yaw_blend_distance_m = max(
            self.arrival_tolerance,
            float(yaw_cfg.get('final_yaw_blend_distance_m', 2.0)))
        self.last_commanded_yaw: Optional[float] = None
        self.last_yaw_stamp = rospy.Time(0)
        self.final_hold_start = rospy.Time(0)
        self.yaw_alignment_active = False

        # Execution-layer LiDAR safety gate.  It is independent of the global
        # map/A* planner and freezes the PX4 position setpoint when an obstacle
        # appears in the current horizontal flight direction.
        self.safety_cfg = self.cfg.get('safety', {})
        self.safety_enabled = bool(self.safety_cfg.get('enabled', True))
        self.safety_stop_distance = float(self.safety_cfg.get('stop_distance_m', 0.90))
        self.safety_half_width = float(self.safety_cfg.get('half_width_m', 0.45))
        self.safety_vertical_min = float(self.safety_cfg.get('vertical_min_m', -0.45))
        self.safety_vertical_max = float(self.safety_cfg.get('vertical_max_m', 0.45))
        self.safety_min_forward = float(self.safety_cfg.get('min_forward_m', 0.15))
        self.safety_min_goal_distance = float(self.safety_cfg.get('min_goal_distance_m', 0.20))
        self.safety_point_stride = max(1, int(self.safety_cfg.get('point_stride', 1)))
        self.safety_block_confirm_frames = max(1, int(self.safety_cfg.get('block_confirm_frames', 2)))
        self.safety_clear_confirm_frames = max(1, int(self.safety_cfg.get('clear_confirm_frames', 3)))
        root = rospy.get_param(
            '/vlm_semantic_search',
            {},
        )

        color_cfg = root.get(
            'scheduler',
            {},
        ).get(
            'color_candidate_trigger',
            {},
        )

        self.color_hold_max_sec = float(
            color_cfg.get(
                'hold_max_sec',
                300.0,
            )
        )
        self.safety_min_block_points = max(1, int(self.safety_cfg.get('min_block_points', 12)))
        self.safety_self_filter_radius = float(self.safety_cfg.get('self_filter_radius_m', 0.45))
        lidar_xyz = self.cfg.get('lidar_xyz', [0.0, 0.0, 0.0])
        self.lidar_x = float(lidar_xyz[0])
        self.lidar_y = float(lidar_xyz[1])
        self.lidar_z = float(lidar_xyz[2])
        lidar_rpy = self.cfg.get('lidar_rpy', [0.0, 0.0, 0.0])
        self.lidar_roll = float(lidar_rpy[0])
        self.lidar_pitch = float(lidar_rpy[1])
        self.lidar_yaw = float(lidar_rpy[2])

        self.lock = threading.RLock()
        self.state = State()
        self.local_pose: Optional[PoseStamped] = None
        self.map_pose: Optional[PoseStamped] = None
        self.goal: Optional[PoseStamped] = None
        self.pending_goal: Optional[PoseStamped] = None
        self.route: List[PoseStamped] = []
        self.route_index = 0
        self.route_active = False
        self.pending_route: Optional[List[PoseStamped]] = None

        self.setpoint: Optional[PoseStamped] = None
        self.takeoff_setpoint: Optional[PoseStamped] = None
        self.takeoff_ready = False
        self.takeoff_hold_start = rospy.Time(0)
        self.setpoint_count = 0
        self.last_mode_request = rospy.Time(0)
        self.last_arm_request = rospy.Time(0)
        self.goal_reported = False

        self.safety_blocked = False
        self.safety_block_hits = 0
        self.safety_clear_hits = 0

        self.color_hold_active = False
        self.color_hold_id = ''
        self.color_hold_deadline_wall = 0.0
        self.task_finished = False

        ns = '/' + self.vehicle
        self.setpoint_pub = rospy.Publisher(self.cfg['setpoint_topic'], PoseStamped, queue_size=30)
        self.reached_pub = rospy.Publisher(ns + '/mission/reached', Bool, queue_size=5, latch=True)
        self.takeoff_ready_pub = rospy.Publisher(ns + '/mission/takeoff_ready', Bool, queue_size=1, latch=True)
        self.status_pub = rospy.Publisher(ns + '/mission/status', String, queue_size=10, latch=True)
        self.safety_block_pub = rospy.Publisher(
            self.cfg.get('blocked_topic', ns + '/safety/blocked'), Bool, queue_size=5, latch=True)
        self.takeoff_ready_pub.publish(False)
        self.reached_pub.publish(False)
        self.safety_block_pub.publish(False)

        rospy.Subscriber(self.cfg['mavros_state_topic'], State, self._state_cb, queue_size=20)
        rospy.Subscriber(self.cfg['local_pose_topic'], PoseStamped, self._local_pose_cb, queue_size=20)
        rospy.Subscriber(self.cfg['global_pose_topic'], PoseStamped, self._map_pose_cb, queue_size=20)
        rospy.Subscriber(self.cfg['mission_goal_topic'], PoseStamped, self._goal_cb, queue_size=5)
        rospy.Subscriber(self.cfg.get('planned_path_topic', '/%s/search/planned_path' % self.vehicle),
                         Path, self._path_cb, queue_size=5)
        rospy.Subscriber(
            '/%s/vlm/observation_hold_command' % self.vehicle,
            String,
            self._color_hold_cb,
            queue_size=10,
        )
        rospy.Subscriber('/vlm/query_switch_cancel', String, self._query_switch_cancel_cb, queue_size=10)
        rospy.Subscriber('/experiment/task_finished', String, self._task_finished_cb, queue_size=1)
        rospy.Subscriber(self.cfg['lidar_topic'], PointCloud2, self._cloud_cb,
                         queue_size=1, buff_size=2 ** 24)

        self.set_mode = rospy.ServiceProxy(ns + '/mavros/set_mode', SetMode)
        self.arm = rospy.ServiceProxy(ns + '/mavros/cmd/arming', CommandBool)
        rospy.Timer(rospy.Duration(1.0 / 30.0), self._tick)
        rospy.loginfo('%s A* route executor ready (autostart=%s, takeoff=%.2fm).',
                      self.vehicle, self.autostart, self.takeoff_height)

    def _state_cb(self, msg):
        with self.lock:
            self.state = msg

    def _local_pose_cb(self, msg):
        with self.lock:
            self.local_pose = msg
            if self.takeoff_setpoint is None:
                self.takeoff_setpoint = PoseStamped()
                self.takeoff_setpoint.header.frame_id = msg.header.frame_id or 'local_origin'
                self.takeoff_setpoint.pose = msg.pose
                self.takeoff_setpoint.pose.position.z = msg.pose.position.z + self.takeoff_height
                self.setpoint = self.takeoff_setpoint
                self.status_pub.publish('takeoff_setpoint_ready')
                rospy.loginfo('%s takeoff gate setpoint local [%.2f, %.2f, %.2f].', self.vehicle,
                              self.takeoff_setpoint.pose.position.x,
                              self.takeoff_setpoint.pose.position.y,
                              self.takeoff_setpoint.pose.position.z)

    def _map_pose_cb(self, msg):
        with self.lock:
            self.map_pose = msg

    @staticmethod
    def _distance(a: PoseStamped, b: PoseStamped) -> float:
        pa, pb = a.pose.position, b.pose.position
        return math.sqrt((pa.x - pb.x) ** 2 + (pa.y - pb.y) ** 2 + (pa.z - pb.z) ** 2)

    def _activate_goal_locked(self, msg: PoseStamped) -> None:
        self.route = []
        self.route_index = 0
        self.route_active = False
        self.goal = msg
        self.pending_goal = None
        self.goal_reported = False
        self.final_hold_start = rospy.Time(0)
        self.yaw_alignment_active = False
        self.reached_pub.publish(False)
        self.status_pub.publish('tracking_direct_goal')
        rospy.loginfo('%s accepted direct map goal [%.2f, %.2f, %.2f].', self.vehicle,
                      msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)

    def _activate_route_locked(self, poses: List[PoseStamped]) -> None:
        if not poses:
            return
        self.route = list(poses)
        self.route_index = 0
        self.route_active = True
        self.goal = self.route[0]
        self.pending_route = None
        self.goal_reported = False
        self.final_hold_start = rospy.Time(0)
        self.yaw_alignment_active = False
        self.reached_pub.publish(False)
        self.status_pub.publish('tracking_astar_route')
        rospy.loginfo('%s accepted A* route with %d waypoint(s).', self.vehicle, len(self.route))

    def _goal_cb(self, msg):
        if (msg.header.frame_id or 'map') != 'map':
            rospy.logwarn('%s ignores goal in frame %s; expected map.', self.vehicle, msg.header.frame_id)
            return
        with self.lock:
            if self.task_finished:
                rospy.logwarn_throttle(5.0, '%s ignores goal after task completion.', self.vehicle)
                return
            if not self.takeoff_ready:
                self.pending_goal = msg
                self.pending_route = None
                self.status_pub.publish('goal_queued_until_takeoff_ready')
                return
            self._activate_goal_locked(msg)

    def _path_cb(self, msg: Path):
        if (msg.header.frame_id or 'map') != 'map':
            rospy.logwarn('%s ignores path in frame %s; expected map.', self.vehicle, msg.header.frame_id)
            return
        if not msg.poses:
            rospy.logwarn('%s received empty A* route; retaining current command.', self.vehicle)
            return
        with self.lock:
            if self.task_finished:
                rospy.logwarn_throttle(5.0, '%s ignores route after task completion.', self.vehicle)
                return
            if not self.takeoff_ready:
                self.pending_route = list(msg.poses)
                self.pending_goal = None
                self.status_pub.publish('route_queued_until_takeoff_ready')
                return
            self._activate_route_locked(list(msg.poses))
        
    def _color_hold_cb(self, msg: String) -> None:
        try:
            command = json.loads(msg.data)
        except Exception:
            return

        if not isinstance(command, dict):
            return

        hold_id = str(
            command.get('hold_id', '')
        )

        active = bool(
            command.get('active', False)
        )

        with self.lock:
            if active:
                self.color_hold_active = True
                self.color_hold_id = hold_id

                self.color_hold_deadline_wall = (
                    time.monotonic()
                    + self.color_hold_max_sec
                )

                rospy.logwarn(
                    '%s entered color candidate hover id=%s.',
                    self.vehicle,
                    hold_id,
                )
                return

            # An old color event cannot release a newer color hold.
            if (
                self.color_hold_id
                and hold_id
                and self.color_hold_id != hold_id
            ):
                return

            self.color_hold_active = False
            self.color_hold_id = ''
            self.color_hold_deadline_wall = 0.0

            rospy.loginfo(
                '%s released color candidate hover id=%s.',
                self.vehicle,
                hold_id,
            )


    def _query_switch_cancel_cb(self, msg: String) -> None:
        notice = None
        try:
            notice = json.loads(msg.data)
        except Exception:
            return
        if not isinstance(notice, dict):
            return
        with self.lock:
            self.route = []
            self.route_index = 0
            self.route_active = False
            self.pending_route = None
            self.pending_goal = None
            self.goal = None
            self.goal_reported = False
            self.final_hold_start = rospy.Time(0)
            self.yaw_alignment_active = False
            self.color_hold_active = False
            self.color_hold_id = ''
            self.color_hold_deadline_wall = 0.0
            hold = self._hover_setpoint_locked(rospy.Time.now())
            if hold is not None:
                self.setpoint = hold
            self.reached_pub.publish(False)
            self.status_pub.publish('query_switch_cancel_hold')
        rospy.logwarn('%s cancelled active route due to target query switch v%s->v%s.',
                      self.vehicle, notice.get('old_query_version'), notice.get('new_query_version'))

    def _task_finished_cb(self, msg: String) -> None:
        try:
            event = json.loads(msg.data)
        except Exception:
            return
        if not isinstance(event, dict) or not bool(event.get('finished', False)):
            return
        with self.lock:
            if self.task_finished:
                return
            self.task_finished = True
            self.route = []
            self.route_index = 0
            self.route_active = False
            self.pending_route = None
            self.pending_goal = None
            self.goal = None
            self.goal_reported = True
            self.final_hold_start = rospy.Time(0)
            self.yaw_alignment_active = False
            self.color_hold_active = False
            self.color_hold_id = ''
            self.color_hold_deadline_wall = 0.0
            hold = self._hover_setpoint_locked(rospy.Time.now())
            if hold is not None:
                self.setpoint = hold
            self.status_pub.publish('task_finished_hover')
        rospy.logwarn(
            '%s entered terminal hover: success=%s reason=%s.',
            self.vehicle,
            event.get('success'),
            event.get('finish_reason'),
        )

    def _color_hold_active_locked(self) -> bool:
        if not self.color_hold_active:
            return False

        if (
            self.color_hold_deadline_wall > 0.0
            and time.monotonic()
            >= self.color_hold_deadline_wall
        ):
            rospy.logwarn(
                '%s color candidate hover expired; resuming route.',
                self.vehicle,
            )

            self.color_hold_active = False
            self.color_hold_id = ''
            self.color_hold_deadline_wall = 0.0

            return False

        return True

    def _update_safety_gate(self, raw_blocked: bool) -> None:
        """Debounce LiDAR obstacle detections and publish the safety state."""
        with self.lock:
            previous = self.safety_blocked
            if raw_blocked:
                self.safety_block_hits += 1
                self.safety_clear_hits = 0
                if self.safety_block_hits >= self.safety_block_confirm_frames:
                    self.safety_blocked = True
            else:
                self.safety_clear_hits += 1
                self.safety_block_hits = 0
                if self.safety_clear_hits >= self.safety_clear_confirm_frames:
                    self.safety_blocked = False
            changed = (previous != self.safety_blocked)
            blocked = self.safety_blocked
            hits = self.safety_block_hits
            clears = self.safety_clear_hits

        if changed:
            self.safety_block_pub.publish(blocked)
            rospy.logwarn(
                '%s LiDAR flight gate blocked=%s (hits=%d, clears=%d).',
                self.vehicle, blocked, hits, clears)

    def _cloud_cb(self, msg: PointCloud2) -> None:
        """Check the LiDAR corridor in the *current commanded flight direction*.

        The multirotor may move laterally while maintaining its current yaw, so the
        test uses the map-frame vector from the current pose to the active A* waypoint,
        expressed in the body/LiDAR frame.  This is safer than checking only body +x.
        """
        if not self.safety_enabled:
            return

        with self.lock:
            ready = self.takeoff_ready
            pose = self.map_pose
            goal = self.goal

        if not ready or pose is None or goal is None:
            self._update_safety_gate(False)
            return

        dx = goal.pose.position.x - pose.pose.position.x
        dy = goal.pose.position.y - pose.pose.position.y
        distance = math.hypot(dx, dy)
        if distance < self.safety_min_goal_distance:
            self._update_safety_gate(False)
            return

        q = pose.pose.orientation
        yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])[2]

        # Unit travel vector expressed in body FLU coordinates: +x forward, +y left.
        forward = math.cos(yaw) * dx + math.sin(yaw) * dy
        left = -math.sin(yaw) * dx + math.cos(yaw) * dy
        norm = math.hypot(forward, left)
        if norm < 1e-6:
            self._update_safety_gate(False)
            return
        ux, uy = forward / norm, left / norm

        raw_blocked = False
        block_point_count = 0
        try:
            for index, point in enumerate(pc2.read_points(
                    msg, field_names=('x', 'y', 'z'), skip_nans=True)):
                if index % self.safety_point_stride:
                    continue
                x, y, z = point
                if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
                    continue

                # LiDAR-frame point -> body-frame point.  The current UAV config
                # has zero LiDAR rotation, so only the declared translation is needed.
                # 先应用 LiDAR 相对于机体的 yaw 外参，再叠加平移外参。
                c = math.cos(self.lidar_yaw)
                s = math.sin(self.lidar_yaw)

                # LiDAR-frame point -> body-frame point.
                bx = x + self.lidar_x
                by = y + self.lidar_y
                bz = z + self.lidar_z

                # -------------------------------------------------------------
                # 新增：过滤无人机自身附近的 LiDAR 回波。
                # 例如机体、支架、旋翼附近的伪点。
                # 必须放在 bx/by/bz 计算完成后、along/lateral 判断前。
                # -------------------------------------------------------------
                range_xy = math.hypot(bx, by)

                if range_xy < self.safety_self_filter_radius:
                    continue

                # 仅保留与无人机当前飞行高度接近的点。
                if bz < self.safety_vertical_min or bz > self.safety_vertical_max:
                    continue

                # 将点投影到当前实际飞行方向上。
                along = bx * ux + by * uy
                lateral = -uy * bx + ux * by
                if (self.safety_min_forward <= along <= self.safety_stop_distance
                        and abs(lateral) <= self.safety_half_width):

                    block_point_count += 1

                    if block_point_count >= self.safety_min_block_points:
                        raw_blocked = True
                        # rospy.logwarn_throttle(
                        #     1.0,
                        #     '%s safety corridor blocked: points=%d, '
                        #     'along=%.2f, lateral=%.2f, '
                        #     'map_pose=(%.2f, %.2f), goal=(%.2f, %.2f)',
                        #     self.vehicle,
                        #     block_point_count,
                        #     along,
                        #     lateral,
                        #     pose.pose.position.x,
                        #     pose.pose.position.y,
                        #     goal.pose.position.x,
                        #     goal.pose.position.y,
                        # )
                        break
        except Exception as exc:
            rospy.logwarn_throttle(
                3.0, '%s LiDAR flight-safety parse failed: %r', self.vehicle, exc)
            return

        self._update_safety_gate(raw_blocked)

    def _hover_setpoint_locked(self, now: rospy.Time) -> Optional[PoseStamped]:
        """Freeze the local PX4 setpoint at the current pose while blocked."""
        if self.local_pose is None:
            return None
        hold = PoseStamped()
        hold.header.stamp = now
        hold.header.frame_id = self.local_pose.header.frame_id or 'local_origin'
        hold.pose = self.local_pose.pose
        return hold

    def _goal_yaw_locked(self) -> Optional[float]:
        if self.goal is None:
            return None
        q = self.goal.pose.orientation
        norm = math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)
        if norm < 1e-3:
            return None
        return euler_from_quaternion([q.x, q.y, q.z, q.w])[2]

    def _actual_yaw_locked(self) -> Optional[float]:
        pose = self.map_pose or self.local_pose
        if pose is None:
            return None
        q = pose.pose.orientation
        return euler_from_quaternion([q.x, q.y, q.z, q.w])[2]

    def _goal_yaw_error_locked(self) -> float:
        desired = self._goal_yaw_locked()
        actual = self._actual_yaw_locked()
        if desired is None or actual is None:
            return 0.0
        return self._wrap_angle(desired - actual)

    def _advance_route_locked(self, now: rospy.Time) -> bool:
        if not self.route_active or self.map_pose is None or not self.route:
            return False

        # Intermediate waypoints are consumed without a stop-and-turn phase.
        while (
            self.route_index < len(self.route) - 1
            and self._distance(
                self.map_pose,
                self.route[self.route_index],
            ) <= self.arrival_tolerance
        ):
            self.route_index += 1
            self.goal = self.route[self.route_index]
            self.final_hold_start = rospy.Time(0)
            rospy.loginfo(
                '%s advances A* route waypoint %d/%d.',
                self.vehicle,
                self.route_index + 1,
                len(self.route),
            )

        self.goal = self.route[self.route_index]

        if self.route_index != len(self.route) - 1:
            return False

        if self._distance(self.map_pose, self.goal) > self.arrival_tolerance:
            return False

        # Only the final frontier viewpoint requires the final observation yaw.
        # Position remains fixed naturally at the endpoint while the bounded yaw
        # command removes any small residual error. No fixed dwell is required.
        yaw_error = abs(self._goal_yaw_error_locked())
        if self.yaw_follow_enabled and yaw_error > self.final_yaw_tolerance:
            self.status_pub.publish('final_view_yaw_aligning')
            return False

        self.route_active = False
        self.final_hold_start = rospy.Time(0)
        return True

    @staticmethod
    def _wrap_angle(value: float) -> float:
        return math.atan2(math.sin(value), math.cos(value))

    def _smooth_yaw_locked(self, desired_yaw: float, now: rospy.Time) -> float:
        if self.local_pose is None or not self.yaw_follow_enabled:
            return desired_yaw
        current_q = self.local_pose.pose.orientation
        current_yaw = euler_from_quaternion(
            [current_q.x, current_q.y, current_q.z, current_q.w])[2]
        if self.last_commanded_yaw is None:
            self.last_commanded_yaw = current_yaw
            self.last_yaw_stamp = now
        dt = (now - self.last_yaw_stamp).to_sec() if not self.last_yaw_stamp.is_zero() else 1.0 / 30.0
        dt = min(0.2, max(1.0 / 120.0, dt))
        error = self._wrap_angle(desired_yaw - self.last_commanded_yaw)
        if abs(error) <= self.yaw_tolerance:
            command = desired_yaw
        else:
            maximum = self.max_yaw_rate * dt
            command = self.last_commanded_yaw + max(-maximum, min(maximum, error))
        self.last_commanded_yaw = self._wrap_angle(command)
        self.last_yaw_stamp = now
        return self.last_commanded_yaw

    @staticmethod
    def _blend_angle(start: float, end: float, ratio: float) -> float:
        ratio = max(0.0, min(1.0, float(ratio)))
        delta = WaypointExecutor._wrap_angle(end - start)
        return WaypointExecutor._wrap_angle(start + ratio * delta)

    def _desired_yaw_locked(self) -> float:
        actual_yaw = self._actual_yaw_locked()
        if actual_yaw is None:
            actual_yaw = 0.0

        if self.goal is None or self.map_pose is None:
            return actual_yaw

        current = self.map_pose.pose.position
        target = self.goal.pose.position
        dx = float(target.x) - float(current.x)
        dy = float(target.y) - float(current.y)
        distance = math.hypot(dx, dy)
        final_yaw = self._goal_yaw_locked()

        if distance <= 1e-6:
            return final_yaw if final_yaw is not None else actual_yaw

        travel_yaw = math.atan2(dy, dx)

        if self.route_active and self.route:
            final_index = len(self.route) - 1

            if self.route_index < final_index:
                lookahead = max(
                    self.arrival_tolerance,
                    self.path_yaw_lookahead_m,
                )

                if distance < lookahead:
                    next_pose = self.route[
                        min(self.route_index + 1, final_index)
                    ].pose.position
                    ndx = float(next_pose.x) - float(current.x)
                    ndy = float(next_pose.y) - float(current.y)

                    if math.hypot(ndx, ndy) > 1e-6:
                        next_yaw = math.atan2(ndy, ndx)
                        ratio = 1.0 - distance / lookahead
                        ratio = ratio * ratio * (3.0 - 2.0 * ratio)
                        return self._blend_angle(
                            travel_yaw,
                            next_yaw,
                            ratio,
                        )

                return travel_yaw

            if final_yaw is None:
                return travel_yaw

            blend_distance = max(
                self.arrival_tolerance,
                self.final_yaw_blend_distance_m,
            )
            ratio = 1.0 - min(1.0, distance / blend_distance)
            ratio = ratio * ratio * (3.0 - 2.0 * ratio)
            return self._blend_angle(
                travel_yaw,
                final_yaw,
                ratio,
            )

        # Legacy direct goals also translate first and finish their requested yaw
        # only near the endpoint instead of flying sideways from the start.
        if final_yaw is not None and distance <= self.arrival_tolerance:
            return final_yaw
        return travel_yaw

    def _map_goal_to_local(self):
        if self.goal is None or self.local_pose is None or self.map_pose is None:
            return None

        task = self.goal.pose.position
        map_now = self.map_pose.pose.position
        local_now = self.local_pose.pose.position

        result = PoseStamped()
        result.header.stamp = rospy.Time.now()
        result.header.frame_id = (
            self.local_pose.header.frame_id or 'local_origin'
        )

        # Translation is never frozen merely to change yaw. Safety and explicit
        # semantic observation holds remain independent and may still hover.
        result.pose.position.x = local_now.x + task.x - map_now.x
        result.pose.position.y = local_now.y + task.y - map_now.y
        result.pose.position.z = local_now.z + task.z - map_now.z

        desired_yaw = self._desired_yaw_locked()
        command_yaw = self._smooth_yaw_locked(
            desired_yaw,
            result.header.stamp,
        )

        quaternion = quaternion_from_euler(
            0.0,
            0.0,
            command_yaw,
        )
        (
            result.pose.orientation.x,
            result.pose.orientation.y,
            result.pose.orientation.z,
            result.pose.orientation.w,
        ) = quaternion

        self.yaw_alignment_active = False
        return result

    def _at_current_goal(self):
        return self.goal is not None and self.map_pose is not None and \
            self._distance(self.map_pose, self.goal) <= self.arrival_tolerance

    def _update_takeoff_gate_locked(self, now):
        if self.takeoff_ready or self.local_pose is None or self.takeoff_setpoint is None:
            return
        if not self.state.armed or self.state.mode != 'OFFBOARD':
            self.takeoff_hold_start = rospy.Time(0)
            return
        target_z = self.takeoff_setpoint.pose.position.z
        if abs(target_z - self.local_pose.pose.position.z) > self.takeoff_tolerance:
            self.takeoff_hold_start = rospy.Time(0)
            self.status_pub.publish('takeoff_climbing')
            return
        if self.takeoff_hold_start.is_zero():
            self.takeoff_hold_start = now
            self.status_pub.publish('takeoff_height_reached_settling')
            rospy.loginfo('%s reached takeoff altitude; holding for %.1fs.', self.vehicle, self.takeoff_hold_sec)
            return
        if (now - self.takeoff_hold_start).to_sec() < self.takeoff_hold_sec:
            return
        self.takeoff_ready = True
        self.takeoff_ready_pub.publish(True)
        self.status_pub.publish('takeoff_ready')
        rospy.loginfo('%s takeoff gate OPEN. High-level mission goals are now enabled.', self.vehicle)
        if self.pending_route:
            self._activate_route_locked(self.pending_route)
        elif self.pending_goal is not None:
            self._activate_goal_locked(self.pending_goal)

    def _request_offboard_or_arm(self, now):
        if not self.autostart or not self.state.connected or self.setpoint_count < 100:
            return
        try:
            if self.state.mode != 'OFFBOARD':
                if (now - self.last_mode_request).to_sec() >= self.request_period:
                    response = self.set_mode(base_mode=0, custom_mode='OFFBOARD')
                    self.last_mode_request = now
                    self.status_pub.publish('offboard_request_sent' if response.mode_sent else 'offboard_request_rejected')
            elif not self.state.armed:
                if (now - self.last_arm_request).to_sec() >= self.request_period:
                    response = self.arm(value=True)
                    self.last_arm_request = now
                    self.status_pub.publish('arm_request_sent' if response.success else 'arm_request_rejected')
                    rospy.loginfo('%s arm request success=%s.', self.vehicle, response.success)
            else:
                self.status_pub.publish('executing' if self.takeoff_ready else 'takeoff_climbing')
        except rospy.ServiceException as exc:
            rospy.logwarn_throttle(3.0, '%s MAVROS service failure: %s', self.vehicle, exc)

    def _tick(self, _event):
        with self.lock:
            if self.setpoint is None:
                return
            now = rospy.Time.now()
            route_done = False

            color_hold = self._color_hold_active_locked()

            if self.task_finished:
                hold = self._hover_setpoint_locked(now)
                if hold is not None:
                    self.setpoint = hold
                self.status_pub.publish('task_finished_hover')

            elif not self.takeoff_ready:
                self.setpoint = self.takeoff_setpoint

            elif color_hold:
                # 不推进 A* route_index；将 PX4 local setpoint 固定为当前局部位姿。
                hold = self._hover_setpoint_locked(now)

                if hold is not None:
                    self.setpoint = hold

                self.status_pub.publish(
                    'color_candidate_hover'
                )

            elif self.safety_enabled and self.safety_blocked:
                # LiDAR safety gate 仍然独立生效。
                hold = self._hover_setpoint_locked(now)

                if hold is not None:
                    self.setpoint = hold

                self.status_pub.publish(
                    'safety_hover_blocked'
                )

            else:
                route_done = self._advance_route_locked(now)

                converted = self._map_goal_to_local()

                if converted is not None:
                    self.setpoint = converted
                    if self.yaw_alignment_active:
                        self.status_pub.publish('route_yaw_aligning_before_translation')

            self.setpoint.header.stamp = now
            self.setpoint_pub.publish(self.setpoint)
            self.setpoint_count += 1
            self._request_offboard_or_arm(now)
            self._update_takeoff_gate_locked(now)
            if (
                self.takeoff_ready
                and not self.task_finished
                and not color_hold
                and (
                    route_done
                    or (
                        not self.route_active
                        and self._at_current_goal()
                    )
                )
                and not self.goal_reported
            ):
                self.goal_reported = True
                self.reached_pub.publish(True)
                self.status_pub.publish('route_reached' if route_done else 'goal_reached')
                rospy.loginfo('%s reached %s.', self.vehicle, 'final A* route goal' if route_done else 'current map goal')


def main():
    rospy.init_node('waypoint_executor')
    WaypointExecutor()
    rospy.spin()


if __name__ == '__main__':
    main()
