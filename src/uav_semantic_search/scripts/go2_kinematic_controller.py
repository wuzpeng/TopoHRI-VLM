#!/usr/bin/env python3
"""Navigation-level planar controller for the Go2-proportioned Gazebo model.

It converts /ugv0/cmd_vel into bounded SE(2) updates via /gazebo/set_model_state.
This deliberately abstracts low-level quadruped gait dynamics. A LiDAR front-sector
safety gate inhibits positive linear motion before the model can enter a detected
near obstacle. It does not replace a full contact-aware legged controller.
"""
from __future__ import annotations

import math
import threading

import rospy
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool, String
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import sensor_msgs.point_cloud2 as pc2


class Go2KinematicController:
    def __init__(self):
        robots = rospy.get_param('/ground_robots', [])
        if not robots:
            raise RuntimeError('Missing /ground_robots configuration.')
        self.robot = robots[0]
        self.name = self.robot['name']
        self.model_name = self.robot['gazebo_model']
        self.nav = self.robot.get('navigation', {})
        self.rate_hz = float(self.nav.get('controller_rate_hz', 20.0))
        self.max_v = float(self.nav.get('max_linear_speed_mps', 0.38))
        self.max_w = float(self.nav.get('max_angular_speed_rps', 0.70))
        self.timeout = float(self.nav.get('command_timeout_sec', 0.60))
        self.fixed_z = float(self.robot.get('spawn', {}).get('z', self.robot.get('base_height_m', 0.34)))
        self.front_stop = float(self.nav.get('front_stop_distance_m', 0.52))
        self.front_half_width = float(self.nav.get('front_half_width_m', 0.27))
        self.ground_reject_z = float(self.nav.get('lidar_ground_reject_z_m', -0.62))
        self.floor_margin = float(self.nav.get('lidar_floor_margin_m', 0.08))
        self.block_confirm_frames = max(1, int(self.nav.get('front_block_confirm_frames', 3)))
        self.clear_confirm_frames = max(1, int(self.nav.get('front_clear_confirm_frames', 3)))

        self.lock = threading.RLock()
        self.pose = None
        self.cmd = Twist()
        self.last_cmd = rospy.Time(0)
        self.front_blocked = False
        self.block_hits = 0
        self.clear_hits = 0
        self.set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        self.block_pub = rospy.Publisher(self.robot['blocked_topic'], Bool, queue_size=5, latch=True)
        self.status_pub = rospy.Publisher('/%s/kinematic/status' % self.name, String, queue_size=10, latch=True)
        self.block_pub.publish(False)

        rospy.Subscriber(self.robot['global_pose_topic'], PoseStamped, self._pose_cb, queue_size=20)
        rospy.Subscriber(self.robot['cmd_vel_topic'], Twist, self._cmd_cb, queue_size=20)
        rospy.Subscriber(self.robot['lidar_topic'], PointCloud2, self._cloud_cb, queue_size=1, buff_size=2 ** 24)
        self.control_timer = rospy.Timer(
            rospy.Duration(1.0 / max(1.0, self.rate_hz)), self._tick)
        rospy.loginfo('%s Go2 navigation-level controller ready for model %s.', self.name, self.model_name)

    def _pose_cb(self, msg):
        with self.lock:
            self.pose = msg

    def _cmd_cb(self, msg):
        with self.lock:
            self.cmd = msg
            self.last_cmd = rospy.Time.now()

    def _cloud_cb(self, msg):
        """Update a debounced *forward-translation* safety gate.

        The gate is deliberately independent of the global planner. It only says
        that advancing along the current robot x-axis is unsafe. It must not be
        interpreted as a reason to prohibit an in-place turn away from a wall.
        """
        raw_blocked = False
        try:
            for x, y, z in pc2.read_points(
                    msg, field_names=('x', 'y', 'z'), skip_nans=True):
                if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
                    continue

                # Downward beams ending on the floor are normally around -0.62 m
                # in the LiDAR frame. Reject a small band above that value as well
                # so floor noise cannot create a false front blockage.
                if z <= self.ground_reject_z + self.floor_margin:
                    continue

                # Keep low/body-height returns only; ceiling/high returns do not
                # determine whether the quadruped can translate on the floor.
                if z > 0.35:
                    continue

                if (0.10 <= x <= self.front_stop and
                        abs(y) <= self.front_half_width):
                    raw_blocked = True
                    break
        except Exception as exc:
            rospy.logwarn_throttle(
                3.0, '%s LiDAR safety parse failed: %r', self.name, exc)
            return

        with self.lock:
            previous = self.front_blocked
            if raw_blocked:
                self.block_hits += 1
                self.clear_hits = 0
                if self.block_hits >= self.block_confirm_frames:
                    self.front_blocked = True
            else:
                self.clear_hits += 1
                self.block_hits = 0
                if self.clear_hits >= self.clear_confirm_frames:
                    self.front_blocked = False
            changed = (previous != self.front_blocked)

        if changed:
            self.block_pub.publish(self.front_blocked)
            rospy.logwarn(
                '%s front translation gate blocked=%s (hits=%d, clears=%d).',
                self.name, self.front_blocked, self.block_hits, self.clear_hits)

    def _tick(self, event):
        now = rospy.Time.now()
        dt = (event.current_real - event.last_real).to_sec() if event.last_real else 1.0 / self.rate_hz
        dt = min(max(dt, 0.001), 0.15)
        with self.lock:
            pose_msg = self.pose
            cmd = self.cmd
            expired = self.last_cmd.is_zero() or (now - self.last_cmd).to_sec() > self.timeout
            blocked = self.front_blocked
        if pose_msg is None:
            self.status_pub.publish('WAIT_POSE')
            return
        v = max(-self.max_v, min(self.max_v, float(cmd.linear.x)))
        w = max(-self.max_w, min(self.max_w, float(cmd.angular.z)))
        if expired:
            v = 0.0
            w = 0.0
        if blocked and v > 0.0:
            # The front gate protects translation only. Preserve a commanded
            # in-place rotation so the robot can turn away from a nearby wall.
            # The waypoint executor never sends positive v while blocked.
            v = 0.0
            self.status_pub.publish(
                'FRONT_BLOCKED_ROTATE' if abs(w) > 1e-4 else 'FRONT_BLOCKED_HOLD')
        else:
            self.status_pub.publish('TRACKING' if abs(v) + abs(w) > 1e-4 else 'HOLD')

        p = pose_msg.pose.position
        q = pose_msg.pose.orientation
        yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
        yaw_next = yaw + w * dt
        state = ModelState()
        state.model_name = self.model_name
        state.reference_frame = 'world'
        state.pose.position.x = p.x + v * math.cos(yaw_next) * dt
        state.pose.position.y = p.y + v * math.sin(yaw_next) * dt
        state.pose.position.z = self.fixed_z
        qn = quaternion_from_euler(0.0, 0.0, yaw_next)
        state.pose.orientation.x, state.pose.orientation.y = qn[0], qn[1]
        state.pose.orientation.z, state.pose.orientation.w = qn[2], qn[3]
        state.twist.linear.x = state.twist.linear.y = state.twist.linear.z = 0.0
        state.twist.angular.x = state.twist.angular.y = state.twist.angular.z = 0.0
        try:
            response = self.set_state(state)
            if not response.success:
                rospy.logwarn_throttle(3.0, '%s Gazebo kinematic update rejected: %s', self.name, response.status_message)
        except rospy.ServiceException as exc:
            rospy.logwarn_throttle(3.0, '%s cannot update Go2 model pose: %s', self.name, exc)


def main():
    rospy.init_node('go2_kinematic_controller')
    Go2KinematicController()
    rospy.spin()


if __name__ == '__main__':
    main()