#!/usr/bin/env python3
"""Shared implementation for fixed map-frame waypoint missions."""
import math
import os

import rospy
import rospkg
import yaml
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from tf.transformations import quaternion_from_euler


class MissionSequencer:
    def __init__(self, default_file):
        mission_file = rospy.get_param('~mission_file', default_file)
        if not os.path.isabs(mission_file):
            mission_file = os.path.join(rospkg.RosPack().get_path('uav_semantic_search'), 'config', mission_file)
        if not os.path.isfile(mission_file):
            raise RuntimeError('Mission YAML does not exist: %s' % mission_file)
        with open(mission_file, 'r') as handle:
            data = yaml.safe_load(handle)
        self.routes = data['mission']
        self.hold_seconds = float(data.get('hold_seconds', 1.0))
        self.start_delay = float(data.get('start_delay_seconds', 5.0))
        self.indices = {name: 0 for name in self.routes}
        self.done = {name: False for name in self.routes}
        self.sent_time = {name: rospy.Time(0) for name in self.routes}
        self.publishers = {
            name: rospy.Publisher('/%s/mission/goal' % name, PoseStamped, queue_size=1, latch=True)
            for name in self.routes
        }
        for name in self.routes:
            rospy.Subscriber('/%s/mission/reached' % name, Bool,
                             lambda msg, n=name: self._reached_cb(n, msg), queue_size=5)
        self.start_time = rospy.Time.now()
        rospy.Timer(rospy.Duration(0.5), self._tick)
        rospy.loginfo('Loaded fixed mission: %s', mission_file)

    @staticmethod
    def _decode_waypoint(item):
        if isinstance(item, (list, tuple)):
            if len(item) < 3:
                raise ValueError('Waypoint list needs at least [x, y, z].')
            return float(item[0]), float(item[1]), float(item[2]), None
        if isinstance(item, dict):
            yaw = item.get('yaw_rad', None)
            if yaw is None and 'yaw_deg' in item:
                yaw = math.radians(float(item['yaw_deg']))
            return float(item['x']), float(item['y']), float(item['z']), yaw
        raise ValueError('Unsupported waypoint: %r' % (item,))

    def _reached_cb(self, vehicle, msg):
        self.done[vehicle] = bool(msg.data)

    def _send(self, vehicle, item):
        x, y, z, yaw = self._decode_waypoint(item)
        msg = PoseStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = 'map'
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        if yaw is None:
            msg.pose.orientation.w = 1.0
        else:
            msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w = \
                quaternion_from_euler(0.0, 0.0, yaw)
        self.publishers[vehicle].publish(msg)
        self.done[vehicle] = False
        self.sent_time[vehicle] = rospy.Time.now()
        rospy.loginfo('mission %s -> [%.2f, %.2f, %.2f], yaw=%s', vehicle, x, y, z,
                      'hold' if yaw is None else '%.2f rad' % yaw)

    def _tick(self, _event):
        if (rospy.Time.now() - self.start_time).to_sec() < self.start_delay:
            return
        for vehicle, route in self.routes.items():
            index = self.indices[vehicle]
            if index >= len(route):
                continue
            if self.sent_time[vehicle].is_zero():
                self._send(vehicle, route[index])
            elif self.done[vehicle] and (rospy.Time.now() - self.sent_time[vehicle]).to_sec() > self.hold_seconds:
                self.indices[vehicle] += 1
                index = self.indices[vehicle]
                if index < len(route):
                    self._send(vehicle, route[index])
                else:
                    rospy.loginfo('%s completed all mission waypoints.', vehicle)
