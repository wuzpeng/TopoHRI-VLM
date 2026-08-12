#!/usr/bin/env python3
"""Publish Go2 ground-truth pose and TF from Gazebo model states.

The Stage-3/4 baseline intentionally uses Gazebo pose ground truth for the
navigation-level Go2 model, matching the UAV Stage-2 simulation assumption.
"""
from __future__ import annotations

import rospy
import tf2_ros
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf.transformations import quaternion_from_euler


class Go2PoseBridge:
    def __init__(self):
        self.frame = rospy.get_param('/map/frame_id', 'map')
        robots = rospy.get_param('/ground_robots', [])
        if not robots:
            raise RuntimeError('Missing /ground_robots configuration.')
        self.robots = {robot['gazebo_model']: robot for robot in robots}
        self.pubs = {
            model: rospy.Publisher(robot['global_pose_topic'], PoseStamped, queue_size=10)
            for model, robot in self.robots.items()
        }
        self.dynamic_tf = tf2_ros.TransformBroadcaster()
        self.static_tf = tf2_ros.StaticTransformBroadcaster()
        self.last_stamp_ns = {robot['name']: -1 for robot in self.robots.values()}
        self.static_sent = set()
        rospy.Subscriber('/gazebo/model_states', ModelStates, self._cb, queue_size=2)
        rospy.loginfo('Go2 pose bridge waits for %s.', list(self.robots))

    def _publish_static_sensor_tf(self, robot):
        name = robot['name']
        if name in self.static_sent:
            return
        xyz = robot.get('lidar_xyz', [0.18, 0.0, 0.28])
        rpy = robot.get('lidar_rpy', [0.0, 0.0, 0.0])
        tfm = TransformStamped()
        tfm.header.stamp = rospy.Time.now()
        tfm.header.frame_id = '%s/base_link' % name
        tfm.child_frame_id = '%s/lidar_link' % name
        tfm.transform.translation.x = float(xyz[0])
        tfm.transform.translation.y = float(xyz[1])
        tfm.transform.translation.z = float(xyz[2])
        q = quaternion_from_euler(float(rpy[0]), float(rpy[1]), float(rpy[2]))
        tfm.transform.rotation.x, tfm.transform.rotation.y = q[0], q[1]
        tfm.transform.rotation.z, tfm.transform.rotation.w = q[2], q[3]
        self.static_tf.sendTransform(tfm)
        self.static_sent.add(name)

    def _cb(self, msg):
        stamp = rospy.Time.now()
        if stamp.is_zero():
            return
        index = {model: i for i, model in enumerate(msg.name)}
        for model, robot in self.robots.items():
            if model not in index:
                continue
            pose = msg.pose[index[model]]
            pose_msg = PoseStamped()
            pose_msg.header.stamp = stamp
            pose_msg.header.frame_id = self.frame
            pose_msg.pose = pose
            self.pubs[model].publish(pose_msg)
            self._publish_static_sensor_tf(robot)

            stamp_ns = stamp.to_nsec()
            name = robot['name']
            if stamp_ns <= self.last_stamp_ns[name]:
                continue
            tfm = TransformStamped()
            tfm.header = pose_msg.header
            tfm.child_frame_id = '%s/base_link' % name
            tfm.transform.translation.x = pose.position.x
            tfm.transform.translation.y = pose.position.y
            tfm.transform.translation.z = pose.position.z
            tfm.transform.rotation = pose.orientation
            self.dynamic_tf.sendTransform(tfm)
            self.last_stamp_ns[name] = stamp_ns


def main():
    rospy.init_node('go2_pose_bridge')
    Go2PoseBridge()
    rospy.spin()


if __name__ == '__main__':
    main()
