#!/usr/bin/env python3
"""Publish a synthetic TargetObservation to test target_fusion_node without the RGB-D path."""
import rospy
from uav_semantic_search.msg import TargetObservation


def main():
    rospy.init_node('publish_synthetic_target')
    robot = rospy.get_param('~robot', 'uav0')
    x = float(rospy.get_param('~x', 26.0))
    y = float(rospy.get_param('~y', 5.7))
    z = float(rospy.get_param('~z', 0.35))
    rate = rospy.Rate(float(rospy.get_param('~rate_hz', 2.0)))
    pub = rospy.Publisher('/%s/semantic/target_observation' % robot, TargetObservation, queue_size=10)
    while not rospy.is_shutdown():
        msg = TargetObservation()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = 'map'
        msg.robot_id = robot
        msg.class_name = 'victim_surrogate'
        msg.confidence = 0.9
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = z
        msg.pose.pose.orientation.w = 1.0
        for i in (0, 7, 14):
            msg.pose.covariance[i] = 0.15 ** 2
        msg.depth_m = 3.0
        pub.publish(msg)
        rate.sleep()


if __name__ == '__main__':
    main()
