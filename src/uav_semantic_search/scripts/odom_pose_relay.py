#!/usr/bin/env python3
"""Optional adapter: publish SLAM Odometry as /uavX/global_pose."""
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
class Relay:
    def __init__(self):
        self.v=rospy.get_param('~vehicle'); topic=rospy.get_param('~odom_topic')
        self.pub=rospy.Publisher('/%s/global_pose'%self.v, PoseStamped, queue_size=20)
        rospy.Subscriber(topic, Odometry, self.cb, queue_size=20)
    def cb(self,m):
        p=PoseStamped(); p.header=m.header; p.header.frame_id='map'; p.pose=m.pose.pose; self.pub.publish(p)
def main(): rospy.init_node('odom_pose_relay'); Relay(); rospy.spin()
if __name__=='__main__': main()
