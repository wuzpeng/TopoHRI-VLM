#!/usr/bin/env python3
"""Baseline corridor route; publishes fixed map-frame waypoints."""
import rospy
from mission_sequencer_common import MissionSequencer


def main():
    rospy.init_node('corridor_demo_mission')
    MissionSequencer('demo_mission.yaml')
    rospy.spin()


if __name__ == '__main__':
    main()
