#!/usr/bin/env python3
"""Stage-2 target-search validation route; uav1 enters the target room."""
import rospy
from mission_sequencer_common import MissionSequencer


def main():
    rospy.init_node('semantic_search_demo_mission')
    MissionSequencer('target_search_mission.yaml')
    rospy.spin()


if __name__ == '__main__':
    main()
