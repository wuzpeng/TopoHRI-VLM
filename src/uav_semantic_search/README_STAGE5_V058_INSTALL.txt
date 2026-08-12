Stage-5 v0.5.8 corrected installation
=======================================

This package is a delta from Stage-5 v0.5.7. It does NOT use the prior
"candidate stores execution_path" design.

1. Stop ROS/Gazebo/PX4 terminals.
2. Copy the delta package contents into ~/harp_sar_ws/src/uav_semantic_search.
3. Run:

   cd ~/harp_sar_ws/src/uav_semantic_search
   python3 scripts/upgrade_stage5_v058_endpoint_selection_post_astar.py

4. Rebuild:

   cd ~/harp_sar_ws
   source /opt/ros/noetic/setup.bash
   catkin_make --force-cmake
   source ~/harp_sar_ws/devel/setup.bash

5. Fully restart heterogeneous_vlm_stage5.launch and run_px4_two_uav.py.
