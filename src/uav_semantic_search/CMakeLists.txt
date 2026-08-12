cmake_minimum_required(VERSION 3.0.2)
project(uav_semantic_search)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  std_srvs
  geometry_msgs
  nav_msgs
  sensor_msgs
  visualization_msgs
  gazebo_msgs
  mavros_msgs
  tf
  tf2_ros
  tf2_sensor_msgs
  cv_bridge
  image_transport
  message_filters
  message_generation
)

add_message_files(
  FILES
  TargetObservation.msg
  TargetHypothesis.msg
  TargetHypothesisArray.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS
    rospy std_msgs std_srvs geometry_msgs nav_msgs sensor_msgs visualization_msgs
    gazebo_msgs mavros_msgs tf tf2_ros tf2_sensor_msgs cv_bridge image_transport message_filters message_runtime
)

# Only actual Python ROS executables belong here.  In particular, do NOT add
# shell files or import-only helper modules; catkin_install_python creates a
# Python wrapper that is unsuitable for both of those cases.
catkin_install_python(PROGRAMS
  scripts/gazebo_pose_bridge.py
  scripts/odom_pose_relay.py
  scripts/central_map_fuser.py
  scripts/waypoint_executor.py
  scripts/mission_sequencer_common.py
  scripts/corridor_demo_mission.py
  scripts/semantic_search_demo_mission.py
  scripts/semantic_detector.py
  scripts/target_fusion_node.py
  scripts/publish_synthetic_target.py
  scripts/patch_px4_gazebo.py
  scripts/restore_px4_gazebo.py
  scripts/run_px4_two_uav.py
  scripts/configure_px4_sitl_offboard.py
  scripts/frontier_extractor.py
  scripts/autonomous_search_manager.py
  scripts/search_metrics_logger.py
  scripts/analyze_search_metrics.py
  scripts/go2_nav_spawner.py
  scripts/go2_pose_bridge.py
  scripts/go2_kinematic_controller.py
  scripts/ground_map_fuser.py
  scripts/ugv_waypoint_executor.py
  scripts/heterogeneous_fuel_manager.py
  scripts/target_query_manager.py
  scripts/vlm_trigger_scheduler.py
  scripts/local_vlm_perception.py
  scripts/semantic_overlay_node.py
  scripts/central_vlm_planner.py
  scripts/vlm_plan_validator.py
  scripts/astar_route_planner.py
  scripts/vlm_map_fallback_planner.py
  scripts/vlm_sync_coordinator.py
  scripts/upgrade_stage5_v057_local_retry_config.py
  scripts/upgrade_stage5_v058_endpoint_selection_post_astar.py
  scripts/human_interaction_manager.py
  scripts/human_ai_dashboard.py
  scripts/robot_trajectory_visualizer.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Imported by the frontier nodes; it must remain a normal Python module.
install(FILES
  scripts/frontier_core.py
  scripts/frontier_topology.py
  scripts/vlm_common.py
  scripts/vlm_candidate_builder.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Shell scripts are installed as shell scripts, not through catkin_install_python.
install(PROGRAMS
  scripts/run_px4_two_uav.sh
  scripts/install_dependencies.sh
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts
)

install(DIRECTORY launch config worlds docs rviz models
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
