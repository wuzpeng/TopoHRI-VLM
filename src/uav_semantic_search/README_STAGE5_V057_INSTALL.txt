Stage-5 v0.5.7 — bounded identical-snapshot local VLM retry

Install delta over v0.5.6:
  1. Stop all Stage-5 terminals.
  2. Copy the delta package into ~/harp_sar_ws/src/uav_semantic_search.
  3. Run:
       python3 scripts/upgrade_stage5_v057_local_retry_config.py
  4. Rebuild:
       cd ~/harp_sar_ws
       catkin_make --force-cmake
       source devel/setup.bash
  5. Restart heterogeneous_vlm_stage5.launch and run_px4_two_uav.py.

Default behavior:
  - local VLM calls remain serial: uav0 -> uav1 -> ugv0;
  - retry uses exactly the same image/prompt snapshot;
  - normal request + at most one retry after 1.5 seconds;
  - no Gazebo physics pause;
  - central VLM is skipped if a local VLM still fails after retry.
