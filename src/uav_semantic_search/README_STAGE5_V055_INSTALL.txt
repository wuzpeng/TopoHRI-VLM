Stage-5 v0.5.5 cloud-safe epoch recovery delta.

Copy this folder over an existing Stage-5 v0.5.4 package, then run:
  cd ~/harp_sar_ws/src/uav_semantic_search
  python3 scripts/upgrade_stage5_v055_cloud_safe_config.py
  cd ~/harp_sar_ws
  catkin_make --force-cmake
  source devel/setup.bash

The config upgrade script preserves backend.api_key, backend.base_url and backend.model.
