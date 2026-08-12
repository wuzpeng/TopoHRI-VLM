Stage-5 v0.5.6: startup-query barrier + serial local cloud-VLM dispatch

Base version: Stage-5 v0.5.5 cloud-safe epoch recovery.

Install the delta package by copying its uav_semantic_search/ contents over:
  ~/harp_sar_ws/src/uav_semantic_search/

Then run:
  cd ~/harp_sar_ws/src/uav_semantic_search
  python3 scripts/upgrade_stage5_v056_serial_local_config.py

  cd ~/harp_sar_ws
  catkin_make --force-cmake
  source devel/setup.bash

The upgrade script preserves backend.api_key, base_url, model and target-query
fields. It changes local multi-robot VLM requests to sequential dispatch, and
makes the per-robot coordinator wait at least 5 seconds longer than the current
HTTP timeout.
