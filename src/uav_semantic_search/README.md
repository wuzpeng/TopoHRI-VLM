# UAV Semantic Search Workspace

Current consolidated version: **0.6.1**.

This build combines the four-map TopoHRI/VLM project with the validated UAV
path-heading yaw controller and timestamp-aligned tf2 LiDAR mapping chain.

The package provides a Gazebo Classic / ROS Noetic research prototype for incremental indoor semantic search:

- Stage-2: dual PX4 UAV flight, LiDAR/RGB-D sensing, shared 2.5D map, red target localization and target fusion;
- Stage-3: centralized RACER/FUEL-inspired frontier exploration for two fixed-height UAVs;
- Stage-4: a navigation-level Go2-proportioned ground robot with a separate near-ground map and a centralized heterogeneous FUEL-style exploration manager.

Read the implementation and startup guide in:

```text
docs/README_STAGE4_GO2_HETEROGENEOUS_CN.md
```

## Stage-5 VLM Semantic Search

The workspace preserves the Stage-4 FUEL baseline launch and adds a separate
synchronous hierarchical VLM mode:

```bash
roslaunch uav_semantic_search heterogeneous_vlm_stage5.launch autostart:=true
```

See `docs/README_STAGE5_VLM_SEMANTIC_SEARCH_CN.md` for installation, target-query
switching, VLM endpoint configuration, architecture and ROS topics.

## Human--AI Experiment Metrics

`human_ai_vlm_stage5.launch` now records success rate, motion-only task time
excluding VLM epochs, team route length, and map coverage at target
confirmation. The original three-terminal startup order is unchanged. See:

```text
docs/README_EXPERIMENT_METRICS_CN.md
```
