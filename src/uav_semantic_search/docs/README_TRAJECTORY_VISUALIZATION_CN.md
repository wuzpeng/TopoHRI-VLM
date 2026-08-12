# 三机器人历史轨迹可视化

`robot_trajectory_visualizer.py` 订阅三台机器人的全局位姿，在同一 `map` 坐标系中持续记录并发布历史轨迹：

- `uav0`：蓝色，话题 `/trajectory/uav0/path`
- `uav1`：红色，话题 `/trajectory/uav1/path`
- `ugv0`：绿色，话题 `/trajectory/ugv0/path`
- 轨迹重合区域：话题 `/trajectory/markers`；两台机器人重合为黄色小点，三台机器人重合为较大的白色点。

轨迹节点已默认加入 `human_ai_vlm_stage5.launch`，原来的启动命令不需要修改。随后启动 RViz：

```bash
roslaunch uav_semantic_search rviz_stage2.launch
```

修改后的 RViz 配置已经默认显示三条轨迹和重合区域。若无需轨迹记录，可在主启动命令中加入：

```bash
enable_trajectory_visualization:=false
```

开始新一轮实验前，可清空上一轮轨迹：

```bash
rosservice call /robot_trajectory_visualizer/reset
```

主要参数位于 `human_ai_vlm_stage5.launch`：

- `min_sample_distance_m`：相邻采样点的最小距离，默认 `0.08 m`，用于抑制静止噪声。
- `overlap_radius_m`：不同机器人轨迹判为重合的空间阈值，默认 `0.30 m`。
- `publish_rate_hz`：可视化刷新频率，默认 `2 Hz`。
- `colours`：三台机器人的 RGB 颜色，数值范围为 `[0,1]`。

如只希望看到彩色历史轨迹而不显示重合标记，可在 RViz 的 Displays 面板中取消勾选 `Trajectory Overlap`。
