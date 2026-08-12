# Stage-5 v0.5.1 启动修复

本补丁修复 Stage-5 v0.5.0 的两处启动缺陷：

1. `/gazebo/pause_physics` 和 `/gazebo/unpause_physics` 使用 `std_srvs/Empty`，而不是 `gazebo_msgs/Empty`。
2. UAV 配置补充 `type: uav` 与 `mission_reached_topic`，并让触发调度器兼容旧 Stage-4 配置。

同时修复暂停 Gazebo 后语义叠加层的定时发布不再推进的问题：本地 VLM 报告到达后会立即发布语义摘要，中央规划器也会直接合并本 epoch 的局部报告。

重新编译：

```bash
cd ~/harp_sar_ws
source /opt/ros/noetic/setup.bash
catkin_make --force-cmake
source devel/setup.bash
```
