# UAV 航向跟随迁移说明

本版本以 `uav_semantic_search(4).zip` 为基础，只迁移已验证参考包中的
UAV 航向控制链，不引入点云—位姿时间阈值、点云丢弃或新的 LiDAR 建图
逻辑。

## 控制链

1. `astar_route_planner.py` 为中间路径点写入下一路径段航向，并为最终
   路径点写入候选目标的 `yaw_rad`。
2. `waypoint_executor.py` 每个控制周期根据 UAV 当前地图位置与当前路径
   点重新计算实际飞行方向。
3. 接近拐点时，航向平滑融合至下一路径段。
4. 接近终点时，飞行方向平滑融合至最终观测方向。
5. 航向变化按 `max_yaw_rate_rad_s` 限速后，转换为四元数并随位置目标
   发布到 MAVROS。

## 配置

全局航向配置位于 `config/experiment_runtime.yaml` 的
`experiment_runtime/uav_yaw_control`。每架 UAV 可在 `config/system.yaml`
的 `yaw_control` 中覆盖全局参数。

`launch/stack.launch` 会在启动 UAV 执行器之前加载上述全局配置。

## 保留内容

本版本完整保留 `(4)` 中的四张实验地图、拓扑 Frontier、人机交互、
分层 VLM、UGV 导航和各地图实验配置。
