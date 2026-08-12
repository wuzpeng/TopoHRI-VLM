# Stage-3 起飞安全门与保守室内速度修复（v0.3.3）

## 修复的问题

早期 Stage-3 中，`autonomous_search_manager.py` 只按固定延时等待地图；而
`waypoint_executor.py` 收到第一条 frontier 航点后会立即用该远距离目标替换初始起飞设定点。
因此无人机可能尚未升到安全高度就同时出现水平与垂直运动，形成斜向起飞，容易撞击走廊墙壁。

## 修复后的飞行顺序

1. `waypoint_executor.py` 首先锁定起飞时的本地 XY，只向上发布垂直起飞 setpoint；
2. PX4 进入 OFFBOARD 并解锁后，无人机爬升至各自 `takeoff_height`；
3. 到达高度误差不超过 `takeoff_tolerance_m`，并持续悬停 `takeoff_hold_sec`；
4. 节点发布 `/uavX/mission/takeoff_ready=True`；
5. `autonomous_search_manager.py` 等待两架 UAV 都 ready，再进行 `post_takeoff_map_bootstrap_sec` 秒地图扫描；
6. 最后才开始 frontier 分配与 A* 航点发布。

此外，修复包将 Stage-3 A* 障碍物膨胀提高到约 `0.60 m`，并将 PX4 室内速度限制为：
`MPC_XY_VEL_MAX=1.2 m/s`、`MPC_Z_VEL_MAX_UP=0.7 m/s`、`MPC_Z_VEL_MAX_DN=0.5 m/s`、`MPC_ACC_HOR=1.0 m/s²`。

## 覆盖文件

- `scripts/waypoint_executor.py`
- `scripts/autonomous_search_manager.py`
- `scripts/configure_px4_sitl_offboard.py`
- `config/system.yaml`
- `config/racer_stage3.yaml`

不修改 `CMakeLists.txt`，因此不会触发此前的 Catkin Python/Shell 包装器问题。
