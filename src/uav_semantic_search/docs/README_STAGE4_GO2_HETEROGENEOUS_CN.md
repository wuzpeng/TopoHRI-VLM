# Stage-4：Go2 导航级模型与异构 FUEL-style 协同搜索

## 1. 版本定位

本版本在已整合 Stage-3 v0.3.1/v0.3.2/v0.3.3 修复的基础上，新增一台地面四足机器人：

```text
ugv0 = Unitree official Unitree Go2 navigation proxy
```

默认模型具有 Go2 风格的机身尺寸、四足视觉外形、近地 LiDAR 安装位置和机体碰撞包络，但**不包含真实足端接触、关节力矩、步态生成或动力学行走控制**。其地面移动由 `go2_kinematic_controller.py` 将 `/ugv0/cmd_vel` 转换为 Gazebo 中的有界平面运动。

因此，本版本研究的对象是：

```text
异构搜索、地图构建、可通行性判断、任务分配与航点执行，
而不是四足步态控制。
```

## 2. 地图与规划层级

```text
uav0 / uav1 LiDAR
        ↓
/global_map_2d
        ↓
固定高度 UAV FUEL-style frontier + A* 规划

ugv0 近地 LiDAR
        ↓
/ugv0/ground_map_2d
        ↓
Go2 ground FUEL-style frontier + A* 规划
```

两张地图均从未知状态增量构建；它们不应直接合并：

- `/global_map_2d`：适合固定高度无人机。由 UAV LiDAR 构建，不把 Go2 低矮障碍纳入 UAV 可飞行判断。
- `/ugv0/ground_map_2d`：仅用于 Go2。将 `z=0.08~0.78 m` 范围内的回波作为地面障碍候选，过滤地板。

`heterogeneous_fuel_manager.py` 同时运行两套 FUEL-style 循环：

```text
前沿提取 → 信息增益视点 → A* → 稀疏航点 → 到点反馈 → 增量重规划
```

其中 UAV0/UAV1 在空中图层上联合分配前沿，减少重叠并平衡路径负载；Go2 在自身近地图上独立选择可达地面前沿。

## 3. 新增主要文件

```text
models/go2_nav_proxy/model.sdf
scripts/go2_nav_spawner.py
scripts/go2_pose_bridge.py
scripts/go2_kinematic_controller.py
scripts/ground_map_fuser.py
scripts/ugv_waypoint_executor.py
scripts/heterogeneous_fuel_manager.py
config/go2_nav.yaml
launch/heterogeneous_fuel_stage3.launch
```

## 4. 安装

本压缩包是**完整整合版**，请先备份当前包：

```bash
cd ~/harp_sar_ws/src
mv uav_semantic_search uav_semantic_search_before_stage4
```

解压后将压缩包中的 `src/uav_semantic_search` 复制到：

```text
~/harp_sar_ws/src/uav_semantic_search
```

编译：

```bash
cd ~/harp_sar_ws
source /opt/ros/noetic/setup.bash
rm -rf build/uav_semantic_search devel/lib/uav_semantic_search
catkin_make --pkg uav_semantic_search --force-cmake
source devel/setup.bash
```

确认新节点：

```bash
ls ~/harp_sar_ws/devel/lib/uav_semantic_search | \
  grep -E 'go2|ground_map|ugv_waypoint|heterogeneous'
```

应看到：

```text
go2_nav_spawner.py
go2_pose_bridge.py
go2_kinematic_controller.py
ground_map_fuser.py
ugv_waypoint_executor.py
heterogeneous_fuel_manager.py
```

## 5. 启动

### 终端 1：ROS、UAV 感知栈、Go2、异构 FUEL 管理器

```bash
source /opt/ros/noetic/setup.bash
source ~/harp_sar_ws/devel/setup.bash
roslaunch uav_semantic_search heterogeneous_fuel_stage3.launch autostart:=true
```

### 终端 2：Gazebo 与双 PX4 SITL

```bash
source /opt/ros/noetic/setup.bash
source ~/harp_sar_ws/devel/setup.bash
rosrun uav_semantic_search run_px4_two_uav.py
```

Go2 spawner 会等待 `/gazebo/spawn_sdf_model` 服务；Gazebo 就绪后应输出：

```text
Spawned navigation-level Go2 model go2_0 at [...].
```

若这是新的 PX4 SITL 会话，待 MAVROS 连接后按原流程配置室内参数：

```bash
rosrun uav_semantic_search configure_px4_sitl_offboard.py
```

## 6. 必要验证

```bash
rostopic echo -n 1 /ugv0/global_pose
rostopic hz /ugv0/lidar/points
rostopic hz /ugv0/ground_map_2d
rostopic echo /ugv0/mission/goal
rostopic echo /ugv0/mission/reached
rostopic echo /ugv0/safety/blocked
rostopic echo /search/heterogeneous_status
```

正常状态示例：

```text
UAV: 4 frontiers | UGV: 3 frontiers
ugv0 FUEL-style goal [x, y, 0.34], ugv_frontier_002 1/3.
ugv0 reached ground goal.
```

## 7. RViz 显示

在已有 RViz 中增加：

```text
PointCloud2: /ugv0/lidar/points
Map:         /ugv0/ground_map_2d
Path:        /ugv0/search/planned_path
MarkerArray: /search/heterogeneous_fuel_markers
Pose:        /ugv0/global_pose
```

颜色约定：

```text
蓝色球：UAV 图层前沿视点
绿色球：Go2 地面图层前沿视点
青色/粉色圆柱：UAV 当前任务
绿色圆柱：Go2 当前任务
```

## 8. 关键安全边界

Go2 使用 `set_model_state` 的导航级运动学更新，不具备真实腿部接触动力学。安全机制包括：

```text
地面占据地图上的已知自由空间 A*；
前方 LiDAR 扇区紧急停止；
低速线速度与角速度限制；
局部阻塞后释放任务并等待重新规划。
```

因此，当前模型适合验证高层异构协同逻辑。它不应被用于评估真实 Go2 步态稳定性、足端碰撞、台阶跨越能力或动力学能耗。


> **Stage-4.1 note:** use `README_STAGE4_1_OFFICIAL_GO2_CN.md`. The previous hand-built visual proxy is deprecated and is no longer the default model.
