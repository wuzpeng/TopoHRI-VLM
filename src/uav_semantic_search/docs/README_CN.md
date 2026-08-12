# uav_semantic_search：PX4 双无人机 LiDAR/RGB-D 室内地图融合包

## 已实现内容

本工作空间对应“**两架 PX4 无人机在 Gazebo 走廊—房间场景中，挂载 3D LiDAR 与 RGB-D；各自构建局部图并融合为统一 2.5D 占据图；能够接收 map-frame 航点并由 PX4 Offboard 执行**”这一基础工程目标。

包内包含：

- `corridor_rooms.world`：一条长走廊、两侧 6 个房间、家具和一个高可见度受害人员替身；
- `patch_px4_gazebo.py`：对当前使用的 PX4 Iris 模型做**可恢复的原位传感器注入**；
- 两台 UAV 的 MAVROS 配置、Offboard 执行器、地图接口；
- 两张本地 2.5D 占据图及一个融合后的 `/global_map_2d`；
- 可选的走廊航点示例。

当前地图融合阶段使用 Gazebo 模型位姿建立统一 `map` 坐标系，用于先验证点云、地图和控制链路。它不是 FAST-LIO2/多机 SLAM 的替代品。集成 SLAM 后，停止 `gazebo_pose_bridge.py`，使用 `external_pose.launch` 将 SLAM Odometry 转为 `/uavX/global_pose`。

## 关键约束

**一次 Gazebo 运行只能使用一个 PX4 目录。**建议你的 Ubuntu 20.04 + v1.13 环境优先选：

```bash
export PX4_ROOT=$HOME/PX4_Firmware_13
```

不要混合 `PX4-Autopilot` 的 build/plugin 路径和 `PX4_Firmware_13` 的模型路径，否则 Gazebo Classic 易出现插件 ABI 或模型加载错误。

## 1. 复制并编译

```bash
cd ~/harp_sar_ws/src
cp -r /path/to/uav_semantic_search .
cd ~/harp_sar_ws
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
```

依赖安装：

```bash
roscd uav_semantic_search
./scripts/install_dependencies.sh
```

## 2. 注入传感器和安装 Gazebo 世界

关闭所有 Gazebo/PX4 实例后：

```bash
export PX4_ROOT=$HOME/PX4_Firmware_13
roscd uav_semantic_search
python3 scripts/patch_px4_gazebo.py --px4-root "$PX4_ROOT"
```

脚本会备份 `iris.sdf.jinja`（或 `iris.sdf`），然后在原 Iris 模型中注入：

- 10 Hz、16线、水平 360°、20 m 量程的 Gazebo `gpu_ray` 3D LiDAR；
- 15 Hz、640×480、前向 80° RGB-D 相机；
- 按 `mavlink_id` 生成的独立 ROS 命名空间。

应得到：

```text
/uav0/lidar/points                  sensor_msgs/PointCloud2
/uav1/lidar/points                  sensor_msgs/PointCloud2
/uav0/camera/rgb/image_raw          sensor_msgs/Image
/uav1/camera/rgb/image_raw          sensor_msgs/Image
/uav0/camera/depth/image_raw        sensor_msgs/Image
/uav1/camera/depth/image_raw        sensor_msgs/Image
```

恢复原模型：

```bash
python3 scripts/restore_px4_gazebo.py --px4-root "$PX4_ROOT"
```

## 3. 启动

### 终端 1：ROS、双 MAVROS、地图节点和 Offboard 执行器

```bash
source /opt/ros/noetic/setup.bash
source ~/harp_sar_ws/devel/setup.bash
roslaunch uav_semantic_search stack.launch autostart:=true
```

### 终端 2：双 PX4 SITL 和室内 Gazebo 场景

```bash
source /opt/ros/noetic/setup.bash
source ~/harp_sar_ws/devel/setup.bash
export PX4_ROOT=$HOME/PX4_Firmware_13
rosrun uav_semantic_search run_px4_two_uav.sh
```

其本质命令为：

```bash
cd "$PX4_ROOT"
./Tools/gazebo_sitl_multiple_run.sh -n 2 -m iris -w corridor_rooms
```

### 终端 3：检查接口

```bash
rostopic list | egrep 'uav[01]/(lidar|camera|mavros)|global_map'
rostopic hz /uav0/lidar/points
rostopic hz /uav1/camera/rgb/image_raw
rostopic echo -n 1 /global_map_2d/info
```

### 终端 4：可选的完整航点烟雾测试

终端 1 不要再重复启动 `stack.launch`，只运行：

```bash
source ~/harp_sar_ws/devel/setup.bash
rosrun uav_semantic_search corridor_demo_mission.py
```

## 核心 ROS 接口

### 地图

```text
/uav0/local_map_2d        nav_msgs/OccupancyGrid
/uav1/local_map_2d        nav_msgs/OccupancyGrid
/global_map_2d            nav_msgs/OccupancyGrid
/global_map_uav_poses     geometry_msgs/PoseArray
```

其中：`-1=unknown`、`0=free`、`100=occupied`。地图融合是在 evidence 层完成的，而不是把两张已阈值化图简单覆盖。

### 航点输入与执行反馈

```text
/uav0/mission/goal        geometry_msgs/PoseStamped, frame_id=map
/uav1/mission/goal        geometry_msgs/PoseStamped, frame_id=map
/uav0/mission/reached     std_msgs/Bool
/uav1/mission/reached     std_msgs/Bool
/uav0/mission/status      std_msgs/String
/uav1/mission/status      std_msgs/String
```

手动航点示例：

```bash
rostopic pub -1 /uav0/mission/goal geometry_msgs/PoseStamped "{header: {frame_id: 'map'}, pose: {position: {x: 8.0, y: -0.7, z: 1.8}, orientation: {w: 1.0}}}"
```

## 当前阶段与后续分层框架的对接

当前包对应第三层的“传感器—地图—安全执行底座”。后续模块按顺序对接：

1. FAST-LIO2 / LIO-SAM：替换 Gazebo pose bridge；
2. RGB-D 目标检测和定位：写入全局语义地图；
3. frontier、骨架、房间入口提取：构造全局语义—拓扑地图；
4. 第一层大模型：输出 `uav0/uav1` 子任务；
5. 第二层大模型：从几何候选航点集合中选取并排序；
6. 将选择结果持续发布到本包已有的 `/uavX/mission/goal`，保持底层 PX4 执行器不变。

## 已知限制与排障

- 传感器注入依赖 PX4 v1.13 典型的 Jinja 变量 `mavlink_id`。如果你的多机脚本不使用该变量，请先打开其生成的 Iris SDF 确认；这种情况下可把 `robotNamespace` 固定改成对应模型名称后再运行。
- 如果 `/gazebo/model_states` 不出现，确认 world 中能加载 `/opt/ros/noetic/lib/libgazebo_ros_api_plugin.so`，并在启动 PX4 前 `source /opt/ros/noetic/setup.bash`。
- 如果 RViz/ROS 中没有 LiDAR 或相机话题，确认 `ros-noetic-gazebo-plugins` 已安装，并停止 Gazebo 后重新执行 patch 脚本。重新构建/启动 PX4 会重新渲染 Jinja SDF。
- 如果 `OFFBOARD` 被拒绝，检查 setpoint 话题是否持续达到 30 Hz、`/uavX/mavros/state` 是否 `connected: True`，以及实例端口是否与已有多机配置一致。
- 2.5D 图通过过滤地面与天花板点生成，适用于当前单层室内场景；它不表示多层建筑的完整三维可通行性。
