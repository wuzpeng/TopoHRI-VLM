# Stage 2：双机语义目标搜索工作空间与使用教程

## 1. 本阶段目标与边界

Stage 2 的目标是让已能按固定航点飞行的两架 PX4 无人机，形成如下闭环：

```text
3D LiDAR → 双机局部 2.5D 地图 → 中央全局 2.5D 占据地图
RGB-D → 目标检测 → 深度反投影 → map 坐标目标观测
双机目标观测 → 空间关联 + 多帧验证 → 目标候选/确认 → RViz 语义标记
```

本包**暂不实现**未知环境前沿探索、自动房间分配、VLM 任务分解或真实受害者深度检测。这些模块应当建立在本阶段的“地图可信、目标可定位、观测可融合”基础之上。

## 2. 总体架构

```text
                  ┌──────────────────────────────┐
                  │ PX4 + Gazebo Classic × 2      │
                  │ iris_0 / iris_1               │
                  └──────────────┬───────────────┘
                                 │
      ┌──────────────────────────┼────────────────────────────┐
      │                          │                            │
  LiDAR PointCloud2          RGB + Depth + K              /gazebo/model_states
      │                          │                            │
      ▼                          ▼                            ▼
central_map_fuser      semantic_detector × 2       gazebo_pose_bridge
      │                          │                            │
      ▼                          ▼                            │
/global_map_2d    /uavX/semantic/target_observation         │
                                 │                            │
                                 └──────► target_fusion_node ◄┘
                                                 │
             ┌───────────────────────────────────┼──────────────────────────┐
             ▼                                   ▼                          ▼
/semantic_map/target_hypotheses    /semantic_map/confirmed_targets   /semantic_map/target_markers
```

任务层仍然复用已有接口：

```text
任务管理器 / 固定 demo
        ↓  geometry_msgs/PoseStamped, frame_id=map
/uavX/mission/goal
        ↓
waypoint_executor.py
        ↓
/uavX/mavros/setpoint_position/local
        ↓
PX4 OFFBOARD
```

## 3. 工作空间文件结构

```text
uav_semantic_search_ws/
└── src/
    └── uav_semantic_search/
        ├── config/
        │   ├── system.yaml                  # 两机、地图、相机外参、检测/融合阈值
        │   ├── demo_mission.yaml            # 基础走廊航线
        │   └── target_search_mission.yaml   # Stage-2 目标房间搜索航线
        ├── launch/
        │   ├── stack.launch                 # 基础 PX4/MAVROS/建图/航点执行层
        │   ├── semantic_stage2.launch       # 基础层 + 两机语义检测 + 目标融合
        │   └── rviz_stage2.launch           # Stage-2 RViz 可视化
        ├── msg/
        │   ├── TargetObservation.msg
        │   ├── TargetHypothesis.msg
        │   └── TargetHypothesisArray.msg
        ├── scripts/
        │   ├── central_map_fuser.py
        │   ├── gazebo_pose_bridge.py
        │   ├── waypoint_executor.py
        │   ├── semantic_detector.py
        │   ├── target_fusion_node.py
        │   ├── semantic_search_demo_mission.py
        │   ├── publish_synthetic_target.py
        │   ├── patch_px4_gazebo.py
        │   ├── restore_px4_gazebo.py
        │   ├── run_px4_two_uav.sh
        │   └── configure_px4_sitl_offboard.py
        ├── worlds/corridor_rooms.world
        ├── rviz/stage2_semantic_search.rviz
        └── docs/README_STAGE2_CN.md
```

## 4. 与当前 PX4 环境的对应关系

本包默认适配你当前 `PX4_Firmware_13` 的多机 MAVLink 端口：

```text
PX4 instance 0：PX4 监听 34580，MAVROS 本地绑定 24540
PX4 instance 1：PX4 监听 34581，MAVROS 本地绑定 24541
```

因此 `launch/stack.launch` 默认使用：

```text
uav0: udp://:24540@127.0.0.1:34580
uav1: udp://:24541@127.0.0.1:34581
```

这些数值来自你当前 `px4-rc.mavlink` 的：

```bash
udp_offboard_port_local=$((34580+px4_instance))
udp_offboard_port_remote=$((24540+px4_instance))
```

以后若你更换 PX4 树，必须先检查：

```bash
export PX4_ROOT=$HOME/PX4_Firmware_13
grep -nE "udp_offboard_port_local|udp_offboard_port_remote" \
  "$PX4_ROOT/ROMFS/px4fmu_common/init.d-posix/px4-rc.mavlink"
```

并据此修改 `stack.launch` 的 `uav0_fcu_url`、`uav1_fcu_url`。

---

# 5. 安装与替换步骤

## 5.1 备份当前已能运行的版本

你已经跑通基础 demo，因此先备份当前包：

```bash
cd ~/harp_sar_ws/src
mv uav_semantic_search uav_semantic_search_before_stage2
```

解压 Stage-2 工作空间后，将其中的包拷贝到 `src`：

```bash
cp -r /path/to/uav_semantic_search_stage2_ws/src/uav_semantic_search \
  ~/harp_sar_ws/src/
```

随后编译：

```bash
cd ~/harp_sar_ws
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
```

> `msg/` 新增了自定义消息，因此这一步必须执行；仅替换 Python 文件而不重新编译，会导致 `TargetObservation` 等消息不可用。

## 5.2 安装 Stage-2 依赖

```bash
source /opt/ros/noetic/setup.bash
source ~/harp_sar_ws/devel/setup.bash

roscd uav_semantic_search
./scripts/install_dependencies.sh
```

核心新增依赖包括：

```text
cv_bridge
OpenCV
image_transport
rviz
rqt_image_view
velodyne_gazebo_plugins
```

验证关键 LiDAR 插件存在：

```bash
find /opt/ros/noetic -name 'libgazebo_ros_velodyne_laser.so' -print
```

应包含：

```text
/opt/ros/noetic/lib/libgazebo_ros_velodyne_laser.so
```

## 5.3 将 Stage-2 传感器/场景写入 PX4 Gazebo 树

先完全停止旧的 PX4、Gazebo 与 MAVROS：

```bash
pkill -f mavros_node || true
pkill -f px4 || true
pkill -f gzserver || true
pkill -f gzclient || true
```

再执行安全 patch：

```bash
export PX4_ROOT=$HOME/PX4_Firmware_13
source ~/harp_sar_ws/devel/setup.bash

rosrun uav_semantic_search patch_px4_gazebo.py \
  --px4-root "$PX4_ROOT"
```

该脚本会：

1. 备份一次 `iris.sdf.jinja` 或 `iris.sdf`；
2. 写入 16 线 `ray` + `libgazebo_ros_velodyne_laser.so` LiDAR；
3. 写入 RGB-D 相机与深度图话题；
4. 使用 `mavlink_id|int` 避免之前遇到的 Jinja 字符串减法错误；
5. 安装去除了错误 world plugin 的 `corridor_rooms.world`；
6. 保留 `*.uav_semantic_search.bak` 作为回滚点。

若需要回滚：

```bash
rosrun uav_semantic_search restore_px4_gazebo.py \
  --px4-root "$PX4_ROOT"
```

---

# 6. 启动 Stage-2 完整系统

建议按以下顺序启动。

## 终端 1：ROS 基础层、双 MAVROS、建图、语义检测和目标融合

```bash
source /opt/ros/noetic/setup.bash
source ~/harp_sar_ws/devel/setup.bash

roslaunch uav_semantic_search semantic_stage2.launch autostart:=true
```

该终端启动：

```text
/uav0/mavros, /uav1/mavros
/gazebo_pose_bridge
/central_map_fuser
/uav0/waypoint_executor, /uav1/waypoint_executor
/uav0/semantic_detector, /uav1/semantic_detector
/target_fusion_node
```

## 终端 2：PX4 双机 SITL 和 Gazebo 场景

```bash
source /opt/ros/noetic/setup.bash
source ~/harp_sar_ws/devel/setup.bash

rosrun uav_semantic_search run_px4_two_uav.sh
```

脚本会自动配置：

```text
PX4_ROOT=$HOME/PX4_Firmware_13
GAZEBO_PLUGIN_PATH=/opt/ros/noetic/lib:...
LD_LIBRARY_PATH=/opt/ros/noetic/lib:...
```

并以以下位置生成无人机：

```text
iris_0 / uav0 : (2.0, -0.7)
iris_1 / uav1 : (2.0,  0.7)
```

这两个位置均在走廊内。

## 终端 3：检查 MAVROS 并设置无人值守 OFFBOARD 参数

等两架 Iris 出现在 Gazebo 中后：

```bash
source /opt/ros/noetic/setup.bash
source ~/harp_sar_ws/devel/setup.bash

rostopic echo -n 1 /uav0/mavros/state
rostopic echo -n 1 /uav1/mavros/state
```

两个状态都必须先出现：

```text
connected: True
```

若还没有配置过 SITL 的 RC-loss 相关参数，执行一次：

```bash
rosrun uav_semantic_search configure_px4_sitl_offboard.py
```

它为 `uav0`、`uav1` 设置：

```text
COM_RC_IN_MODE = 1
COM_RCL_EXCEPT = 4
```

之后查看：

```bash
rostopic hz /uav0/mavros/setpoint_position/local
rostopic hz /uav1/mavros/setpoint_position/local
```

应出现约 `30 Hz` 的连续 setpoint 流。随后状态应趋向：

```text
connected: True
armed: True
mode: "OFFBOARD"
```

## 终端 4：打开 RViz

```bash
source ~/harp_sar_ws/devel/setup.bash
roslaunch uav_semantic_search rviz_stage2.launch
```

RViz 中应显示：

```text
/global_map_2d
/uav0/local_map_2d
/uav1/local_map_2d
/uav0/lidar/points
/uav1/lidar/points
/semantic_map/target_markers
```

## 终端 5：运行目标搜索 demo

```bash
source ~/harp_sar_ws/devel/setup.bash
rosrun uav_semantic_search semantic_search_demo_mission.py
```

该航线的关键设计：

```text
uav0：沿走廊南侧推进，形成协同建图背景；
uav1：沿走廊北侧推进，并从 x≈25 的门口进入北侧目标房间；
uav1：最后在目标前方保持朝 +x 的机体朝向，使红色目标进入前向 RGB-D 相机视场。
```

---

# 7. 运行后应看到的输出

## 7.1 传感器与地图

```bash
rostopic hz /uav0/lidar/points
rostopic hz /uav1/lidar/points
rostopic echo -n 1 /global_map_2d/info
```

期望：

```text
/uavX/lidar/points：有效 PointCloud2，实际频率通常 ≥5 Hz
/global_map_2d：nav_msgs/OccupancyGrid
```

## 7.2 RGB-D 与检测调试图

```bash
rostopic hz /uav1/camera/rgb/image_raw
rostopic hz /uav1/camera/depth/image_raw
rostopic echo -n 1 /uav1/camera/rgb/camera_info

rqt_image_view /uav1/semantic/debug_image
```

当目标被看到时，调试图上会出现黄色候选框和蓝色中心点。

## 7.3 单机语义目标观测

```bash
rostopic echo /uav1/semantic/target_observation
```

其中关键字段为：

```text
robot_id: "uav1"
class_name: "victim_surrogate"
confidence: ...
pose: map 坐标系中的目标位置
pixel_u / pixel_v: 目标图像中心
bbox_*: 检测框
```

## 7.4 中央目标融合结果

```bash
rostopic echo /semantic_map/target_hypotheses
rostopic echo /semantic_map/confirmed_targets
```

状态解释：

| 状态 | 触发条件 | 含义 |
|---|---|---|
| `candidate` | 少量单帧观测 | 初始候选，不应直接用于宣布搜索成功 |
| `verified` | 至少 3 次融合观测 | 同一机器人多帧稳定观测 |
| `confirmed` | 至少 4 次观测且来自 2 架机，或观测总数达到 10 | 可作为高层任务完成或重分配依据 |

目前的 Stage-2 固定航线主要确保 `uav1` 能形成 `verified`。若希望很快得到 `confirmed`，可以让 `uav0` 也进入该房间，或临时使用下述合成观测测试融合逻辑。

---

# 8. 目标定位的几何逻辑

检测器先从深度图获得目标点深度 `d`，再由相机内参 `(fx, fy, cx, cy)` 反投影：

```text
p_optical = [ (u-cx)d/fx, (v-cy)d/fy, d ]^T
```

随后使用固定相机外参与无人机当前全局位姿：

```text
p_map = T_map_body · T_body_camera · p_optical
```

其中：

- `T_map_body`：来自 `/uavX/global_pose`；当前由 Gazebo 真值位姿桥接提供；
- `T_body_camera`：由 `config/system.yaml` 中的 `camera_xyz` 和 `camera_optical_to_body_rpy` 给出；
- `p_map`：最终写入 `TargetObservation.pose` 的全局目标坐标。

当前采用 Gazebo 真值位姿是为了先验证语义定位与融合算法，而不是表示真实系统无需 SLAM。以后替换为 FAST-LIO、VIO 或其他状态估计时，只需要让 `/uavX/global_pose` 继续以同样的 `map` 坐标系发布位姿。

---

# 9. 合成目标融合测试

在不飞行、不依赖相机的情况下，可单独验证融合节点：

```bash
# 终端 A：模拟 uav0 在目标处连续观测
rosrun uav_semantic_search publish_synthetic_target.py \
  _robot:=uav0 _x:=26.0 _y:=5.7 _z:=0.35

# 终端 B：模拟 uav1 在相邻位置观测同一目标
rosrun uav_semantic_search publish_synthetic_target.py \
  _robot:=uav1 _x:=26.15 _y:=5.65 _z:=0.35
```

然后查看：

```bash
rostopic echo /semantic_map/confirmed_targets
```

应出现包含两个 `observed_by` 条目的 `confirmed` 目标。

---

# 10. 关键可调参数

所有参数集中在：

```bash
roscd uav_semantic_search
nano config/system.yaml
```

## 10.1 检测阈值

```yaml
semantic_detector:
  min_area_px: 180
  min_depth_m: 0.35
  max_depth_m: 7.50
  red_hsv_lower_1: [0, 100, 70]
  red_hsv_upper_1: [10, 255, 255]
```

目标太小导致检测不到：降低 `min_area_px`，例如 `100`。

目标误检太多：提高 `min_area_px`，并提高 HSV 中的饱和度/亮度下限。

## 10.2 多机器人关联阈值

```yaml
target_fusion:
  association_radius_m: 1.20
```

两次观测的坐标差小于该距离，且类别一致时，会被融合到同一目标假设中。

若实际投影点偏差较大但仍确认是同一目标，可以暂时调为 `1.5`；不要一开始设置过大，否则不同房间的红色物体也可能被错误合并。

## 10.3 相机外参

```yaml
camera_xyz: [0.18, 0.0, -0.04]
camera_optical_to_body_rpy: [-1.57079632679, 0.0, -1.57079632679]
```

若 RViz 中的语义球标记持续出现在目标后方或左右镜像位置，优先检查：

1. 无人机当前机体朝向是否与预设航点 yaw 一致；
2. RGB-D 相机实际安装方向；
3. 该相机到机体的外参；
4. ROS optical frame 到 PX4 FLU body frame 的旋转。

不要通过随意修改全局目标坐标“修正”问题；应当修改 `T_body_camera`。

---

# 11. 常见故障排查

## 11.1 `/uavX/mavros/state` 的 `connected: False`

说明 MAVROS 端口与 PX4 实际端口不匹配。先检查 PX4：

```bash
grep -nE "udp_offboard_port_local|udp_offboard_port_remote" \
  "$HOME/PX4_Firmware_13/ROMFS/px4fmu_common/init.d-posix/px4-rc.mavlink"
```

再更新 `launch/stack.launch` 中的 FCU URL。

## 11.2 无人机有模型但 `AUTO.RTL`、无法解锁

先确认 setpoint 有连续输出：

```bash
rostopic hz /uav0/mavros/setpoint_position/local
```

再运行：

```bash
rosrun uav_semantic_search configure_px4_sitl_offboard.py
```

查看 PX4 拒绝原因：

```bash
rostopic echo /uav0/mavros/statustext/recv
```

## 11.3 LiDAR 插件加载失败

检查：

```bash
find /opt/ros/noetic -name 'libgazebo_ros_velodyne_laser.so' -print
```

并确认 `run_px4_two_uav.sh` 中已设置：

```bash
export GAZEBO_PLUGIN_PATH="/opt/ros/noetic/lib:${GAZEBO_PLUGIN_PATH:-}"
export LD_LIBRARY_PATH="/opt/ros/noetic/lib:${LD_LIBRARY_PATH:-}"
```

## 11.4 相机话题存在但没有目标观测

依次检查：

```bash
rostopic hz /uav1/camera/rgb/image_raw
rostopic hz /uav1/camera/depth/image_raw
rostopic echo -n 1 /uav1/camera/rgb/camera_info
rqt_image_view /uav1/semantic/debug_image
```

还需要保证 `uav1` 真正进入北侧目标房间并以朝 `+x` 的方向停在目标前方。仅沿走廊直飞不会看到位于房间内部的红色替身。

## 11.5 `target_observation` 有数据，但 RViz 标记位置明显不对

这通常是相机外参或光学坐标轴旋转问题，而不是 target fusion 的问题。先在 RViz 中同时显示：

```text
/uav1/global_pose
/uav1/semantic/debug_image
/semantic_map/target_markers
```

然后根据目标在图像中的位置、无人机航向及投影结果校正 `camera_optical_to_body_rpy`。

---

# 12. Stage-2 验收标准

完成本阶段时，至少应满足：

1. 两架无人机可按 `semantic_search_demo_mission.py` 的航点完成飞行；
2. `/uav0/lidar/points`、`/uav1/lidar/points` 持续发布；
3. `/global_map_2d` 随飞行逐渐形成走廊、房间墙体和障碍的占据图；
4. `uav1` 进入目标房间后，`/uav1/semantic/debug_image` 出现红色目标候选框；
5. `/uav1/semantic/target_observation` 输出合理的 map 坐标点；
6. `/semantic_map/target_hypotheses` 至少形成 `verified` 目标；
7. 使用两机观测或合成观测时，`/semantic_map/confirmed_targets` 输出 `confirmed` 目标；
8. RViz 中目标标记与目标替身所在房间位置基本一致。

达到这些条件后，下一阶段才适合加入：

```text
全局 frontier 提取
→ 任务效用评估
→ 双机区域分配
→ 自动生成 /uavX/mission/goal
→ 基于语义目标置信度的验证、重分配与任务终止
→ VLM/LLM 高层任务推理
```
