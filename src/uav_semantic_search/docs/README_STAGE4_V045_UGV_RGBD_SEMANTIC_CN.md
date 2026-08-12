# Stage-4 v0.4.5：Go2 前向 RGB-D 语义检测扩展

本增量包基于 Stage-4 v0.4.4，向导航级 Go2 代理模型增加前向 RGB-D 相机，并使其复用现有 UAV 的 `semantic_detector.py` 红色目标候选检测与 RGB-D 地图定位链路。

## 新增能力

```text
Go2 前向 RGB-D 相机
  -> /ugv0/camera/camera/rgb/image_raw
  -> /ugv0/camera/camera/depth/image_raw
  -> /ugv0/camera/camera/rgb/camera_info
  -> /ugv0/semantic/target_observation
  -> /semantic_map/target_hypotheses
  -> /semantic_map/confirmed_targets
```

Go2 观测使用 `/ugv0/global_pose` 与配置的相机外参转换到统一 `map` 坐标系。`target_fusion_node.py` 现同时订阅 UAV 与 UGV 的 observation 话题，因此 Go2 的观测可参与跨平台目标关联和确认。

## 设计边界

- Go2 仍是导航级、运动学平面代理；该扩展不增加四足步态控制。
- 视觉算法仍是现有 HSV 红色 surrogate 检测器，未替换为学习式目标检测网络。
- 当前 `heterogeneous_fuel.stop_on_confirmed_target: true` 保持不变。目标融合输出 `confirmed` 后，异构管理器会使所有机器人 HOLD。若要实现“UAV 候选 -> UGV 最终复核”，应在后续单独调整确认状态机，而不是仅修改相机模块。

## 修改文件

```text
config/go2_nav.yaml
models/go2_nav_proxy/model.sdf
scripts/semantic_detector.py
scripts/target_fusion_node.py
launch/heterogeneous_fuel_stage3.launch
```

无需修改 `CMakeLists.txt`：`semantic_detector.py` 原本已安装为 ROS 可执行节点，`models/` 原本已由安装规则一并安装。

## 重新编译与启动

```bash
cd ~/harp_sar_ws
catkin_make --pkg uav_semantic_search --force-cmake
source devel/setup.bash
```

关闭并重新启动 Gazebo/PX4/ROS，因为 Go2 的 RGB-D 传感器写入 `model.sdf`，仅重启单个检测节点不会让 Gazebo 已生成的旧模型加载新传感器。

```bash
source /opt/ros/noetic/setup.bash
source ~/harp_sar_ws/devel/setup.bash
roslaunch uav_semantic_search heterogeneous_fuel_stage3.launch autostart:=true
```

另一终端：

```bash
source /opt/ros/noetic/setup.bash
source ~/harp_sar_ws/devel/setup.bash
rosrun uav_semantic_search run_px4_two_uav.py
```

## 验证步骤

### 1. 验证 RGB-D 话题

```bash
rostopic hz /ugv0/camera/camera/rgb/image_raw
rostopic hz /ugv0/camera/camera/depth/image_raw
rostopic echo -n 1 /ugv0/camera/camera/rgb/camera_info
```

预期 RGB 和 depth 均约为 15 Hz。若实际 topic 名与预期不同，先执行：

```bash
rostopic list | grep '/ugv0.*camera'
```

然后将 `config/go2_nav.yaml` 中的 `rgb_topic`、`depth_topic`、`camera_info_topic` 改为实际话题并重新 launch。

### 2. 查看图像与调试图

```bash
rqt_image_view /ugv0/camera/camera/rgb/image_raw
rqt_image_view /ugv0/semantic/debug_image
```

Go2 相机沿机体 +x 前方安装。将红色 `victim_surrogate` 放在 Go2 当前朝向前方、约 0.35--7.5 m 处，可在调试图中看到黄色候选框。

### 3. 验证目标检测与融合

```bash
rostopic echo /ugv0/semantic/target_observation
rostopic echo /semantic_map/target_hypotheses
rostopic echo /semantic_map/confirmed_targets
```

单帧检测会发布 `TargetObservation`；融合节点以 `robot_id: ugv0` 记录观测来源。当前配置允许多机器人交叉观测达到 confirmed，也允许单一机器人积累足够观测数后强制 confirmed。

## 常见问题

### 没有任何 `/ugv0/camera/...` 话题

首先确认 Go2 已在重新启动后的 Gazebo 中生成：

```bash
rostopic echo -n 1 /gazebo/model_states
```

如果 LiDAR 正常而相机完全没有话题，检查 Gazebo 终端是否加载 `libgazebo_ros_openni_kinect.so` 失败。当前 UAV RGB-D 已使用同一插件，因此正常情况下 ROS Noetic 的 `gazebo_plugins` 已安装。

### 有 RGB 图像，但没有语义观测

检查 Go2 是否有 map 位姿：

```bash
rostopic echo -n 1 /ugv0/global_pose
```

检查 depth 和 CameraInfo 是否存在，并确认红色区域面积大于 `/semantic_detector/min_area_px`。调试图持续输出而没有候选框，通常说明颜色阈值或目标可见面积不足。

### UGV 检测后全部机器人停止

这是当前 `stop_on_confirmed_target: true` 与目标融合 `confirmed` 状态的既有全局任务完成逻辑，不是相机故障。该行为适合“确认一个目标即结束本轮任务”的测试；若需要 Go2 对 UAV 候选目标执行专门复核，应改为 `candidate -> aerial_confirmed -> ground_verified` 状态机。