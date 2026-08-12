# Stage-3：中央化 RACER-inspired 双无人机自主探索基线

## 1. 本阶段新增的能力

Stage-2 已经完成：双无人机按人工设定航点飞行、LiDAR 共享建图、RGB-D 红色目标定位和多帧目标融合。

Stage-3 不再使用固定任务路线，而是让中央管理器根据实时 `/global_map_2d` 自动选择航点：

```text
共享 2.5D 占据地图
→ 提取已知自由区与未知区之间的 frontier
→ 聚类并生成安全观察点
→ 中央 hgrid 区域去重
→ 信息增益、路径代价、风险与负载均衡联合评分
→ 为 uav0/uav1 选择不同搜索区域
→ A* 生成安全栅格路径并压缩为 map 航点
→ waypoint_executor.py
→ PX4 OFFBOARD
→ 新地图到达后持续重新规划
```

该版本名称为：

```text
Centralized RACER-Inspired Cooperative Exploration (CRICE)
```

它借鉴 RACER 的在线区域分解、避免重复覆盖和负载均衡思想，但采用本工程已有的共享中央地图和中央任务分配器；它不是原始去中心化 RACER 的逐行复现。

---

## 2. 新增程序文件

```text
scripts/frontier_core.py
```

纯算法工具库，提供：

```text
OccupancyGrid → NumPy 网格
frontier 提取与连通域聚类
安全观察点选择
局部未知空间增益估计
障碍物膨胀
2D A*
路径视线压缩
```

```text
scripts/frontier_extractor.py
```

诊断与可视化节点。它从 `/global_map_2d` 中提取有效 frontier，并发布：

```text
/search/frontier_viewpoints      geometry_msgs/PoseArray
/search/frontier_markers         visualization_msgs/MarkerArray
```

该节点不发布飞行航点；其目的是先在 RViz 中验证“未知区域边界”和“候选观察点”是否合理。

```text
scripts/autonomous_search_manager.py
```

Stage-3 的核心决策节点。它完成：

```text
frontier 聚类
+ hgrid 区域归属
+ 联合双机任务分配
+ 负载均衡
+ A* 路径规划
+ 顺序发布 map 航点
+ 到点后的下一段路径推进
+ 地图变化后的事件/周期重规划
+ 确认目标后的停止探索
```

它是当前唯一应当发布：

```text
/uav0/mission/goal
/uav1/mission/goal
```

的上层节点。

```text
scripts/search_metrics_logger.py
```

可选实验记录节点。它以 CSV 记录：

```text
仿真时间
已知地图比例
两机位置
当前搜索状态
confirmed target 数量
```

用于后续比较：Nearest Frontier、Information-Gain、CRICE、VLM 方法。

```text
config/racer_stage3.yaml
```

Stage-3 的全部参数，包括：

```text
frontier 长度阈值
障碍物膨胀半径
hgrid 尺寸
信息增益半径
效用函数权重
负载均衡权重
任务去重距离
A* 最大搜索量
目标确认后的停机策略
```

```text
launch/racer_stage3.launch
```

启动 Stage-2 全部感知与执行层，再加载 Stage-3 前沿可视化和中央自主探索管理器。

---

## 3. 与当前 Stage-2 接口的关系

Stage-3 不修改下层飞控和感知接口：

```text
/global_map_2d                         来自 central_map_fuser.py
/uavX/global_pose                      来自 gazebo_pose_bridge.py
/uavX/mission/reached                  来自 waypoint_executor.py
/uavX/semantic/target_observation      来自 semantic_detector.py
/semantic_map/confirmed_targets         来自 target_fusion_node.py
/uavX/mission/goal                     仍由 waypoint_executor.py 执行
```

新的链路仅替代原先人工航点发布器：

```text
原 Stage-2：
semantic_search_demo_mission.py
→ /uavX/mission/goal

Stage-3：
autonomous_search_manager.py
→ /uavX/mission/goal
```

因此，**运行 Stage-3 时不能同时启动 `semantic_search_demo_mission.py`、`corridor_demo_mission.py` 或任何手动航点发布脚本。** 否则多个发布者会相互覆盖目标点。

---

## 4. 安装/替换方式

建议先备份已经跑通的版本：

```bash
cd ~/harp_sar_ws/src
mv uav_semantic_search uav_semantic_search_before_stage3
```

将本包中的 `src/uav_semantic_search` 拷贝到：

```text
~/harp_sar_ws/src/uav_semantic_search
```

然后编译：

```bash
cd ~/harp_sar_ws
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
```

本版本没有增加新的 ROS 自定义消息，因此不涉及新的 `.msg` 定义；但新增的 Python 节点必须通过 `catkin_make` 安装到 `devel/lib/uav_semantic_search/`，否则 `roslaunch` 可能仍执行旧副本。

---

## 5. 启动方式

### 终端 1：启动 Stage-3 自主探索栈

```bash
source /opt/ros/noetic/setup.bash
source ~/harp_sar_ws/devel/setup.bash

roslaunch uav_semantic_search racer_stage3.launch autostart:=true
```

该终端启动：

```text
/uav0/mavros, /uav1/mavros
/gazebo_pose_bridge
/central_map_fuser
/uav0/waypoint_executor, /uav1/waypoint_executor
/uav0/semantic_detector, /uav1/semantic_detector
/target_fusion_node
/frontier_extractor
/autonomous_search_manager
```

### 终端 2：启动 PX4 双机 SITL 与 Gazebo 场景

```bash
source /opt/ros/noetic/setup.bash
source ~/harp_sar_ws/devel/setup.bash

rosrun uav_semantic_search run_px4_two_uav.py
```

### 终端 3：首次运行时设置无人值守 OFFBOARD 参数

当两个 MAVROS 节点均 `connected: True` 后：

```bash
source /opt/ros/noetic/setup.bash
source ~/harp_sar_ws/devel/setup.bash

rosrun uav_semantic_search configure_px4_sitl_offboard.py
```

该设置通常只需在每套 SITL 参数配置中执行一次。

---

## 6. 正常运行时应看到什么

等待 `startup_delay_sec` 和初始 LiDAR 建图完成后，终端 1 应出现类似：

```text
Frontier extractor publishes 3 valid clusters.
Stage-3 plan: 3 frontiers; assignments=[('uav0', 1, 4.2, ...), ('uav1', 3, 6.1, ...)]
uav0 RACER-style goal [x, y, 1.80] for frontier_001.
uav1 RACER-style goal [x, y, 2.20] for frontier_003.
```

随后会看到：

```text
uavX reached current map goal.
uavX completed frontier_XXX.
Stage-3 plan: ...
```

这表明系统已形成：

```text
地图更新 → 自动航点 → 飞行 → 到达 → 再规划
```

的闭环。

---

## 7. 关键可视化与检查命令

### 查看管理器状态

```bash
rostopic echo /search/status
```

常见状态：

```text
WAIT_MAP
WAIT_POSES
MAP_BOOTSTRAP
EXPLORE: N frontiers, M new assignments
TARGET_CONFIRMED
MISSION_COMPLETE_NO_FRONTIER
```

### 检查前沿候选观察点

```bash
rostopic echo -n 1 /search/frontier_viewpoints
```

### 查看每架无人机的 A* 路径

```bash
rostopic echo -n 1 /uav0/search/planned_path
rostopic echo -n 1 /uav1/search/planned_path
```

### RViz 中添加以下显示项

```text
Map:
/global_map_2d

MarkerArray:
/search/frontier_markers
/search/racer_markers
/semantic_map/target_markers

Path:
/uav0/search/planned_path
/uav1/search/planned_path
```

颜色含义：

```text
蓝色球：有效 frontier 候选观察点
青色圆柱：uav0 当前分配的 frontier 终点
粉色圆柱：uav1 当前分配的 frontier 终点
路径：A* 转换后的任务路径
```

---

## 8. 关键参数及第一轮调参建议

所有参数在：

```text
config/racer_stage3.yaml
```

### `obstacle_inflation_m`

当前默认：

```yaml
obstacle_inflation_m: 0.40
```

作用：把墙体/障碍物向外扩张，保证 A* 路径不贴墙。

- 无人机无法穿过走廊：减小到 `0.30`；
- 路径太贴近墙体：增大到 `0.45~0.55`。

### `min_frontier_length_m`

当前默认：

```yaml
min_frontier_length_m: 0.60
```

作用：过滤小噪声 frontier。

- 没有任何 candidate：尝试减小到 `0.40`；
- candidate 过多且在地图噪声处跳动：增大到 `0.80~1.00`。

### `gain_radius_m`

当前默认：

```yaml
gain_radius_m: 3.20
```

表示候选点局部可获得多少未知区域收益的近似统计半径。

- 更偏向局部近距离探索：减小；
- 更偏向优先进入能看到大空间的门口/房间：增大。

### `min_assignment_separation_m`

当前默认：

```yaml
min_assignment_separation_m: 2.20
```

确保两架无人机不会被分给太接近的候选点。

- 两机仍经常前往同一门口：增大到 `3.0`；
- 环境很小导致只有一架机工作：减小到 `1.5`。

### `load_balance_weight`

当前默认：

```yaml
load_balance_weight: 0.80
```

该项惩罚两架机的路径成本差：

\[
J_{balance}=\mu\left|L_{uav0}-L_{uav1}\right|.
\]

- 一架机总被分配远任务：增大；
- 过度追求均衡、忽略高信息增益房间：减小。

---

## 9. 运行阶段与当前边界

本版本已经实现：

```text
单机/双机自主 frontier 选择
中央 hgrid 区域去重
信息增益 + 距离 + 风险 + 负载均衡分配
A* 路径与航点序列
事件/周期性重规划
确认目标后停止派发新探索任务
```

本版本尚未实现：

```text
门口的严格时空预约
连续轨迹层多机避碰优化
多 frontier 批量 CVRP 路径排序
candidate/verified target 自动复核航线
基于房间语义和目标先验的搜索概率地图
VLM/LLM 任务优先级调整
```

其中最推荐的后续实现顺序是：

```text
先验证 Stage-3 自主 frontier 双机探索稳定
→ 再增加 candidate target 出现后的 uav0/uav1 复核分工
→ 再增加房间级语义表示和 VLM 高层决策
```

---

## 10. 可选：记录实验 CSV

启动时增加：

```bash
roslaunch uav_semantic_search racer_stage3.launch \
  autostart:=true \
  record_metrics:=true \
  run_name:=crice_run_01
```

默认写入：

```text
~/.ros/uav_semantic_search_metrics/
```

每秒记录一次地图覆盖率、无人机位置、当前状态和确认目标数。这些日志可以直接用于后续统计：首次发现时间、确认时间、覆盖率—时间曲线、累计路径长度和重复覆盖率。
