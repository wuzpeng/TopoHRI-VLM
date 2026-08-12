# Stage-5：分层 VLM 引导的空地异构语义搜索

## 1. 版本定位

本工作空间完整保留 Stage-4 的 FUEL-style 空地异构探索 baseline：

```bash
roslaunch uav_semantic_search heterogeneous_fuel_stage3.launch autostart:=true
```

Stage-5 新增独立的 VLM 运行模式：

```bash
roslaunch uav_semantic_search heterogeneous_vlm_stage5.launch autostart:=true
```

Stage-5 不删除或覆盖 FUEL baseline 的节点、地图、A*、PX4、Go2、LiDAR、RGB-D 与安全控制逻辑。它仅用新的高层 VLM 协同决策链路替代 `heterogeneous_fuel_manager.py` 的固定 frontier utility 与规则分配。

---

## 2. 已实现的在线闭环

```text
UAV / UGV 连续 LiDAR 建图 + RGB-D 接收
        ↓
轻量触发器：地图变化、位姿/视域变化、图像新颖性、航点完成、阻塞、任务查询切换
        ↓
暂停 Gazebo（可配置）
        ↓
触发机器人的本地 VLM：单张最佳 RGB 关键帧 + 深度/位姿/查询上下文
        ↓
稀疏 SemanticOverlay：与 UAV/UGV 几何地图同一 map 坐标系
        ↓
集中式 VLM：角色分配 + 任务类型 + 安全候选航点选择
        ↓
二次 A* 验证
        ↓
发布既有 /uavX/mission/goal、/ugv0/mission/goal
        ↓
恢复 Gazebo，继续在线探索
```

### 关键边界

- **本地 VLM**：负责单机器人视觉语义理解，不做全局任务分配。
- **中央 VLM**：负责团队级角色分配、任务组织、候选航点选择，不输出任意新坐标。
- **候选生成器**：使用 robot-specific occupancy map 生成安全候选点；只生成可行动作，不决定最终选择。
- **验证器**：再次执行 A* 可达性检查；VLM 结果无效时采用最近安全候选点或 HOLD。
- **执行层**：完全复用 Stage-4 PX4、Go2、A*、waypoint executor 与安全门控。

---

## 3. 触发机制

大 VLM 不处理连续每一帧。`vlm_trigger_scheduler.py` 持续运行轻量模块，并在以下事件触发语义决策周期：

1. UAV 起飞完成、UAV/UGV 地图和 RGB-D 上下文首次就绪；
2. 机器人到达当前航点；
3. UGV 报告前向阻塞；
4. 机器人相对上一次语义解释移动超过阈值；
5. 偏航变化后，RGB-D 估计的当前可视地图区域中存在足够比例的新语义覆盖；
6. HSV 颜色直方图表明当前画面显著不同；
7. 当前几何地图新增明显自由空间；
8. 局部自由空间扇区结构出现拓扑变化提示；
9. 通过 `/vlm/set_target_query` 切换开放词汇目标。

当前版本的视觉新颖性使用无需额外依赖的 HSV 直方图距离。后续可以把该轻量门控替换为 DINOv2、NetVLAD 或 CLIP embedding，但不改变整体 ROS 接口。

---

## 4. 语义地图不是第三张 OccupancyGrid

原有两张几何地图继续保留：

```text
/global_map_2d          UAV 固定飞行高度带可通行性
/ugv0/ground_map_2d     UGV 近地可通行性
```

VLM 语义信息发布为稀疏 `SemanticOverlay`：

```text
/semantic_overlay/summary
/semantic_overlay/markers
```

其中包含：已语义解释的覆盖栅格、门口/家具/障碍等对象记录、对象地图位置、跨平台观测来源、目标候选证据与查询版本。它不修改 `OccupancyGrid.data` 的 `-1/0/100` 语义，因此不会破坏 A*、障碍膨胀或 frontier 提取。

---

## 5. VLM 后端

配置文件：

```text
config/vlm_semantic_search.yaml
```

默认：

```yaml
backend:
  mode: mock
```

`mock` 模式可完整验证 ROS 数据流、触发、暂停、语义叠加、候选生成、中央计划、A* 验证和恢复执行，但不会产生真实视觉语义理解。

切换到视觉语言模型时：

```yaml
backend:
  mode: openai_compatible
  base_url: "http://127.0.0.1:8000/v1"
  api_key_env: VLM_API_KEY
  model: "your-vision-language-model"
```

然后在启动终端设置：

```bash
export VLM_API_KEY='你的密钥'
```

后端必须实现 OpenAI-compatible `/chat/completions`，并支持 user message 中的 `image_url` data URL。系统仅要求模型返回 JSON；不绑定特定厂商或特定模型。

---

## 6. 启动步骤

### 6.1 保留原 baseline

当前工作空间无需删除 baseline。建议复制为新工作空间，例如：

```bash
cd ~/harp_sar_ws
cp -a src/uav_semantic_search src/uav_semantic_search_stage4_backup
```

将本包的 `src/` 覆盖或解压到新的 Stage-5 工作空间后：

```bash
cd ~/harp_sar_ws
rm -rf build devel logs
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
```

### 6.2 启动 VLM 模式

终端 1：

```bash
source /opt/ros/noetic/setup.bash
source ~/harp_sar_ws/devel/setup.bash
roslaunch uav_semantic_search heterogeneous_vlm_stage5.launch autostart:=true
```

终端 2：

```bash
source /opt/ros/noetic/setup.bash
source ~/harp_sar_ws/devel/setup.bash
rosrun uav_semantic_search run_px4_two_uav.py
```

---

## 7. 在线切换搜索目标

例如切换为搜索黄色急救箱：

```bash
rostopic pub -1 /vlm/set_target_query std_msgs/String \
"data: '{\"query_text\": \"寻找黄色急救箱\", \"query_type\": \"first_aid_kit\", \"ground_verification_required\": false}'"
```

切换为搜索穿红色上衣的倒地人员：

```bash
rostopic pub -1 /vlm/set_target_query std_msgs/String \
"data: '{\"query_text\": \"寻找一名穿红色上衣、可能倒在地面上的受困人员\", \"query_type\": \"person\", \"ground_verification_required\": true}'"
```

目标查询改变会生成新的 query version，并触发一个同步 VLM 决策周期。默认情况下，UAV0、UAV1、UGV0 会各自对其当前视角执行一次本地 VLM 语义理解；历史语义对象保留在 Overlay 中，但新查询相关目标证据从新版本重新积累。

---

## 8. 主要调试话题

```bash
rostopic echo /vlm/trigger_status
rostopic echo /vlm/sync_status
rostopic echo /vlm/target_query
rostopic echo /vlm/local_semantic_observation
rostopic echo /semantic_overlay/summary
rostopic echo /vlm/central_plan
rostopic echo /vlm/validated_plan
```

关键图像：

```bash
rqt_image_view /uav0/vlm/debug_image
rqt_image_view /uav1/vlm/debug_image
rqt_image_view /ugv0/vlm/debug_image
```

当前阶段默认使用 mock backend，因此 debug 图像会发布原始帧，但不会出现真实 VLM 对象框。接入真实 VLM 后，模型返回 `entities[].bbox` 或 `target_evidence.bbox` 时会在 debug 图像上绘制框。

---

## 9. 当前实现范围与后续扩展

已实现：

- Stage-4 基线保留；
- 线上目标文本切换；
- 单关键帧本地 VLM 接口；
- RGB-D + 位姿将 VLM bbox 映射到 map 坐标；
- 稀疏语义叠加层；
- 中央 VLM 安全候选选择接口；
- 同步 Gazebo 暂停/恢复；
- 二次 A* 验证与安全 fallback；
- frontier、inspection、verification、scan-in-place 候选动作。

当前仍为第一版工程实现：

- 拓扑变化触发是基于局部自由空间扇区的几何提示，不等同于已完成的房间分割；
- 视觉新颖性默认是 HSV 直方图距离，尚未接入 DINOv2/NetVLAD；
- mock backend 用于验证管线，不代表真实 VLM 性能；
- `REQUEST_OBSERVATION` 被落地为预生成的 `INSPECT` / `SCAN_IN_PLACE` 候选点，复杂的多视角 NBV 采样可作为下一步扩展；
- 同步暂停机制适用于当前静态室内仿真，不适合实时动态目标场景。
