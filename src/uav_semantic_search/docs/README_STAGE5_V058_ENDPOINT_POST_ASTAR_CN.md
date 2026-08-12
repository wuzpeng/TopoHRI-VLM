# Stage-5 v0.5.8：端点选择与后置 A* 路径规划

本版本基于 v0.5.7，遵循严格的分层接口：

```text
FUEL-style 前沿/语义候选端点生成
→ VLM 或确定性降级模块选择 candidate_id
→ Validator 复核候选端点
→ A* Route Planner 从当前机器人位姿和当前地图重新规划
→ UGV/UAV 执行器逐路径点跟踪
```

因此：

- VLM **只选择目标候选点**，不输出坐标、不输出路径；
- 候选生成阶段可用 A* 估计可达性和路径代价，仅用于筛选与提供属性；
- 真正执行的 A* 路径在 VLM 返回后、以最新位姿和最新地图重新计算；
- `/robot/search/planned_path` 是 A* route planner 的输出，不是 VLM 输出；
- 本地/中央 VLM 后端失败时，`vlm_map_fallback_planner` 只选择另一个 FUEL-style 候选端点，随后仍由 A* route planner 规划并执行路线。

## 关键新增节点

- `astar_route_planner.py`：从当前状态到已选 endpoint 重新运行 A*，发布 `nav_msgs/Path`；
- `vlm_map_fallback_planner.py`：VLM 失败后的确定性 endpoint 选择器；
- 更新的 `vlm_plan_validator.py`：只发布 endpoint route request，不直接下发最终 Pose goal；
- 更新的 UAV/UGV executors：逐路径点跟踪。

## 查看链路

```bash
rostopic echo /vlm/validated_plan
rostopic echo /vlm/route_request
rostopic echo /vlm/route_result
rostopic echo /uav0/search/planned_path
rostopic echo /uav1/search/planned_path
rostopic echo /ugv0/search/planned_path
```

UGV 正常日志示例：

```text
A* route ready for ugv0 candidate=ugv0_F_3 with 4 waypoint(s).
ugv0 accepted A* route with 4 waypoint(s).
ugv0 advances A* route waypoint 2/4.
```

VLM 超时后的降级日志示例：

```text
MAP_FALLBACK:LOCAL_BACKEND_FAILURE:epoch_xxx
Map fallback plan produced for epoch epoch_xxx: 1 assignment(s)...
A* route ready for ugv0 candidate=ugv0_F_5 with ... waypoint(s).
```
