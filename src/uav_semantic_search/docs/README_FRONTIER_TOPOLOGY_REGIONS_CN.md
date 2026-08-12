# 基于自由空间拓扑骨架的 Frontier 区域标注

## 1. 功能闭环

本版本在原有 frontier 连通域聚类基础上增加以下确定性处理：

1. 对机器人膨胀后的 `passable` 自由空间执行 Zhang-Suen 骨架化；
2. 修剪短毛刺，识别骨架端点和分叉点；
3. 在分叉点处分割骨架，生成自由空间骨架分支；
4. 使用多源、八邻域自由空间传播，将每个 frontier 挂接到最近的可通行骨架分支；
5. 为 EXPLORE 候选写入 `topology_region_id`；
6. 候选截断时先保留不同拓扑区域，再补充同一区域备选；
7. Central VLM 按区域身份进行联合分配；
8. Validator 在存在其他安全可达区域时拒绝同区域重复分配；
9. Validator 的自动补全分配同样优先使用尚未占用的区域。

`topology_region_id` 与人类框选区域 `human_priority_regions` 是两套独立概念，
不会互相覆盖。

## 2. 候选新增字段

```json
{
  "topology_region_id": "uav_topology:R003",
  "topology_branch_id": 3,
  "topology_layer": "uav_topology",
  "topology_association_distance_m": 0.8,
  "topology_confidence": "HIGH"
}
```

无法可靠关联到骨架时，使用当前 frontier 独占的低置信度编号：

```text
uav_topology:UNASSIGNED:F005
```

因此两个机器人对同一个未关联 frontier 的副本仍可被 Validator 识别，
但不同未关联 frontier 不会被错误合并。

## 3. 运行方式

重新编译工作空间：

```bash
cd ~/harp_sar_ws
catkin_make
source devel/setup.bash
```

按原有方式启动 Stage-5。`frontier_extractor.py` 会继续发布：

```text
/search/frontier_viewpoints
/search/frontier_markers
```

终端会周期性输出：

```text
Topology: skeleton=1842 branches=9 frontier=6 regions=4 unassigned=0
```

RViz 标记含义：

- 黄色点：自由空间骨架；
- 红色点：骨架分叉点；
- 白色点：骨架端点；
- 不同颜色的 frontier 球：不同拓扑区域；
- `F3 / R2`：frontier 3 属于拓扑区域 R2。

注意：`topology_show_skeleton: false` 只关闭黄色骨架显示，不关闭区域计算。

## 4. 推荐调参顺序

### 骨架短枝过多

逐步提高：

```yaml
topology_spur_prune_length_m: 0.6
topology_min_branch_length_m: 0.4
```

每次建议增加 0.2 m。不要一次提高过多，否则刚探索出的短通道会消失。

### 大量 frontier 显示 UNASSIGNED

先检查黄色骨架是否位于已知自由空间中央，再适当提高：

```yaml
topology_frontier_association_max_distance_m: 4.0
```

室内场景建议保持在 3–5 m，过大会增加错误关联风险。

### 同一区域候选仍然过多

降低：

```yaml
max_frontiers_per_topology_region_per_robot: 2
```

可以设为 1，但通常建议保留 2 个不同观察点供 VLM 选择。

### 地图只有一个区域时机器人没有任务

保持：

```yaml
topology_allow_shared_region_if_no_alternative: true
```

此时只有不存在其他安全可达区域，并且目标点间距达到
`topology_shared_region_min_goal_separation_m`，才允许共享区域。

## 5. 排查步骤

1. 检查日志中的 `branches` 是否大于 0；
2. 检查 frontier 标签是否包含 `Rxxx`；
3. 查看 `/vlm/central_plan` 的 `candidate_catalog` 是否包含 `topology_region_id`；
4. 查看 `/vlm/validated_plan` 中每个 accepted assignment 的区域编号；
5. 如果 VLM 选择重复区域，Validator 应记录：

```text
topology_region_already_assigned
```

6. 如果地图确实只剩一个可达区域，检查共享目标之间是否至少相距 2.2 m。

## 6. 修改文件

- `scripts/frontier_topology.py`：新增骨架及区域关联算法；
- `scripts/frontier_core.py`：扩展 FrontierCluster 拓扑字段；
- `scripts/vlm_candidate_builder.py`：区域标注与多样化预筛选；
- `scripts/central_vlm_planner.py`：将区域身份传给 Central VLM；
- `scripts/vlm_common.py`：拓扑感知 mock 分配；
- `scripts/vlm_plan_validator.py`：区域冲突检查与 fallback；
- `scripts/frontier_extractor.py`：RViz 骨架与区域可视化；
- `config/racer_stage3.yaml`：骨架与可视化参数；
- `config/vlm_semantic_search.yaml`：候选和 Validator 参数；
- `CMakeLists.txt`：安装新增导入模块。
