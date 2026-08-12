# Topo 与状态–动作 Gate 消融测试

## 1. 四种配置

基于相同地图、目标位置、随机种子和 VLM 参数分别运行：

```bash
# Full
roslaunch uav_semantic_search human_ai_vlm_stage5.launch \
  experiment_config:=$(rospack find uav_semantic_search)/config/experiments/h4.yaml \
  enable_topology_planning:=true enable_state_action_gate:=true \
  run_name:=h4_full_01

# Without Topo
roslaunch uav_semantic_search human_ai_vlm_stage5.launch \
  experiment_config:=$(rospack find uav_semantic_search)/config/experiments/h4.yaml \
  enable_topology_planning:=false enable_state_action_gate:=true \
  run_name:=h4_no_topo_01

# Without Gate
roslaunch uav_semantic_search human_ai_vlm_stage5.launch \
  experiment_config:=$(rospack find uav_semantic_search)/config/experiments/h4.yaml \
  enable_topology_planning:=true enable_state_action_gate:=false \
  run_name:=h4_no_gate_01

# 可选：同时关闭两者
roslaunch uav_semantic_search human_ai_vlm_stage5.launch \
  experiment_config:=$(rospack find uav_semantic_search)/config/experiments/h4.yaml \
  enable_topology_planning:=false enable_state_action_gate:=false \
  run_name:=h4_no_topo_no_gate_01
```

## 2. 开关实际作用

`enable_topology_planning:=false`：

- 关闭基于拓扑区域的候选多样性预筛选；
- 不向 Central VLM 暴露拓扑字段和拓扑分配提示；
- 关闭 validator 的拓扑区域容量约束；
- 仍后台生成区域标签，仅供原始 RCR 评估，不参与决策。

`enable_state_action_gate:=false`：

- 目标已确认时不再在候选构建阶段删除移动候选；
- 在相应事件下，即使存在 frontier/目标候选，也允许生成 QUERY_RESCAN；
- 从 Central VLM 提示中移除状态–动作优先级规则；
- 关闭 validator 的任务优先级修复；
- 候选 ID、机器人匹配、实时几何可达性等执行安全检查仍保留。

## 3. CVR 与 RCR

两个指标都在收到 `/vlm/central_plan` 时立即基于 VLM 原始
`plan.assignments` 计算，完全不读取 `/vlm/validated_plan`。

```text
CVR_raw = 原始可评估动作中的状态–动作违反数 / 原始可评估动作数
RCR_raw = 原始EXPLORE分配中的可避免同区域机器人对 / 可评估机器人对
```

未知机器人或不存在的候选 ID 不混入 CVR，单独累计为
`raw_invalid_decisions`。同一个动作即使同时触发多条违反原因，在 CVR
分子中仍只计一次；各原因另外记录在 `cvr_reason_counts`。

RCR 仅统计非人类直接指定的 EXPLORE 机器人对。只有当两台机器人当时
存在可选择不同区域的组合时，该机器人对才进入分母。

逐决策记录位于 `decision_metrics.csv`，最终累计结果位于 `summary.json`
和 `summary.csv`。跨多次试验应优先报告池化比率：

```text
CVR_pooled = sum(cvr_violating_decisions) / sum(cvr_raw_decisions)
RCR_pooled = sum(rcr_conflicting_pairs) / sum(rcr_eligible_pairs)
```

不要先对每次试验的百分比做简单平均作为主结果，因为不同试验的有效
动作数或有效机器人对数量不同。
