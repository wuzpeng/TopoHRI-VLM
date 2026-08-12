# Stage-5 v0.5.3：任务完成门控与本地 VLM 启动屏障修复

## 修复内容

1. `GOAL_REACHED` 不再仅依赖 `/robot/mission/reached` 的 `False -> True` 边沿。
   验证器会在发布任务航点前发送 `/vlm/goal_dispatch`，调度器只对新的、需要移动完成的任务（`EXPLORE`、`INSPECT`、`GROUND_VERIFY`、`AERIAL_INSPECT`）重新武装一次到达事件。`SCAN_IN_PLACE` 与 `HOVER_AND_SCAN` 不会形成下一轮无限触发。
2. 同一 `candidate_id + task_type + goal pose` 的重复发布不会重新武装。
3. 每个本地 VLM 节点发布 latched `/robot/vlm/ready`。触发器在三个本地 VLM 均已订阅请求后才产生首次 `INITIAL_CONTEXT_READY`，避免 launch 启动时 `uav1` 或 `ugv0` 丢失首次本地请求，从而不生成初始 debug 图像。

## 预期

- 同一航点的完成只能产生一条 `TRIGGERED:GOAL_REACHED:<robot>`。
- 在首次 VLM epoch 后，三个 `/robot/vlm/debug_image` 均为 latched，之后启动 `rqt_image_view` 也能显示。
- 调试：`rostopic echo /vlm/goal_dispatch` 可以确认每次航点任务的 candidate 和 task_type。
