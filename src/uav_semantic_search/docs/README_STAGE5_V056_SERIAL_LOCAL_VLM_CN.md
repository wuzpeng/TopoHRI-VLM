# Stage-5 v0.5.6：启动查询屏障与串行本地 VLM 调度

本修复针对云端 OpenAI-compatible VLM 服务的两个运行时问题：

1. `/vlm/target_query` 是 latched topic。调度器订阅后会收到一次初始查询；旧逻辑把它误认为真实的 `TARGET_QUERY_CHANGE`，可能在 UAV 起飞与全部传感器准备完成前创建一个空的 VLM epoch。
2. `INITIAL_CONTEXT_READY` 与真实目标切换需要 UAV0、UAV1、UGV0 都进行本地视觉理解。旧逻辑一次广播给三个本地节点，三个图像请求同时进入同一个云端 API，容易造成服务端排队并使其中一个请求超时。

## v0.5.6 行为

- 第一条 latched `/vlm/target_query` 仅初始化调度器内部状态，不触发 VLM epoch。
- 只有 `INITIAL_CONTEXT_READY` 已完成后，后续人工发布 `/vlm/set_target_query` 才会触发 `TARGET_QUERY_CHANGE`。
- 多机器人本地 VLM 请求默认顺序执行：`uav0 -> uav1 -> ugv0`。
- 每一个机器人产生本地报告或超时后，才向下一个机器人发送图像请求。
- Gazebo/PX4 在云端推理期间继续运行；若任何本地报告失败，当前 epoch 不调用中央 VLM，也不覆盖上一条有效航点。

## 安装

在已经安装 Stage-5 v0.5.5 的工作空间上覆盖增量包后执行：

```bash
cd ~/harp_sar_ws/src/uav_semantic_search
python3 scripts/upgrade_stage5_v056_serial_local_config.py

cd ~/harp_sar_ws
catkin_make --force-cmake
source devel/setup.bash
```

升级脚本会备份 `config/vlm_semantic_search.yaml`，并保留其中的 API Key、URL、模型名与目标查询。

## 推荐云端参数

```yaml
backend:
  timeout_sec: 45.0

local_dispatch:
  mode: sequential
  participant_order: [uav0, uav1, ugv0]
  per_robot_response_timeout_sec: 50.0
  inter_request_delay_sec: 0.30
```

`per_robot_response_timeout_sec` 必须大于 `backend.timeout_sec`。三台机器人顺序调用时，首次上下文建立的最坏等待时间可能约为三倍单请求超时；但实际云端调用成功时通常远低于这个上限，且不会再发生三路图像请求相互排队。

## 检查

启动后，正常不应再看到 UAV 起飞完成前的：

```text
VLM epoch ... local perception timeout; missing ['uav0', 'uav1', 'ugv0']
```

首次 epoch 中可看到：

```text
VLM epoch ... dispatches local perception serially to uav0 (1/3).
VLM epoch ... dispatches local perception serially to uav1 (2/3).
VLM epoch ... dispatches local perception serially to ugv0 (3/3).
```

通过以下话题查看状态：

```bash
rostopic echo /vlm/sync_status
rostopic echo /vlm/local_semantic_observation
rostopic echo /vlm/central_plan
```
