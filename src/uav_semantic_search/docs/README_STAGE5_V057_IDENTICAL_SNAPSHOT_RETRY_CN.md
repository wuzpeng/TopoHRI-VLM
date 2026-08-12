# Stage-5 v0.5.7：本地 VLM 同快照有限重试

## 解决的问题

当单个机器人本地 VLM 请求出现临时 timeout、连接重置、HTTP 429 或 HTTP 5xx 时，v0.5.6 会立即终止当前 epoch。v0.5.7 对该机器人增加有限重试，以提升第三方云端 VLM 的瞬时抖动恢复能力。

## 重试原则

- **同一 epoch 的同一份快照**：第一次调用前缓存 RGB、深度、位姿、地图摘要和 prompt；后续重试不重新读取 ROS 相机话题。
- **发送相同图像数据**：不缩放、不降低 JPEG 质量、不减少 `max_tokens`，不改变模型或 prompt。
- **有限重试**：默认“首发 + 最多 1 次重试”。
- **指数退避**：默认等待 1.5 秒后重试。
- **总时限**：默认单机器人 75 秒；协调器等待 80 秒，避免在重试尚未结束时误判该机器人缺失。
- **不重试配置性错误**：401、403、400、404、持续无 JSON 等错误会立即失败。
- **中央 VLM 保护**：任一本地 VLM 最终失败时，仍跳过中央 VLM，保持上一条有效任务。

## 默认参数

```yaml
local_retry:
  enabled: true
  max_retries: 1
  attempt_timeout_sec: 35.0
  total_deadline_sec: 75.0
  initial_backoff_sec: 1.5
  backoff_multiplier: 2.0
```

`max_retries: 1` 表示最多两次同数据请求。建议先观察成功率，再根据云端接口稳定性增加到 `2`；不要无限增加重试次数。

## 日志示例

首次 timeout 后：

```text
Local VLM ugv0 attempt 1/2 failed in epoch epoch_xxx; retrying identical snapshot in 1.5s: ...
```

第二次成功后：

```text
Local VLM ugv0 completed epoch epoch_xxx with N entities.
```

语义报告附带：

```json
"vlm_attempt_count": 2,
"vlm_retry_count": 1
```
