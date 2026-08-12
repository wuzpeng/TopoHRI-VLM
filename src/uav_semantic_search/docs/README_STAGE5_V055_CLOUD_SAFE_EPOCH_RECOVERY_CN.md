# Stage-5 v0.5.5：云端 VLM 低频运行与异常恢复

本补丁针对云端 OpenAI-compatible VLM 的高延迟/超时问题，避免 VLM 调用暂停 Gazebo/PX4/MAVROS，并避免失败后由几何触发器立刻重复发起新 epoch。

## 核心修改

1. 默认云端安全模式不暂停 Gazebo 物理引擎；机器人继续执行上一条有效任务，VLM 成功后才下发新任务。
2. 本地 VLM 失败或 mock fallback 时仍发布结构化报告，调度器更新语义基线并对对应机器人进入 cooldown。
3. 若触发机器人本地 VLM 超时/失败，协调器默认跳过中央 VLM 调用并恢复正常执行，避免“本地 20 秒超时 + 中央 20 秒超时”的双重等待。
4. 中央 VLM 失败时会发布 BACKEND_ERROR 包；验证器不再发布 fallback 移动目标，而是维持上一条已验证任务。
5. 默认关闭距离、HSV 视觉新颖性、地图自由空间扩展和拓扑变化的自动触发，只保留初始上下文、目标查询切换、任务到达和 UGV 阻塞等离散高价值事件。

## 安装后执行

```bash
cd ~/harp_sar_ws/src/uav_semantic_search
python3 scripts/upgrade_stage5_v055_cloud_safe_config.py
cd ~/harp_sar_ws
catkin_make --force-cmake
source devel/setup.bash
```

该配置升级脚本会保留 `backend.api_key`、`backend.base_url` 和 `backend.model`。

## 预期状态

真实云端 API 延迟期间，Gazebo 不会暂停，PX4/MAVROS 不应再因 `/clock` 冻结而丢失 heartbeat。若本地 VLM 请求失败，`/vlm/sync_status` 将显示 `LOCAL_TIMEOUT_RESUMED:*` 或 `LOCAL_BACKEND_FAILURE_RESUMED:*`；此时机器人保持上一条有效任务而不是下发新的 fallback 航点。

后续待云端端点稳定后，可在 `vlm_semantic_search.yaml` 中逐个重新打开 `trigger_on_visual_novelty`、`trigger_on_map_free_space_expanded` 等自动触发器，并逐步降低 `min_trigger_interval_sec`。
