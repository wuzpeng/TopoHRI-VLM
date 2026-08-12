# Stage-5 v0.5.4：直接 API Key 配置

本增量包在 `config/vlm_semantic_search.yaml` 中新增：

```yaml
backend:
  mode: openai_compatible
  base_url: "https://api.example.com/v1"
  api_key: "在此粘贴你的 API Key"
  api_key_env: VLM_API_KEY
  model: "your-vision-language-model"
  allow_mock_fallback: false
```

## 使用规则

1. 当 `backend.api_key` 非空时，程序优先使用该值，不再需要设置 `VLM_API_KEY` 环境变量。
2. 当 `backend.api_key` 为空时，程序兼容此前版本，继续读取 `api_key_env` 指定的环境变量。
3. 真实 VLM 测试时建议设置 `allow_mock_fallback: false`，避免 API 调用失败后静默回退为 mock。
4. API Key 会保存在项目配置文件中；不要把该文件提交到 Git、打包共享或上传公开仓库。

## 修改后重启

本次仅修改 Python 和 YAML。关闭正在运行的 launch 后，执行：

```bash
cd ~/harp_sar_ws
source /opt/ros/noetic/setup.bash
catkin_make --force-cmake
source devel/setup.bash
roslaunch uav_semantic_search heterogeneous_vlm_stage5.launch autostart:=true
```

启动日志或 `/vlm/local_semantic_observation` 不再出现 `Mock local VLM report`，才说明真实视觉端点已被成功调用。
