# 人机交互 VLM 搜索实验指标

本版本只在现有 `human_ai_vlm_stage5.launch` 中增加被动指标记录，不改变
Gazebo、PX4、Frontier 提取、中央/局部 VLM、HRI 界面或机器人执行链。
原来的三个终端和启动顺序保持不变。

## 1. 启动

首次替换工程后编译：

```bash
cd ~/harp_sar_ws
catkin_make
source devel/setup.bash
```

随后仍按原顺序启动。例如 H4：

```bash
# 终端1
source ~/harp_sar_ws/devel/setup.bash
roslaunch uav_semantic_search human_ai_vlm_stage5.launch \
  experiment_config:=$(rospack find uav_semantic_search)/config/experiments/h4.yaml
```

```bash
# 终端2
source ~/harp_sar_ws/devel/setup.bash
rosrun uav_semantic_search run_px4_two_uav.sh
```

```bash
# 终端3
source ~/harp_sar_ws/devel/setup.bash
rosrun uav_semantic_search frontier_extractor.py
```

指标节点已经在终端1的 launch 中默认启用，不需要第四个终端。

如需为本次试验指定名称，可在终端1命令末尾增加：

```bash
run_name:=h4_trial_01
```

不指定时，目录名自动采用 `地图编号_时间戳`。

## 2. 指标的严格定义

1. **成功率（%）**
   - 单次试验：当前 query 的目标候选状态为 `CONFIRMED`，或目标置信度达到
     `confirmed_target_completion_confidence`（当前默认 0.80），记为成功。
   - 多次试验：成功次数除以总试验次数，再乘 100%。
   - 只接受与当前 `query_version` 一致的证据，旧查询的目标不会误判成功。

2. **任务完成时间（s）**
   - 从首次检测到机器人有效运动开始。
   - 只累计“至少一台机器人正在运动”的时间并集。
   - `/vlm/epoch_active=true` 的 Local/Central VLM 查询阶段不计入。
   - 多台机器人同时运动时只累计一次，不把三台机器人的运动时间相加。

3. **团队路线长度（m）**
   - 三台机器人的实际累计轨迹长度之和。
   - UAV 使用三维距离，UGV 使用平面距离。
   - 该指标反映真实运动消耗，因此即使机器人在 VLM epoch 期间仍发生运动，
     对应轨迹仍计入路线长度。

4. **搜索到目标时的探索地图覆盖率（%）**
   - 在首次成功确认目标的时刻固定取值。
   - 已探索区域为 `/global_map_2d` 与 `/ugv0/ground_map_2d` 中已知栅格
     （值不为 `-1`）的并集。
   - 覆盖率 = 已探索栅格数 / 地图全部栅格数 × 100%。
   - 面积同时按 `栅格数 × resolution²` 输出，地图总面积指配置的完整矩形
   栅格面积，不是仅可通行自由空间面积。

5. **区域分配冲突率（RCR，%）**
   - 由同一 `epoch_id` 的 `/vlm/central_plan` 候选目录与
     `/vlm/validated_plan` 最终分配联合计算。
   - 仅统计最终均执行 `EXPLORE`、均具有有效拓扑区域标签、且候选目录中
     存在无冲突区域组合的机器人对。
   - 人类优先区域或人类直接指定的候选不进入统计。
   - `RCR = 同一区域的冲突机器人对数 / 有效自主探索机器人对数 × 100%`。
   - 若一次实验没有有效机器人对，则该次 RCR 记为 `null`，不人为记为0。

6. **约束违反率（CVR，%）**
   - 统计验证器处理之前，中央 VLM 原始分配中的约束违反。
   - `CVR = 被验证器拒绝的原始分配数 / 原始分配总数 × 100%`。
   - 拒绝原因（未知机器人/候选、机器人不兼容、重复分配、过期目标验证、
     状态优先级违反、HRI区域容量、拓扑区域冲突和不可达等）按类别记录。
   - API超时、网络错误、JSON错误以及完全无输出不会作为约束违反；没有
     原始分配的实验，其CVR记为 `null`。

## 3. 单次试验结果

默认保存位置：

```text
~/harp_sar_ws/experiment_results/<run_name>/
```

其中：

```text
summary.json       六项最终指标、计数分母和每台机器人路线长度
summary.csv        便于直接导入 Excel 的单行结果
time_series.csv    0.1 s 周期的审计数据
decision_metrics.csv 逐决策周期的RCR/CVR分子、分母和违反原因
```

成功确认目标时会自动写入最终结果。若试验失败或需要提前结束，可先调用：

```bash
rosservice call /experiment_metrics/finish
```

也可以直接在终端1按 `Ctrl+C`；节点关闭时仍会写入失败试验结果。

## 4. 多次试验汇总

完成多次试验后执行：

```bash
rosrun uav_semantic_search analyze_search_metrics.py \
  ~/harp_sar_ws/experiment_results
```

生成：

```text
aggregate_metrics.csv
aggregate_metrics.json
```

汇总文件分别按地图和全部试验给出成功率；任务完成时间、团队路线长度和
成功时覆盖率只对成功试验计算均值与样本标准差。RCR与CVR先在每次实验
内部计算，再对具有有效分母的实验报告均值与样本标准差；同时保留合并事件
计数，仅用于审计和描述。失败试验仍计入成功率分母。

## 5. 可选参数

如需临时关闭指标记录：

```bash
record_metrics:=false
```

如需更改输出目录：

```bash
metrics_dir:=/home/fangyc/harp_sar_ws/experiment_results
```

默认运动判定速度阈值为 `0.03 m/s`，累计路线时接受的相邻采样位移范围为
`0.002--1.0 m`，用于过滤静止抖动和定位跳变。
