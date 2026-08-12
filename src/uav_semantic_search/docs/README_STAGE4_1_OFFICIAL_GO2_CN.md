# Stage-4.1：官方 Unitree Go2 外观模型与导航级异构探索

## 这次修复了什么

Stage-4.0 的 `go2_nav_proxy` 是手工搭建的几何代理，并非 Unitree 官方 Go2
的网格模型。本版本不再使用该代理。它在运行时加载官方 `go2_description`
包中的 Go2 URDF 与 DAE 网格，并将腿部关节冻结在 URDF 的参考站立构型。
因此：

- Gazebo 中显示的是官方 Go2 几何外形；
- 机体的尺寸、连杆外观与碰撞几何来自官方描述包；
- 仍然不模拟足端接触和步态；
- `/ugv0/cmd_vel` 仍由导航级运动学控制器转化为 Gazebo 位姿更新；
- 近地 LiDAR 与地面 FUEL-style frontier 探索逻辑保持不变。

## 先安装官方模型

完整工作空间安装后，在编译本包前执行：

```bash
cd ~/harp_sar_ws
bash src/uav_semantic_search/scripts/install_official_go2_description.sh ~/harp_sar_ws
catkin_make
source devel/setup.bash
rospack find go2_description
```

`rospack find` 必须返回 `~/harp_sar_ws/src/go2_description`。若机器无法访问
GitHub，请在另一台可以联网的机器完成下载后，将上游仓库中的
`robots/go2_description` 目录复制到当前工作空间的 `src/` 下。

## 启动

终端 1：

```bash
source /opt/ros/noetic/setup.bash
source ~/harp_sar_ws/devel/setup.bash
roslaunch uav_semantic_search heterogeneous_fuel_stage3.launch autostart:=true
```

终端 2：

```bash
source /opt/ros/noetic/setup.bash
source ~/harp_sar_ws/devel/setup.bash
rosrun uav_semantic_search run_px4_two_uav.py
```

## Go2 存在性检查

```bash
rostopic echo -n 1 /ugv0/model_ready
rostopic echo -n 1 /ugv0/model_spawn_status
rostopic echo -n 1 /ugv0/global_pose
rosservice call /gazebo/get_model_state "model_name: 'go2_0'
relative_entity_name: 'world'"
```

正常序列：

```text
SPAWNING_OFFICIAL_GO2
SPAWN_REQUEST_ACCEPTED
READY
```

若长期显示 `MISSING_GO2_DESCRIPTION`，表示官方描述包没有安装或没有重新
`source ~/harp_sar_ws/devel/setup.bash`。

## 关于原有 GetModelState 报错

旧版 spawner 用 `/gazebo/get_model_state` 查询 `go2_0` 是否存在。模型尚未
生成时，Gazebo 会打印 `model [go2_0] does not exist`。本版不再用该服务做
存在性轮询，而改用 `/gazebo/model_states` 确认模型出现，因此该噪声报错不再
出现。
