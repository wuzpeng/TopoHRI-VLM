# UAV 航向跟随与时间戳建图同步

本版本在四地图 TopoHRI/VLM 工程基础上同时启用两条配套链路：

1. `waypoint_executor.py` 按实际运动方向实时生成并平滑 UAV yaw；
2. `central_map_fuser.py` 按 `PointCloud2.header.stamp` 查询
   `map -> uavX/lidar_link`，再将点云变换到地图坐标系。

二者必须同时部署。只启用 yaw 控制而继续使用回调时的“最新位姿”融合点云，
会在 UAV 转向时形成放射状或扇形地图拖影。

## 部署

```bash
cd ~/harp_sar_ws
catkin_make
source devel/setup.bash

cd ~/harp_sar_ws/src/uav_semantic_search
python3 scripts/patch_px4_gazebo.py --px4-root "$PX4_ROOT"
```

`patch_px4_gazebo.py` 会更新已有传感器块，把两架 UAV 的点云 frame 分别设置为
`uav0/lidar_link` 和 `uav1/lidar_link`。执行后必须完全关闭并重新启动 Gazebo、
PX4 SITL、MAVROS 和工程 launch。

如果系统缺少运行依赖：

```bash
sudo apt update
sudo apt install ros-noetic-tf2-sensor-msgs
```

## 运行检查

```bash
rostopic echo -n 1 /uav0/lidar/points/header
rostopic echo -n 1 /uav1/lidar/points/header
rosrun tf tf_echo map uav0/lidar_link
rosrun tf tf_echo map uav1/lidar_link
```

两个点云 frame 应分别为 `uav0/lidar_link`、`uav1/lidar_link`，且两个 TF 查询均应
持续输出。同步统计可通过以下话题查看：

```bash
rostopic echo /mapping/uav_tf_sync_status
rostopic echo /mapping/ugv_tf_sync_status
```

启动最初少量点云因 TF 缓存尚未建立而被丢弃是正常现象；若
`dropped_tf` 持续快速增长，应优先检查点云 frame、`/clock` 和 TF 链，而不是调大
“最近位姿”容差或恢复旧式融合。
