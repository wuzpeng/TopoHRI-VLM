# Stage-4 v0.4.6: Official Go2 RGB-D camera fix

## Root cause corrected

The active Go2 is spawned by `scripts/go2_nav_spawner.py` from the official
`go2_description/urdf/go2_description.urdf` using `/gazebo/spawn_urdf_model`.
It does not use `models/go2_nav_proxy/model.sdf`. Earlier RGB-D additions to the
proxy SDF therefore never entered the real Gazebo model.

This delta injects a Gazebo Classic `type=depth` sensor and
`libgazebo_ros_openni_kinect.so` plugin onto the official URDF's existing
`front_camera` link. LiDAR injection remains unchanged.

## Expected topics

```text
/ugv0/front_camera/rgb/image_raw
/ugv0/front_camera/depth/image_raw
/ugv0/front_camera/rgb/camera_info
/ugv0/front_camera/depth/camera_info
/ugv0/front_camera/depth/points
```

The UGV semantic detector from v0.4.5 reads the first three topics through
`config/go2_nav.yaml`; no changes to `semantic_detector.py` or
`target_fusion_node.py` are required in this delta.

## Install

This is a delta over the current official-Go2 + v0.4.5 workspace. Copy the
`uav_semantic_search/` directory contents over the package root, then rebuild:

```bash
cd ~/harp_sar_ws
catkin_make --pkg uav_semantic_search --force-cmake
source devel/setup.bash
```

Completely stop Gazebo, PX4, MAVROS and ROS launch processes before restarting.
The sensor is created only when Go2 is spawned into a fresh Gazebo world.

## Verify

```bash
rostopic info /ugv0/front_camera/rgb/image_raw
rostopic hz /ugv0/front_camera/rgb/image_raw
rostopic hz /ugv0/front_camera/depth/image_raw
rostopic echo -n 1 /ugv0/front_camera/rgb/camera_info
gz model -m go2_0 -i | grep -nE 'front_camera|front_rgbd|type: "depth"'
```

Both RGB and depth topics should have a Gazebo publisher and update at roughly
15 Hz.