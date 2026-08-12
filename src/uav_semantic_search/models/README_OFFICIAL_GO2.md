# Official Go2 assets

This project no longer contains a hand-built Go2 proxy model.  The actual Unitree
Go2 URDF and DAE meshes are installed separately as the `go2_description` ROS
package by running:

```bash
bash ~/harp_sar_ws/src/uav_semantic_search/scripts/install_official_go2_description.sh
```

The navigation-level spawner consumes that official package at runtime, freezes
articulated joints at their URDF reference pose, disables gravity, and attaches a
simulated LiDAR.  Planar movement remains a task-level abstraction.
