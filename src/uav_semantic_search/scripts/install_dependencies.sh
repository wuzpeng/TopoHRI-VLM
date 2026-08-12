#!/usr/bin/env bash
set -euo pipefail
sudo apt update
sudo apt install -y \
  python3-opencv \
  python3-yaml \
  ros-noetic-cv-bridge \
  ros-noetic-image-transport \
  ros-noetic-vision-opencv \
  ros-noetic-rviz \
  ros-noetic-rqt-image-view \
  ros-noetic-gazebo-plugins \
  ros-noetic-gazebo-ros-pkgs \
  ros-noetic-velodyne-gazebo-plugins

echo "Stage-2 dependencies installed. Re-source /opt/ros/noetic/setup.bash afterwards."
