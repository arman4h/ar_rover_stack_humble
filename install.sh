#!/usr/bin/env bash
set -euo pipefail
sudo apt-get update
sudo apt-get install -y python3-colcon-common-extensions python3-rosdep python3-numpy ros-humble-xacro ros-humble-robot-state-publisher ros-humble-tf2-sensor-msgs ros-humble-nav2-costmap-2d ros-humble-nav2-bringup
sudo rosdep init 2>/dev/null || true
rosdep update
