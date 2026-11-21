#!/usr/bin/env bash
set -euo pipefail
source /opt/ros/humble/setup.bash
source "$(cd "$(dirname "$0")/.." && pwd)/install/setup.bash"
echo "ROS_DISTRO=$ROS_DISTRO"
ros2 topic list | grep -E '^/(livox/lidar|terrain/local_costmap|terrain/local_elevation)$' || true
ros2 run tf2_ros tf2_echo odom livox_frame
