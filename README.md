# Rover Stack — ROS 2 Humble

A clean, CPU-first ROS 2 workspace for a skid-steer rover using Livox MID-360S and ZED 2i.

## Important scope and honesty

This is a build-oriented baseline, not a claim of hardware commissioning. It includes:
- robot description with the supplied sensor mounting transforms;
- a PointCloud2-to-rolling-elevation/costmap mapper using TF at the cloud timestamp;
- a Nav2 costmap layer plugin consuming the generated OccupancyGrid;
- a velocity-command watchdog;
- bringup, diagnostics, validation scripts and configuration.

It does **not** include vendor sensor drivers, ZED calibration, a motor driver, a global SLAM backend, or a validated Xavier NX CUDA backend. Those are separate integrations. CPU mapping is the reference implementation. The GPU backend is deliberately not enabled until measured on the target JetPack image.

## Supported environment

ROS 2 Humble on Ubuntu 22.04 (recommended). On Jetson Xavier NX with JetPack 5.x, use a compatible Ubuntu 22.04 / ROS 2 Humble container. JetPack 5 host OS is Ubuntu 20.04; native Humble installation there is not the supported binary target. Verify your container actually reports `ROS_DISTRO=humble`.

## Build

```bash
cd ~/rover_stack_humble
source /opt/ros/humble/setup.bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --event-handlers console_direct+
source install/setup.bash
```

## Run in stages

1. Start Livox driver and verify `/livox/lidar` is `sensor_msgs/msg/PointCloud2`.
2. Start ZED wrapper with positional tracking enabled and verify its odometry and TF configuration. The required TF authority is `odom -> base_link`; do not also publish a static transform for this edge.
3. Start robot description:

```bash
ros2 launch rover_bringup description.launch.py
```

4. Start terrain mapping (only after the TF chain is valid):

```bash
ros2 launch rover_bringup full_system.launch.py
```

This starts the robot description, terrain mapper and command watchdog. It does not start the vendor sensor drivers or Nav2; start those separately after configuring their frame and topic parameters.

```bash
ros2 launch rover_bringup terrain.launch.py
```

5. Inspect:

```bash
ros2 topic hz /terrain/local_costmap
ros2 topic echo /terrain/local_costmap --once
ros2 run tf2_ros tf2_echo odom livox_frame
```

## TF ownership

`robot_state_publisher` owns `base_link -> livox_frame` and `base_link -> zed_camera_link` from URDF. ZED positional tracking or a dedicated localization adapter must own `odom -> base_link`. This workspace intentionally does not publish a fake static odometry transform.

The ZED wrapper's exact parameter names and base-frame behavior vary by wrapper version. Configure it so the published odometry pose represents `base_link`, or add a calibrated camera-to-base pose adapter. Do not simply rename a camera-frame odometry child frame without applying the rigid-body extrinsic.

## Mapper behavior

The mapper transforms incoming points into `odom` using TF at the message timestamp. It bins points into a rolling 2D grid, estimates ground height using the low percentile per cell, and derives a simple local traversability cost from height discontinuity. This is a baseline estimator, not a complete semantic ground classifier. It requires enough ground returns and correct sensor extrinsics.

Published topics:
- `/terrain/elevation_cloud` (`sensor_msgs/PointCloud2`): metric XYZ samples at observed elevation-grid cell centers.
- `/terrain/local_costmap` (`nav_msgs/OccupancyGrid`): 0 free, 1–99 increasing cost, 100 lethal, -1 unknown.

The costmap layer plugin subscribes to `/terrain/local_costmap`. Add it to a Nav2 `plugins` list only after building and testing the plugin. A local costmap also needs a valid obstacle source and a controller footprint matching the physical chassis. Remap Nav2 controller output `/cmd_vel` to `/cmd_vel_nav`; the watchdog owns the final `/cmd_vel` output. Do not allow Nav2 and the watchdog to publish to the motor command topic simultaneously.

## Validation and limitations

```bash
./scripts/validate_workspace.sh
./scripts/check_runtime.sh
```

The validation script checks package manifests, XML, YAML and Python syntax. A successful static validation is not a substitute for `colcon build`. This environment does not include ROS 2, so the archive has not been compiled against your exact Humble installation. Build on the target ROS environment and provide the first compiler output if anything fails.

Before driving: validate TF, timestamps, cloud rate, terrain output, Nav2 costmap alignment, command watchdog, physical E-stop and motor controller behavior. Keep the wheels off the ground for initial command tests.
