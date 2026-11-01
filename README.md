# Sapphire ROS 2

ROS 2 Humble interface for the ROS-independent `sapphire` LiDAR-inertial
odometry core.

The wrapper owns ROS message conversion, parameters, launch integration, TF,
and artifact output. Algorithms remain in the sibling `sapphire` package.

## Interface

See [docs/INTERFACE_CONTRACT.md](docs/INTERFACE_CONTRACT.md) for the behavior
shared with `sapphire_ros1`.

The node uses a two-thread `MultiThreadedExecutor`. LiDAR and IMU callbacks are
in separate mutually-exclusive callback groups because LiDAR processing may
wait for scan-end IMU coverage. PGO visualization shares the LiDAR lane so it
cannot starve the IMU callback.

## Build

```bash
colcon build --symlink-install --packages-select sapphire_ros2
source install/setup.bash
```

## Run

```bash
ros2 launch sapphire_ros2 sapphire.launch.py \
  config_file:=/workspace/src/sapphire/cfg/sapphire_mid360.toml
```

Common overrides:

```bash
ros2 launch sapphire_ros2 sapphire.launch.py \
  config_file:=/workspace/src/sapphire/cfg/sapphire_mid360.toml \
  lidar_topic:=/velodyne_points \
  imu_topic:=/imu_raw \
  pose_frame:=imu \
  lidar_frame:=velodyne \
  imu_frame:=imu_link \
  trajectory_output:=/tmp/trajectory.tum
```

For a ROS 2 bag:

```bash
ros2 launch sapphire_ros2 sapphire.launch.py \
  config_file:=/workspace/src/sapphire/cfg/sapphire_mid360.toml \
  use_sim_time:=true \
  lidar_topic:=/points \
  imu_topic:=/imu
```

## Tests

```bash
colcon test --packages-select sapphire_ros2
colcon test-result --verbose
```
