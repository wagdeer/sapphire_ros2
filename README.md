# Sapphire ROS 2

ROS 2 Humble interface for [Sapphire](https://github.com/wagdeer/sapphire).

Put the core and this package in the same workspace. The core is found as the
sibling directory `../sapphire`.

```text
ros2_ws/src/
├── sapphire/          https://github.com/wagdeer/sapphire
└── sapphire_ros2/
```

Topic names, frames, and QoS are in
[docs/INTERFACE_CONTRACT.md](docs/INTERFACE_CONTRACT.md). Dataset replay is in
the core repository:
[DATASET_REPLAY.md](https://github.com/wagdeer/sapphire/blob/master/docs/DATASET_REPLAY.md).

## Demos

Recordings below are RViz sessions from the core repository.

**S3E**

![S3E RViz mapping](https://github.com/wagdeer/sapphire/raw/main/docs/sapphire.gif)

**HILTI 2022**

![HILTI 2022 construction stairs RViz mapping](https://github.com/wagdeer/sapphire/raw/main/docs/hilti-2022.gif)

**HILTI 2023**

![HILTI 2023 RViz mapping](https://github.com/wagdeer/sapphire/raw/main/docs/hilti-2023.gif)

**Newer College 2020**

![Newer College RViz mapping](https://github.com/wagdeer/sapphire/raw/main/docs/newer_college.gif)

## Docker

From this repository:

```bash
docker build -f docker/Dockerfile -t sapphire:humble .
```

The image is Ubuntu 22.04 with ROS Humble. Eigen, PCL, and spdlog come from Ubuntu. GTSAM comes from `ppa:borglab/gtsam-develop` (the 4.3 development series; 4.3a0 when this file was written). The image also builds small_gicp 1.0.1. toml++ is not installed in the image; CMake downloads 3.4.0 while configuring the core.

Mount the workspace and build inside it:

```bash
docker run --rm -it -v ~/ros2_ws:/workspace sapphire:humble
cd /workspace
colcon build --symlink-install --packages-select sapphire_ros2
source install/setup.bash
```

## Without Docker

Install Humble and the packages that come from apt:

```bash
sudo add-apt-repository ppa:borglab/gtsam-develop
sudo apt install build-essential cmake git pkg-config \
  libboost-all-dev libeigen3-dev libflann-dev libgtsam-dev \
  libomp-dev libpcl-dev libspdlog-dev \
  python3-colcon-common-extensions \
  ros-humble-pcl-conversions ros-humble-rosbag2 \
  ros-humble-rviz2 ros-humble-tf2-ros
```

`libgtsam-dev` from that PPA is the GTSAM 4.3 development series (4.3a0 at the time this image was written). The PPA can move, so record the installed version if you need to reproduce a result. small_gicp is not packaged for Ubuntu 22.04. Build it from source and install it where CMake can find it:

| Library | Version |
|---|---|
| [small_gicp](https://github.com/koide3/small_gicp) | 1.0.1 |

toml++ 3.4.0 is downloaded by CMake when it is not already installed.

Then build the workspace:

```bash
source /opt/ros/humble/setup.bash
cd ~/ros2_ws
colcon build --symlink-install --packages-select sapphire_ros2
source install/setup.bash
```

## Run

`config_file` is required. The topic arguments below override the launch
defaults `/front_lidar` and `/front_lidar/imu`.

```bash
ros2 launch sapphire_ros2 sapphire.launch.py \
  config_file:=/path/to/sapphire/cfg/sapphire_mid360.toml \
  lidar_topic:=/your_lidar \
  imu_topic:=/your_imu \
  rviz:=true
```

The node always publishes `/sapphire/odometry` and `/sapphire/deskewed`.
`/sapphire/trajectory`, `/sapphire/pgo/map`, and `/sapphire/pgo/graph` are
published only when the profile sets `pgo.enabled` to true.

To play a bag, add `use_sim_time:=true` to the launch command, then in another terminal:

```bash
ros2 bag play your_bag --clock
```

Stop the launch with Ctrl-C so the TUM file can be written. A killed process
does not flush it. `trajectory_output:=/path/to/trajectory.tum` overrides the
path in the TOML profile.
