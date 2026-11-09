# Sapphire ROS 2

ROS 2 Humble interface for [Sapphire](https://github.com/wagdeer/sapphire).

Put this package next to the core. The build uses `../sapphire` directly.

```text
ros2_ws/src/
├── sapphire/
└── sapphire_ros2/
```

[sapphire_ros1](https://github.com/wagdeer/sapphire_ros1) is the ROS 1 Noetic package. Topic names, frames, and QoS match that package and are listed in [docs/INTERFACE_CONTRACT.md](docs/INTERFACE_CONTRACT.md).

## Docker

Use the image. It already has Humble and the libraries.

```bash
docker build -f docker/Dockerfile -t sapphire:humble .
docker run --rm -it -v ~/ros2_ws:/workspace sapphire:humble
```

`~/ros2_ws/src` needs both `sapphire` and `sapphire_ros2`. Inside the container:

```bash
source /opt/ros/humble/setup.bash
cd /workspace
colcon build --symlink-install --packages-select sapphire_ros2
source install/setup.bash
```

## Without Docker

| Library | Version | Ubuntu 22.04 |
|---|---|---|
| Eigen | 3.4 | `libeigen3-dev` |
| PCL | 1.12 or newer | `libpcl-dev` |
| OpenMP | | `libomp-dev` |
| spdlog | | `libspdlog-dev` |
| GTSAM | 4.2 | `ppa:borglab/gtsam-release-4.2` |
| small_gicp | 1.0.1 | see below |

toml++ 3.4 is downloaded by CMake when it is not already installed. Message and TF packages are listed in `package.xml`.

```bash
sudo apt install build-essential cmake git software-properties-common \
  libeigen3-dev libpcl-dev libomp-dev libspdlog-dev
sudo add-apt-repository -y ppa:borglab/gtsam-release-4.2
sudo apt install -y libgtsam-dev
```

small_gicp 1.0.1 is built in [docker/Dockerfile](docker/Dockerfile).

```bash
source /opt/ros/humble/setup.bash
cd ~/ros2_ws
colcon build --symlink-install --packages-select sapphire_ros2
source install/setup.bash
```

## Run

`config_file` is required. The topic arguments override `/front_lidar` and `/front_lidar/imu`.

```bash
ros2 launch sapphire_ros2 sapphire.launch.py \
  config_file:=/path/to/sapphire/cfg/mid360.toml \
  lidar_topic:=/your_lidar \
  imu_topic:=/your_imu \
  rviz:=true
```

The node always publishes `/sapphire/odometry` and `/sapphire/deskewed`. It publishes `/sapphire/trajectory`, `/sapphire/pgo/map`, and `/sapphire/pgo/graph` when the profile sets `pgo.enabled` to true.

## Play a bag

```bash
ros2 launch sapphire_ros2 sapphire.launch.py \
  config_file:=/path/to/sapphire/cfg/mid360.toml \
  lidar_topic:=/your_lidar \
  imu_topic:=/your_imu \
  use_sim_time:=true \
  trajectory_output:=/path/to/trajectory.tum
```

In another terminal:

```bash
ros2 bag play your_bag --clock
```

Stop the launch with Ctrl-C after the bag ends. That writes `trajectory.tum`. `kill -9` does not. `trajectory_output` replaces the path stored in the TOML profile. Pass it: several profiles point at `/workspace/results/...`.

Dataset profiles and bag conversion are in [DATASET_REPLAY.md](https://github.com/wagdeer/sapphire/blob/master/docs/DATASET_REPLAY.md). ROS 2 has no separate HILTI launch file. That page lists the topics, frames, and time offset for each rig.

## License

MIT.
