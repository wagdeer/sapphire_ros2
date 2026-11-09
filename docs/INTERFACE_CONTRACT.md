# Interface

`sapphire_ros1` and `sapphire_ros2` take the same parameters and publish the same topics. Queue settings differ because ROS 1 has no QoS.

## Parameters

| Parameter | Default | Meaning |
|---|---|---|
| `config_file` | required | Absolute path to a TOML profile in the core `cfg/` directory |
| `world_frame` | `map` | Parent frame for poses and map-frame clouds |
| `lidar_frame` | `lidar` | LiDAR frame name |
| `imu_frame` | `imu` | IMU frame name |
| `pose_frame` | `lidar` | `lidar` or `imu`. Odometry, TF, the path, and the TUM file all use this frame |
| `lidar_time_offset` | `0.0` | Seconds added to the scan header and to every point time |
| `imu_time_offset` | `0.0` | Seconds added to every IMU stamp |
| `trajectory_output` | empty | TUM path. Empty uses `output.trajectory_dir` from the profile |

Launch arguments override the topic names. The node itself subscribes to `lidar/points` and `imu/data`, and the launch file remaps those names.

## Topics

| Direction | Default name | Type |
|---|---|---|
| Input | `/front_lidar` | `sensor_msgs/PointCloud2` |
| Input | `/front_lidar/imu` | `sensor_msgs/Imu` |
| Output | `/sapphire/odometry` | `nav_msgs/Odometry` |
| Output | `/sapphire/deskewed` | `sensor_msgs/PointCloud2` |
| Output | `/sapphire/trajectory` | `nav_msgs/Path` |
| Output | `/sapphire/pgo/map` | `sensor_msgs/PointCloud2` |
| Output | `/sapphire/pgo/graph` | `visualization_msgs/MarkerArray` |

Odometry and the deskewed cloud are always advertised. The trajectory, PGO map, and PGO graph exist only when the profile sets `pgo.enabled` to true.

## Queues

| Stream | ROS 1 | ROS 2 |
|---|---|---|
| LiDAR | queue 1 | best effort, keep last 1 |
| IMU | queue 100 | best effort, keep last 100 |
| Odometry | queue 10 | reliable, keep last 10 |
| Deskewed cloud | queue 1 | best effort, keep last 1 |
| Trajectory | queue 1, latched | reliable, transient local, keep last 1 |
| PGO map and graph | queue 1, latched | reliable, transient local, keep last 1 |

LiDAR and IMU are processed on separate threads. A scan is dropped when IMU samples covering the end of that scan do not arrive in time.

## Trajectory file

The TUM file is the pose-graph trajectory. It is written once, when the node shuts down normally. A profile that sets `output.trajectory_dir` while `pgo.enabled` is false is rejected at load. `trajectory_output` only changes the path. With PGO disabled, shutdown creates no file.

A malformed cloud or IMU message is logged and dropped. A bad `config_file` or an unknown parameter value stops the node at startup.
