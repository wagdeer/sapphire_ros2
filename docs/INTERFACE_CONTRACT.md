# Sapphire ROS Interface Contract

The ROS 1 and ROS 2 wrappers expose the same behavior. ROS-specific
mechanisms differ, but parameters, frame semantics, timestamp rules, and
artifact outputs do not.

## Parameters

| Parameter | Type | Default | Meaning |
|---|---|---|---|
| `config_file` | string | required | Absolute path to a core-owned algorithm TOML |
| `world_frame` | string | `map` | Parent frame for poses and map-frame clouds |
| `lidar_frame` | string | `lidar` | LiDAR frame name |
| `imu_frame` | string | `imu` | IMU frame name |
| `pose_frame` | string | `lidar` | `lidar` or `imu`; applies to odometry, TF, path, and TUM |
| `lidar_time_offset` | double | `0.0` | Added to the scan header and every point timestamp |
| `imu_time_offset` | double | `0.0` | Added to every IMU timestamp |
| `trajectory_output` | string | empty | TUM path override; empty uses the TOML value |

For output poses:

```text
T_world_pose = T_world_lidar * T_pose_lidar
```

When `pose_frame=imu`, `T_pose_lidar` is the inverse of the configured
`T_imu_lidar`. Odometry, TF, path, and TUM output must all use the same result.

## Topics

| Direction | Topic | Type |
|---|---|---|
| Input | `/front_lidar` | `sensor_msgs/PointCloud2` |
| Input | `/front_lidar/imu` | `sensor_msgs/Imu` |
| Output | `/sapphire/odometry` | `nav_msgs/Odometry` |
| Output | `/sapphire/deskewed` | `sensor_msgs/PointCloud2` |
| Output | `/sapphire/trajectory` | `nav_msgs/Path` |
| Output | `/sapphire/pgo/map` | `sensor_msgs/PointCloud2` |
| Output | `/sapphire/pgo/graph` | `visualization_msgs/MarkerArray` |

Every external topic is configurable in launch.

## Delivery semantics

ROS 1 uses queue sizes and latching. ROS 2 maps them to QoS:

| Stream | ROS 1 | ROS 2 |
|---|---|---|
| LiDAR | queue 1 | BEST_EFFORT, KEEP_LAST 1 |
| IMU | queue 100 | BEST_EFFORT, KEEP_LAST 100 |
| Odometry | queue 10 | RELIABLE, KEEP_LAST 10 |
| Deskewed cloud | queue 1 | BEST_EFFORT, KEEP_LAST 1 |
| Trajectory | queue 1, latched | RELIABLE, TRANSIENT_LOCAL, KEEP_LAST 1 |
| PGO map/graph | queue 1, latched | RELIABLE, TRANSIENT_LOCAL, KEEP_LAST 1 |

LiDAR and IMU must use separate processing lanes so LiDAR can wait for
scan-end IMU coverage without blocking the IMU callback.

## Shutdown and artifacts

The final TUM trajectory is written once on normal spin exit. Writes are
idempotent. A malformed input frame is logged and dropped; startup/configuration
errors remain fatal.
