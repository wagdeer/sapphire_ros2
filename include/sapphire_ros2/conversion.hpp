#pragma once

#include <sapphire/sensor/cloud_ingest.hpp>
#include <sapphire/types.hpp>

#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <cstddef>
#include <cstdint>
#include <string>

namespace sapphire_ros2 {

/// Convert a ROS 2 timestamp to seconds without discarding nanoseconds.
double stamp_to_seconds(const builtin_interfaces::msg::Time& stamp);

/// Convert absolute seconds to a ROS 2 timestamp.
/// Throws std::invalid_argument for unsupported values.
builtin_interfaces::msg::Time stamp_from_seconds(double stamp);

/// Build the core's zero-copy view over a ROS PointCloud2 payload.
sapphire::sensor::CloudView make_cloud_view(
    const sensor_msgs::msg::PointCloud2& message);

/// Convert ROS IMU fields to the core IMU type.
sapphire::ImuData to_core_imu(const sensor_msgs::msg::Imu& message);

/// Materialize core points from a successful cloud-ingest result.
sapphire::PointCloudPtr to_core_cloud(
    const sapphire::sensor::IngestResult& ingested,
    double time_offset_s = 0.0);

/// Convert a core isometry to a normalized ROS pose.
geometry_msgs::msg::Pose to_ros_pose(
    const sapphire::Isometry3d& transform);

/// Convert an odometry result with an explicit output frame.
nav_msgs::msg::Odometry to_ros_odometry(
    const sapphire::OdometryResult& result,
    const std::string& world_frame,
    const std::string& child_frame,
    const sapphire::Isometry3d& T_child_lidar =
        sapphire::Isometry3d::Identity());

/// Serialize the Sapphire point cloud directly to a self-describing
/// PointCloud2 message. The layout is x/y/z/intensity/timestamp.
sensor_msgs::msg::PointCloud2 to_ros_point_cloud(
    const sapphire::PointCloud& cloud,
    const builtin_interfaces::msg::Time& stamp,
    const std::string& frame_id);

const char* datatype_label(uint8_t datatype);

}  // namespace sapphire_ros2
