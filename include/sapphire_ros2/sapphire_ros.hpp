#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <sapphire/odometry/pipeline.hpp>
#include <memory>

namespace sapphire_ros2 {

/// Sapphire ROS2 wrapper — LiDAR + IMU → odometry
/// Handles ROS data I/O + message conversion. Core algorithms in sapphire::OdometryPipeline.
class SapphireRos : public rclcpp::Node {
public:
    SapphireRos();

private:
    // ── Callbacks ─────────────────────────────────────────────────
    void lidar_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);

    // ── Helpers ───────────────────────────────────────────────────
    void publish_odometry(const sapphire::OdometryResult& result);

    // ── Publisher ────────────────────────────────────────────────
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

    // ── Subscriptions ────────────────────────────────────────────
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

    // ── Core ─────────────────────────────────────────────────────
    std::unique_ptr<sapphire::OdometryPipeline> pipeline_;
};

}  // namespace sapphire_ros2
