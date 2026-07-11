#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <sapphire/odometry/pipeline.hpp>
#include <memory>
#include <optional>
#include <string>

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
    void publish_deskewed(double stamp);
    void publish_diagnostics(const sapphire::OdometryResult& result);

    // ── Publishers ───────────────────────────────────────────────
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr deskewed_pub_;
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
        diagnostics_pub_;

    // ── Subscriptions ────────────────────────────────────────────
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::CallbackGroup::SharedPtr lidar_cb_group_;
    rclcpp::CallbackGroup::SharedPtr imu_cb_group_;

    // ── Core ─────────────────────────────────────────────────────
    double acc_scale_ = 1.0;  // raw acceleration unit → m/s²
    std::string config_file_;
    std::string world_frame_ = "map";
    std::string lidar_frame_ = "lidar";
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::unique_ptr<sapphire::OdometryPipeline> pipeline_;
    std::optional<double> last_lidar_result_stamp_;
};

}  // namespace sapphire_ros2
