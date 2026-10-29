#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <sapphire/odometry/pipeline.hpp>
#include <sapphire/sensor/cloud_ingest.hpp>
#include <memory>
#include <mutex>
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
    void publish_pgo_visualization();
    /// Replace the whole lidar-rate path with the latest per-keyframe replay.
    void publish_trajectory();

    // ── Publishers ───────────────────────────────────────────────
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr trajectory_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr deskewed_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pgo_map_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
        pgo_graph_pub_;
    rclcpp::TimerBase::SharedPtr pgo_visualization_timer_;

    // ── Subscriptions ────────────────────────────────────────────
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::CallbackGroup::SharedPtr lidar_cb_group_;
    rclcpp::CallbackGroup::SharedPtr imu_cb_group_;

    // ── Core ─────────────────────────────────────────────────────
    sapphire::sensor::IngestConfig lidar_ingest_;
    bool require_point_time_ = false;
    bool lidar_time_locked_ = false;
    double locked_lidar_span_s_ = 0.0;
    bool warned_lidar_span_ratio_ = false;
    bool warned_lidar_header_mismatch_ = false;
    std::string config_file_;
    std::string world_frame_ = "map";
    std::string lidar_frame_ = "lidar";
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::unique_ptr<sapphire::OdometryPipeline> pipeline_;
    std::optional<double> last_lidar_result_stamp_;
    std::mutex trajectory_publish_mutex_;
    bool trajectory_published_ = false;
    size_t trajectory_samples_published_ = 0;
    size_t trajectory_revision_published_ = 0;
};

}  // namespace sapphire_ros2
