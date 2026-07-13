#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <sapphire/odometry/pipeline.hpp>
#include <sapphire/projection/depth_image.hpp>
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
    void publish_pgo_visualization();
    void publish_virtual_depth();
    sapphire::Isometry3d virtual_depth_pose(
        const sapphire::DeskewFrameSnapshot& snapshot);

    // ── Publishers ───────────────────────────────────────────────
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr deskewed_pub_;
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
        diagnostics_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pgo_map_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
        pgo_graph_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr
        depth_camera_info_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr
        depth_confidence_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_debug_pub_;
    rclcpp::TimerBase::SharedPtr pgo_visualization_timer_;

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
    std::string virtual_camera_frame_ = "virtual_camera_optical_frame";
    bool virtual_depth_enabled_ = true;
    bool depth_debug_enabled_ = false;
    std::string depth_orientation_mode_ = "startup_calibrated";
    double depth_yaw_offset_rad_ = 0.0;
    sapphire::projection::DepthImageConfig depth_config_;
    sapphire::Isometry3d T_imu_lidar_ = sapphire::Isometry3d::Identity();
    std::optional<Eigen::Matrix3d> R_lidar_virtual_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::unique_ptr<sapphire::OdometryPipeline> pipeline_;
    std::optional<double> last_lidar_result_stamp_;
    std::optional<double> last_virtual_depth_stamp_;
};

}  // namespace sapphire_ros2
