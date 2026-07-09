#include <sapphire_ros2/sapphire_ros.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <spdlog/spdlog.h>

namespace sapphire_ros2 {

SapphireRos::SapphireRos()
    : Node("sapphire_ros")
{
    spdlog::info("[sapphire_ros2] node starting");

    // Init pipeline with default config (TOML loading later)
    sapphire::Config config;
    pipeline_ = std::make_unique<sapphire::OdometryPipeline>(config);

    // QoS
    auto sensor_qos = rclcpp::SensorDataQoS();
    auto reliable_qos = rclcpp::QoS(10).reliable();

    // Publishers
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
        "sapphire/odometry", reliable_qos);

    // Subscribers
    lidar_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "lidar/points", sensor_qos,
        [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
            lidar_callback(msg);
        });

    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "imu/data", sensor_qos,
        [this](const sensor_msgs::msg::Imu::SharedPtr msg) {
            imu_callback(msg);
        });

    RCLCPP_INFO(this->get_logger(), "Sapphire ROS2 wrapper ready");
    RCLCPP_INFO(this->get_logger(), "  Subscribed: lidar/points, imu/data");
    RCLCPP_INFO(this->get_logger(), "  Publishing: sapphire/odometry");
}

// ── LiDAR callback ────────────────────────────────────────────────────

void SapphireRos::lidar_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    // ROS → PCL
    pcl::PointCloud<pcl::PointXYZI> pcl_cloud;
    pcl::fromROSMsg(*msg, pcl_cloud);

    // PCL → sapphire
    sapphire::PointCloud sapphire_cloud;
    sapphire_cloud.reserve(pcl_cloud.size());
    for (const auto& pt : pcl_cloud) {
        sapphire::Point3d p;
        p.position = {pt.x, pt.y, pt.z};
        p.intensity = pt.intensity;
        p.time_offset = 0.0;  // TODO: extract per-point timestamp from point field
        sapphire_cloud.push_back(p);
    }

    double stamp = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;

    pipeline_->pushLidar(stamp, sapphire_cloud);
    const auto& result = pipeline_->latestResult();
    if (result.converged) {
        publish_odometry(result);
    }
}

// ── IMU callback ──────────────────────────────────────────────────────

void SapphireRos::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    sapphire::ImuData imu;
    imu.stamp = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
    imu.accel = {msg->linear_acceleration.x,
                 msg->linear_acceleration.y,
                 msg->linear_acceleration.z};
    imu.gyro = {msg->angular_velocity.x,
                msg->angular_velocity.y,
                msg->angular_velocity.z};

    pipeline_->pushImu(imu);
}

// ── Publish odometry ──────────────────────────────────────────────────

void SapphireRos::publish_odometry(const sapphire::OdometryResult& result) {
    auto msg = nav_msgs::msg::Odometry();
    msg.header.stamp = rclcpp::Time(
        static_cast<int32_t>(result.stamp),
        static_cast<uint32_t>((result.stamp - static_cast<int32_t>(result.stamp)) * 1e9));
    msg.header.frame_id = "odom";
    msg.child_frame_id = "base_link";

    const auto& T = result.T_world_lidar;
    msg.pose.pose.position.x = T.translation().x();
    msg.pose.pose.position.y = T.translation().y();
    msg.pose.pose.position.z = T.translation().z();
    Eigen::Quaterniond q(T.rotation());
    msg.pose.pose.orientation.x = q.x();
    msg.pose.pose.orientation.y = q.y();
    msg.pose.pose.orientation.z = q.z();
    msg.pose.pose.orientation.w = q.w();

    odom_pub_->publish(msg);
}

}  // namespace sapphire_ros2
