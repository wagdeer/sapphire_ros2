#include <sapphire_ros2/conversion.hpp>

#include <sensor_msgs/msg/point_field.hpp>

#include <cmath>
#include <cstring>
#include <limits>
#include <memory>
#include <stdexcept>
#include <vector>

namespace sapphire_ros2 {
namespace {

constexpr uint32_t kPointStep = 24;
constexpr uint32_t kOffsetX = 0;
constexpr uint32_t kOffsetY = 4;
constexpr uint32_t kOffsetZ = 8;
constexpr uint32_t kOffsetIntensity = 12;
constexpr uint32_t kOffsetTimestamp = 16;

sensor_msgs::msg::PointField make_field(
    const std::string& name, uint32_t offset, uint8_t datatype)
{
    sensor_msgs::msg::PointField field;
    field.name = name;
    field.offset = offset;
    field.datatype = datatype;
    field.count = 1;
    return field;
}

template <typename T>
void append_scalar(std::vector<uint8_t>& bytes, const T& value) {
    const auto* source = reinterpret_cast<const uint8_t*>(&value);
    bytes.insert(bytes.end(), source, source + sizeof(T));
}

}  // namespace

double stamp_to_seconds(const builtin_interfaces::msg::Time& stamp) {
    return static_cast<double>(stamp.sec)
        + static_cast<double>(stamp.nanosec) * 1e-9;
}

builtin_interfaces::msg::Time stamp_from_seconds(double stamp) {
    if (!std::isfinite(stamp) || stamp < 0.0) {
        throw std::invalid_argument(
            "ROS 2 timestamps must be finite and non-negative");
    }
    constexpr double kNanosecondsPerSecond = 1e9;
    constexpr double kMaximumRos2Time =
        static_cast<double>(std::numeric_limits<int32_t>::max()) + 1.0;
    if (stamp >= kMaximumRos2Time) {
        throw std::out_of_range(
            "timestamp exceeds ROS 2 builtin_interfaces/Time range");
    }
    const double whole_seconds = std::floor(stamp);
    int32_t seconds = static_cast<int32_t>(whole_seconds);
    uint32_t nanoseconds = static_cast<uint32_t>(std::llround(
        (stamp - whole_seconds) * kNanosecondsPerSecond));
    if (nanoseconds == 1000000000u) {
        ++seconds;
        nanoseconds = 0;
    }
    builtin_interfaces::msg::Time result;
    result.sec = seconds;
    result.nanosec = nanoseconds;
    return result;
}

sapphire::sensor::CloudView make_cloud_view(
    const sensor_msgs::msg::PointCloud2& message)
{
    sapphire::sensor::CloudView view;
    view.fields.reserve(message.fields.size());
    for (const sensor_msgs::msg::PointField& field : message.fields) {
        view.fields.push_back(
            {field.name, field.offset, field.datatype, field.count});
    }
    view.data = message.data.empty() ? nullptr : message.data.data();
    view.data_size = message.data.size();
    view.width = message.width;
    view.point_count =
        static_cast<size_t>(message.width) * static_cast<size_t>(message.height);
    view.point_step = message.point_step;
    view.row_step = message.row_step;
    view.is_bigendian = message.is_bigendian;
    view.header_stamp = stamp_to_seconds(message.header.stamp);
    return view;
}

sapphire::ImuData to_core_imu(const sensor_msgs::msg::Imu& message) {
    sapphire::ImuData imu;
    imu.stamp = stamp_to_seconds(message.header.stamp);
    imu.accel = Eigen::Vector3d(
        message.linear_acceleration.x,
        message.linear_acceleration.y,
        message.linear_acceleration.z);
    imu.gyro = Eigen::Vector3d(
        message.angular_velocity.x,
        message.angular_velocity.y,
        message.angular_velocity.z);
    return imu;
}

sapphire::PointCloudPtr to_core_cloud(
    const sapphire::sensor::IngestResult& ingested,
    double time_offset_s)
{
    if (!std::isfinite(time_offset_s)) {
        throw std::invalid_argument("lidar time offset must be finite");
    }
    auto cloud = std::make_shared<sapphire::PointCloud>();
    cloud->points.reserve(ingested.points.size());
    for (const sapphire::sensor::CanonicalPoint& source : ingested.points) {
        sapphire::Point point;
        point.x = source.x;
        point.y = source.y;
        point.z = source.z;
        point.data[3] = 1.0f;
        point.intensity = source.intensity;
        point.timestamp = source.stamp + time_offset_s;
        cloud->points.push_back(point);
    }
    cloud->width = static_cast<uint32_t>(cloud->points.size());
    cloud->height = 1;
    cloud->is_dense = false;
    return cloud;
}

geometry_msgs::msg::Pose to_ros_pose(
    const sapphire::Isometry3d& transform)
{
    geometry_msgs::msg::Pose pose;
    const Eigen::Vector3d translation = transform.translation();
    pose.position.x = translation.x();
    pose.position.y = translation.y();
    pose.position.z = translation.z();

    Eigen::Quaterniond rotation(transform.rotation());
    rotation.normalize();
    pose.orientation.x = rotation.x();
    pose.orientation.y = rotation.y();
    pose.orientation.z = rotation.z();
    pose.orientation.w = rotation.w();
    return pose;
}

nav_msgs::msg::Odometry to_ros_odometry(
    const sapphire::OdometryResult& result,
    const std::string& world_frame,
    const std::string& child_frame,
    const sapphire::Isometry3d& T_child_lidar)
{
    const sapphire::Isometry3d T_world_child =
        result.T_world_lidar * T_child_lidar;
    nav_msgs::msg::Odometry message;
    message.header.stamp = stamp_from_seconds(result.stamp);
    message.header.frame_id = world_frame;
    message.child_frame_id = child_frame;
    message.pose.pose = to_ros_pose(T_world_child);

    const Eigen::Vector3d velocity_child =
        T_world_child.rotation().transpose() * result.v_world;
    message.twist.twist.linear.x = velocity_child.x();
    message.twist.twist.linear.y = velocity_child.y();
    message.twist.twist.linear.z = velocity_child.z();
    return message;
}

sensor_msgs::msg::PointCloud2 to_ros_point_cloud(
    const sapphire::PointCloud& cloud,
    const builtin_interfaces::msg::Time& stamp,
    const std::string& frame_id)
{
    constexpr size_t kMaximumPoints =
        std::numeric_limits<uint32_t>::max() / kPointStep;
    if (cloud.points.size() > kMaximumPoints) {
        throw std::overflow_error(
            "point cloud exceeds ROS PointCloud2 row_step range");
    }

    sensor_msgs::msg::PointCloud2 message;
    message.header.stamp = stamp;
    message.header.frame_id = frame_id;
    message.height = 1;
    message.width = static_cast<uint32_t>(cloud.points.size());
    message.fields.reserve(5);
    message.fields.push_back(
        make_field("x", kOffsetX, sensor_msgs::msg::PointField::FLOAT32));
    message.fields.push_back(
        make_field("y", kOffsetY, sensor_msgs::msg::PointField::FLOAT32));
    message.fields.push_back(
        make_field("z", kOffsetZ, sensor_msgs::msg::PointField::FLOAT32));
    message.fields.push_back(make_field(
        "intensity", kOffsetIntensity,
        sensor_msgs::msg::PointField::FLOAT32));
    message.fields.push_back(make_field(
        "timestamp", kOffsetTimestamp,
        sensor_msgs::msg::PointField::FLOAT64));
    message.is_bigendian = false;
    message.point_step = kPointStep;
    message.row_step = message.width * kPointStep;
    message.is_dense = cloud.is_dense;
    message.data.reserve(
        static_cast<size_t>(message.row_step) * message.height);

    for (const sapphire::Point& point : cloud.points) {
        append_scalar(message.data, point.x);
        append_scalar(message.data, point.y);
        append_scalar(message.data, point.z);
        append_scalar(message.data, point.intensity);
        append_scalar(message.data, point.timestamp);
    }
    return message;
}

const char* datatype_label(uint8_t datatype) {
    switch (datatype) {
        case sensor_msgs::msg::PointField::INT8: return "I8";
        case sensor_msgs::msg::PointField::UINT8: return "U8";
        case sensor_msgs::msg::PointField::INT16: return "I16";
        case sensor_msgs::msg::PointField::UINT16: return "U16";
        case sensor_msgs::msg::PointField::INT32: return "I32";
        case sensor_msgs::msg::PointField::UINT32: return "U32";
        case sensor_msgs::msg::PointField::FLOAT32: return "F32";
        case sensor_msgs::msg::PointField::FLOAT64: return "F64";
        default: return "?";
    }
}

}  // namespace sapphire_ros2
