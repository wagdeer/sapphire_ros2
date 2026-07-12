#include <sapphire_ros2/sapphire_ros.hpp>

#include <sapphire/config.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <pcl_conversions/pcl_conversions.h>

#include <chrono>
#include <cmath>
#include <cstring>
#include <iomanip>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>

namespace sapphire_ros2 {

SapphireRos::SapphireRos()
    : Node("sapphire_ros")
{
    RCLCPP_INFO(this->get_logger(), "[sapphire_ros2] node starting");

    acc_scale_ = this->declare_parameter<double>("acc_scale", 1.0);
    if (!std::isfinite(acc_scale_) || acc_scale_ <= 0.0) {
        throw std::invalid_argument(
            "acc_scale must be finite and greater than zero");
    }
    const std::string default_config_file =
        ament_index_cpp::get_package_share_directory("sapphire_ros2")
        + "/cfg/sapphire_mid360.toml";
    config_file_ = this->declare_parameter<std::string>(
        "config_file", default_config_file);
    if (config_file_.empty()) {
        throw std::invalid_argument("config_file must not be empty");
    }
    world_frame_ =
        this->declare_parameter<std::string>("world_frame", "map");
    lidar_frame_ =
        this->declare_parameter<std::string>("lidar_frame", "lidar");
    if (world_frame_.empty() || lidar_frame_.empty()) {
        throw std::invalid_argument(
            "world_frame and lidar_frame must not be empty");
    }

    const sapphire::Config config = sapphire::loadConfig(config_file_);
    RCLCPP_INFO(
        this->get_logger(),
        "[sapphire_ros2] loaded algorithm config: %s",
        config_file_.c_str());
    pipeline_ = std::make_unique<sapphire::OdometryPipeline>(config);
    tf_broadcaster_ =
        std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    auto sensor_qos = rclcpp::SensorDataQoS();
    auto deskewed_qos = rclcpp::SensorDataQoS();
    deskewed_qos.keep_last(1);
    auto reliable_qos = rclcpp::QoS(10).reliable();

    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
        "sapphire/odometry", reliable_qos);

    deskewed_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "sapphire/deskewed", deskewed_qos);
    diagnostics_pub_ =
        this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
            "sapphire/diagnostics", reliable_qos);
    if (config.pgo.enabled) {
        auto visualization_qos =
            rclcpp::QoS(1).reliable().transient_local();
        pgo_map_pub_ =
            this->create_publisher<sensor_msgs::msg::PointCloud2>(
                "sapphire/pgo/map", visualization_qos);
        pgo_graph_pub_ =
            this->create_publisher<visualization_msgs::msg::MarkerArray>(
                "sapphire/pgo/graph", visualization_qos);
        pgo_visualization_timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            [this]() { publish_pgo_visualization(); });
    }

    // Match DLIO's execution model: LiDAR may wait for scan-end IMU data,
    // therefore IMU callbacks must be able to run concurrently.
    lidar_cb_group_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    imu_cb_group_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);

    rclcpp::SubscriptionOptions lidar_options;
    lidar_options.callback_group = lidar_cb_group_;
    lidar_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "lidar/points", sensor_qos,
        [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
            lidar_callback(msg);
        },
        lidar_options);

    rclcpp::SubscriptionOptions imu_options;
    imu_options.callback_group = imu_cb_group_;
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "imu/data", sensor_qos,
        [this](const sensor_msgs::msg::Imu::SharedPtr msg) {
            imu_callback(msg);
        },
        imu_options);

    RCLCPP_INFO(this->get_logger(), "Sapphire ROS2 wrapper ready");
    RCLCPP_INFO(this->get_logger(), "  Subscribed: lidar/points, imu/data");
    RCLCPP_INFO(
        this->get_logger(),
        "  Publishing: sapphire/odometry, sapphire/deskewed, "
        "sapphire/diagnostics");
    if (config.pgo.enabled) {
        RCLCPP_INFO(
            this->get_logger(),
            "  PGO visualization: sapphire/pgo/map, sapphire/pgo/graph "
            "(subscriber-driven)");
    }
    RCLCPP_INFO(this->get_logger(), "  IMU acceleration scale: %.6f", acc_scale_);
    RCLCPP_INFO(
        this->get_logger(), "  Frames: %s -> %s",
        world_frame_.c_str(), lidar_frame_.c_str());
}

namespace {

int fieldOffset(const sensor_msgs::msg::PointCloud2& msg, const char* name) {
    for (const auto& f : msg.fields) {
        if (f.name == name) return static_cast<int>(f.offset);
    }
    return -1;
}

double normalizePointTimestamp(
    double raw,
    const std::string& field_name,
    int datatype,
    double scan_stamp)
{
    // Livox offset_time and Ouster t are relative nanoseconds.
    if (field_name == "offset_time"
        || (field_name == "t" && datatype == 6)) {
        return raw * 1e-9;
    }

    // Some Livox PointCloud2 recordings store an absolute Unix timestamp in
    // nanoseconds as FLOAT64. Convert it to seconds relative to the header.
    if (std::abs(raw) > 1e14) {
        return raw * 1e-9 - scan_stamp;
    }

    // Absolute timestamps already expressed in seconds.
    if (std::abs(raw) > 1e6) {
        return raw - scan_stamp;
    }

    // Velodyne/Ouster "time" fields are commonly relative seconds.
    return raw;
}

template <typename T>
diagnostic_msgs::msg::KeyValue diagnosticValue(
    const std::string& key, const T& value)
{
    std::ostringstream stream;
    stream << std::setprecision(8) << value;
    diagnostic_msgs::msg::KeyValue output;
    output.key = key;
    output.value = stream.str();
    return output;
}

std::string vectorString(const Eigen::Vector3d& value) {
    std::ostringstream stream;
    stream << std::setprecision(8)
           << value.x() << "," << value.y() << "," << value.z();
    return stream.str();
}

}  // namespace

void SapphireRos::lidar_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    const size_t n = msg->width * msg->height;
    if (n == 0) return;

    // One-shot: dump all field names on first message
    {
        static bool once = true;
        if (once) {
            once = false;
            std::string fields;
            for (const auto& f : msg->fields) {
                fields += f.name + "(" + std::to_string(f.datatype) + ") ";
            }
            RCLCPP_INFO(this->get_logger(),
                "PointCloud2 fields: [%s] point_step=%u n=%zu",
                fields.c_str(), msg->point_step, n);
        }
    }

    int ox = fieldOffset(*msg, "x");
    int oy = fieldOffset(*msg, "y");
    int oz = fieldOffset(*msg, "z");
    int oi = fieldOffset(*msg, "intensity");

    if (ox < 0 || oy < 0 || oz < 0) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
            "PointCloud2 missing x/y/z fields, skipping");
        return;
    }

    // Auto-detect timestamp field
    int ots = -1;
    int ots_dtype = 0;
    std::string timestamp_field;
    const char* tnames[] = {"offset_time", "t", "timestamp", "time"};
    for (const char* nm : tnames) {
        ots = fieldOffset(*msg, nm);
        if (ots >= 0) {
            timestamp_field = nm;
            // Get datatype for correct read width
            for (const auto& f : msg->fields) {
                if (f.name == nm) { ots_dtype = f.datatype; break; }
            }
            break;
        }
    }

    auto cloud = std::make_shared<sapphire::PointCloud>();
    cloud->points.reserve(n);

    const double stamp = static_cast<double>(msg->header.stamp.sec)
                       + static_cast<double>(msg->header.stamp.nanosec) * 1e-9;
    const uint8_t* data = msg->data.data();
    const uint32_t step = msg->point_step;

    for (size_t i = 0; i < n; ++i) {
        const uint8_t* raw = data + i * step;

        sapphire::Point p;
        std::memcpy(&p.x, raw + ox, sizeof(float));
        std::memcpy(&p.y, raw + oy, sizeof(float));
        std::memcpy(&p.z, raw + oz, sizeof(float));
        p.data[3] = 1.0f;

        p.intensity = (oi >= 0) ? *reinterpret_cast<const float*>(raw + oi) : 0.0f;

        p.timestamp = 0.0;
        if (ots >= 0) {
            // sensor_msgs datatypes: 6=UINT32, 7=FLOAT32, 8=FLOAT64
            double raw_timestamp = 0.0;
            if (ots_dtype == 6) {
                uint32_t value;
                std::memcpy(&value, raw + ots, sizeof(value));
                raw_timestamp = static_cast<double>(value);
            } else if (ots_dtype == 8) {
                std::memcpy(
                    &raw_timestamp, raw + ots, sizeof(raw_timestamp));
            } else {
                float value;
                std::memcpy(&value, raw + ots, sizeof(value));
                raw_timestamp = static_cast<double>(value);
            }
            p.timestamp = normalizePointTimestamp(
                raw_timestamp, timestamp_field, ots_dtype, stamp);
        }

        cloud->points.push_back(p);
    }

    cloud->width = cloud->points.size();
    cloud->height = 1;
    cloud->is_dense = false;

    pipeline_->pushLidar(stamp, cloud);

    const auto result = pipeline_->latestResult();
    if (last_lidar_result_stamp_.has_value()
        && result.stamp == *last_lidar_result_stamp_) {
        return;
    }
    last_lidar_result_stamp_ = result.stamp;
    publish_diagnostics(result);
    if (result.converged) {
        const auto propagated = pipeline_->latestPropagatedResult();
        publish_odometry(propagated.value_or(result));
        publish_deskewed(result.stamp);
    }
}

void SapphireRos::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    sapphire::ImuData imu;
    imu.stamp = static_cast<double>(msg->header.stamp.sec)
              + static_cast<double>(msg->header.stamp.nanosec) * 1e-9;
    imu.accel = acc_scale_ * Eigen::Vector3d(
        msg->linear_acceleration.x,
        msg->linear_acceleration.y,
        msg->linear_acceleration.z);
    imu.gyro = {msg->angular_velocity.x,
                msg->angular_velocity.y,
                msg->angular_velocity.z};

    pipeline_->pushImu(imu);
    if (const auto propagated = pipeline_->latestPropagatedResult()) {
        publish_odometry(*propagated);
    }
}

void SapphireRos::publish_odometry(const sapphire::OdometryResult& result) {
    const rclcpp::Time stamp(
        static_cast<int64_t>(std::llround(result.stamp * 1e9)));
    auto msg = nav_msgs::msg::Odometry();
    msg.header.stamp = stamp;
    msg.header.frame_id = world_frame_;
    msg.child_frame_id = lidar_frame_;

    const auto& T = result.T_world_lidar;
    msg.pose.pose.position.x = T.translation().x();
    msg.pose.pose.position.y = T.translation().y();
    msg.pose.pose.position.z = T.translation().z();
    Eigen::Quaterniond q(T.rotation());
    msg.pose.pose.orientation.x = q.x();
    msg.pose.pose.orientation.y = q.y();
    msg.pose.pose.orientation.z = q.z();
    msg.pose.pose.orientation.w = q.w();

    const Eigen::Vector3d velocity_lidar =
        T.rotation().transpose() * result.v_world;
    msg.twist.twist.linear.x = velocity_lidar.x();
    msg.twist.twist.linear.y = velocity_lidar.y();
    msg.twist.twist.linear.z = velocity_lidar.z();

    odom_pub_->publish(msg);

    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = stamp;
    transform.header.frame_id = world_frame_;
    transform.child_frame_id = lidar_frame_;
    transform.transform.translation.x = T.translation().x();
    transform.transform.translation.y = T.translation().y();
    transform.transform.translation.z = T.translation().z();
    transform.transform.rotation = msg.pose.pose.orientation;
    tf_broadcaster_->sendTransform(transform);
}

void SapphireRos::publish_diagnostics(
    const sapphire::OdometryResult& result)
{
    if (diagnostics_pub_->get_subscription_count() == 0) {
        return;
    }

    diagnostic_msgs::msg::DiagnosticArray message;
    message.header.stamp = rclcpp::Time(
        static_cast<int64_t>(std::llround(result.stamp * 1e9)));

    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "sapphire/odometry";
    status.hardware_id = "sapphire";
    status.level = result.converged
        ? diagnostic_msgs::msg::DiagnosticStatus::OK
        : diagnostic_msgs::msg::DiagnosticStatus::WARN;
    status.message = result.diagnostics.registration_accepted
        ? "scan registration accepted"
        : (result.converged
            ? "odometry initialized"
            : "scan registration rejected");

    const auto& diagnostics = result.diagnostics;
    const sapphire::PoseGraphStats pose_graph = pipeline_->poseGraphStats();
    status.values = {
        diagnosticValue("registration_accepted",
                        diagnostics.registration_accepted),
        diagnosticValue("preprocess_ms", diagnostics.preprocess_ms),
        diagnosticValue("downsample_ms", diagnostics.downsample_ms),
        diagnosticValue(
            "submap_rebuild_ms", diagnostics.submap_rebuild_ms),
        diagnosticValue(
            "deskew_timeline_ms", diagnostics.deskew_timeline_ms),
        diagnosticValue(
            "deskew_integration_ms", diagnostics.deskew_integration_ms),
        diagnosticValue(
            "deskew_transform_ms", diagnostics.deskew_transform_ms),
        diagnosticValue("deskew_total_ms", diagnostics.deskew_total_ms),
        diagnosticValue(
            "deskew_timestamp_groups",
            diagnostics.deskew_timestamp_groups),
        diagnosticValue(
            "deskew_imu_intervals", diagnostics.deskew_imu_intervals),
        diagnosticValue(
            "deskew_pim_copies", diagnostics.deskew_pim_copies),
        diagnosticValue("registration_ms", diagnostics.registration_ms),
        diagnosticValue("fitness_score", diagnostics.fitness_score),
        diagnosticValue("num_inliers", diagnostics.num_inliers),
        diagnosticValue("iterations", diagnostics.iterations),
        diagnosticValue("source_points", diagnostics.source_points),
        diagnosticValue("target_points", diagnostics.target_points),
        diagnosticValue("keyframe_count", diagnostics.keyframe_count),
        diagnosticValue(
            "stored_keyframe_points",
            diagnostics.stored_keyframe_points),
        diagnosticValue("velocity_world_mps", vectorString(result.v_world)),
        diagnosticValue(
            "accel_bias_mps2", vectorString(diagnostics.accel_bias)),
        diagnosticValue(
            "gyro_bias_radps", vectorString(diagnostics.gyro_bias)),
        diagnosticValue("pgo_keyframes", pose_graph.keyframes),
        diagnosticValue("pgo_optimized_poses", pose_graph.optimized_poses),
        diagnosticValue(
            "pgo_loop_candidates", pose_graph.loop_candidates),
        diagnosticValue(
            "pgo_loop_registration_failures",
            pose_graph.loop_registration_failures),
        diagnosticValue("pgo_loop_edges", pose_graph.loop_edges),
        diagnosticValue(
            "pgo_last_loop_fitness", pose_graph.last_loop_fitness),
        diagnosticValue(
            "pgo_correction_translation_m",
            pose_graph.correction_translation),
        diagnosticValue(
            "pgo_correction_rotation_rad",
            pose_graph.correction_rotation),
        diagnosticValue("pgo_revision", pose_graph.revision),
    };
    message.status.push_back(std::move(status));
    diagnostics_pub_->publish(message);
}

void SapphireRos::publish_pgo_visualization() {
    const rclcpp::Time stamp = this->now();

    if (pgo_graph_pub_->get_subscription_count() > 0) {
        pipeline_->requestPoseGraphSnapshot();
        const sapphire::PoseGraphSnapshot snapshot =
            pipeline_->poseGraphSnapshot();
        if (!snapshot.optimized_poses.empty()) {
            visualization_msgs::msg::Marker odometry_edges;
            odometry_edges.header.stamp = stamp;
            odometry_edges.header.frame_id = world_frame_;
            odometry_edges.ns = "odometry_edges";
            odometry_edges.id = 0;
            odometry_edges.type =
                visualization_msgs::msg::Marker::LINE_LIST;
            odometry_edges.action =
                visualization_msgs::msg::Marker::ADD;
            odometry_edges.pose.orientation.w = 1.0;
            odometry_edges.scale.x = 0.05;
            odometry_edges.color.r = 0.1f;
            odometry_edges.color.g = 0.8f;
            odometry_edges.color.b = 0.2f;
            odometry_edges.color.a = 1.0f;
            odometry_edges.points.reserve(
                (snapshot.optimized_poses.size() - 1) * 2);

            auto graphPoint = [](const sapphire::Isometry3d& pose) {
                geometry_msgs::msg::Point point;
                point.x = pose.translation().x();
                point.y = pose.translation().y();
                point.z = pose.translation().z();
                return point;
            };
            for (size_t i = 1; i < snapshot.optimized_poses.size(); ++i) {
                odometry_edges.points.push_back(
                    graphPoint(snapshot.optimized_poses[i - 1]));
                odometry_edges.points.push_back(
                    graphPoint(snapshot.optimized_poses[i]));
            }

            visualization_msgs::msg::Marker loop_edges;
            loop_edges.header = odometry_edges.header;
            loop_edges.ns = "loop_edges";
            loop_edges.id = 0;
            loop_edges.type = visualization_msgs::msg::Marker::LINE_LIST;
            loop_edges.action = visualization_msgs::msg::Marker::ADD;
            loop_edges.pose.orientation.w = 1.0;
            loop_edges.scale.x = 0.12;
            loop_edges.color.r = 1.0f;
            loop_edges.color.g = 0.1f;
            loop_edges.color.b = 0.1f;
            loop_edges.color.a = 1.0f;
            loop_edges.points.reserve(snapshot.loop_edges.size() * 2);
            for (const sapphire::PoseGraphEdge& edge : snapshot.loop_edges) {
                if (edge.source >= snapshot.optimized_poses.size()
                    || edge.target >= snapshot.optimized_poses.size()) {
                    continue;
                }
                loop_edges.points.push_back(
                    graphPoint(snapshot.optimized_poses[edge.source]));
                loop_edges.points.push_back(
                    graphPoint(snapshot.optimized_poses[edge.target]));
            }

            visualization_msgs::msg::Marker nodes;
            nodes.header = odometry_edges.header;
            nodes.ns = "nodes";
            nodes.id = 0;
            nodes.type = visualization_msgs::msg::Marker::SPHERE_LIST;
            nodes.action = visualization_msgs::msg::Marker::ADD;
            nodes.pose.orientation.w = 1.0;
            nodes.scale.x = 0.15;
            nodes.scale.y = 0.15;
            nodes.scale.z = 0.15;
            nodes.color.r = 0.1f;
            nodes.color.g = 0.4f;
            nodes.color.b = 1.0f;
            nodes.color.a = 1.0f;
            nodes.points.reserve(snapshot.optimized_poses.size());
            for (const sapphire::Isometry3d& pose :
                 snapshot.optimized_poses) {
                nodes.points.push_back(graphPoint(pose));
            }

            visualization_msgs::msg::MarkerArray graph;
            graph.markers.push_back(std::move(odometry_edges));
            graph.markers.push_back(std::move(loop_edges));
            graph.markers.push_back(std::move(nodes));
            pgo_graph_pub_->publish(graph);
        }
    }

    if (pgo_map_pub_->get_subscription_count() > 0) {
        pipeline_->requestPoseGraphMap();
        const sapphire::PointCloudConstPtr map =
            pipeline_->latestPoseGraphMap();
        if (map && !map->empty()) {
            sensor_msgs::msg::PointCloud2 message;
            pcl::toROSMsg(*map, message);
            message.header.stamp = stamp;
            message.header.frame_id = world_frame_;
            pgo_map_pub_->publish(message);
        }
    }
}

void SapphireRos::publish_deskewed(double stamp) {
    if (deskewed_pub_->get_subscription_count() == 0) return;

    auto cloud = pipeline_->latestDeskewed();
    if (!cloud || cloud->points.empty()) return;

    sensor_msgs::msg::PointCloud2 ros_msg;
    pcl::toROSMsg(*cloud, ros_msg);

    ros_msg.header.stamp = rclcpp::Time(
        static_cast<int64_t>(std::llround(stamp * 1e9)));
    ros_msg.header.frame_id = world_frame_;

    deskewed_pub_->publish(ros_msg);
}

}  // namespace sapphire_ros2
