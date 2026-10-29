#include <sapphire_ros2/sapphire_ros.hpp>

#include <sapphire/config.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <pcl_conversions/pcl_conversions.h>

#include <chrono>
#include <memory>
#include <stdexcept>
#include <string>

namespace sapphire_ros2 {

SapphireRos::SapphireRos()
    : Node("sapphire_ros")
{
    RCLCPP_INFO(this->get_logger(), "[sapphire_ros2] node starting");

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
    lidar_ingest_.time_field = config.sensor.lidar.time_field;
    lidar_ingest_.time_unit = config.sensor.lidar.time_unit;
    lidar_ingest_.time_ref = config.sensor.lidar.time_ref;
    lidar_ingest_.max_scan_duration = config.sensor.lidar.max_scan_duration;
    lidar_ingest_.header_tolerance = config.sensor.lidar.header_tolerance;
    require_point_time_ = config.sensor.lidar.require_point_time;
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
    if (config.pgo.enabled) {
        auto trajectory_qos =
            rclcpp::QoS(1).reliable().transient_local();
        trajectory_pub_ = this->create_publisher<nav_msgs::msg::Path>(
            "sapphire/trajectory", trajectory_qos);
    }

    deskewed_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "sapphire/deskewed", deskewed_qos);
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

    RCLCPP_INFO(this->get_logger(), "[sapphire_ros2] ready");
}

namespace {

const char* datatypeLabel(uint8_t datatype) {
    switch (datatype) {
        case 1: return "I8";
        case 2: return "U8";
        case 3: return "I16";
        case 4: return "U16";
        case 5: return "I32";
        case 6: return "U32";
        case 7: return "F32";
        case 8: return "F64";
        default: return "?";
    }
}

sapphire::sensor::CloudView makeView(const sensor_msgs::msg::PointCloud2& msg) {
    sapphire::sensor::CloudView view;
    view.fields.reserve(msg.fields.size());
    for (const auto& field : msg.fields) {
        view.fields.push_back(
            {field.name, field.offset, field.datatype, field.count});
    }
    view.data = msg.data.empty() ? nullptr : msg.data.data();
    view.data_size = msg.data.size();
    view.width = msg.width;
    view.point_count = static_cast<size_t>(msg.width) * msg.height;
    view.point_step = msg.point_step;
    view.row_step = msg.row_step;
    view.is_bigendian = msg.is_bigendian;
    view.header_stamp = static_cast<double>(msg.header.stamp.sec)
        + static_cast<double>(msg.header.stamp.nanosec) * 1e-9;
    return view;
}

}  // namespace

void SapphireRos::lidar_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    if (msg->width == 0 || msg->height == 0) {
        return;
    }

    const sapphire::sensor::CloudView view = makeView(*msg);
    sapphire::sensor::IngestConfig ingest_cfg = lidar_ingest_;
    const auto ingested = sapphire::sensor::ingest(view, ingest_cfg);
    const double stamp = view.header_stamp;

    if (ingested.points.empty()) {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            5000,
            "[sensor] dropping lidar frame: %s",
            ingested.time.note.empty() ? "no points" : ingested.time.note.c_str());
        return;
    }
    if (!ingested.time.ok && require_point_time_) {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            5000,
            "[sensor] dropping lidar frame: %s",
            ingested.time.note.c_str());
        return;
    }

    if (ingested.time.ok && !lidar_time_locked_) {
        lidar_ingest_.time_field = ingested.time.field;
        lidar_ingest_.time_unit = sapphire::sensor::unitName(ingested.time.unit);
        lidar_ingest_.time_ref = ingested.time.ref == sapphire::sensor::TimeRef::Absolute
            ? "absolute"
            : "relative";
        locked_lidar_span_s_ = ingested.time.span_s;
        lidar_time_locked_ = true;

        const char* dtype = "?";
        for (const auto& field : view.fields) {
            if (field.name == ingested.time.field) {
                dtype = datatypeLabel(field.datatype);
                break;
            }
        }
        RCLCPP_INFO(
            this->get_logger(),
            "[sensor] lidar field=%s(%s) unit=%s ref=%s span=%.6fs "
            "header_offset=%+.6fs kept=%zu dropped_time=%zu deskew=%s",
            ingested.time.field.c_str(),
            dtype,
            lidar_ingest_.time_unit.c_str(),
            lidar_ingest_.time_ref.c_str(),
            ingested.time.span_s,
            ingested.time.header_offset_s,
            ingested.points.size(),
            ingested.dropped_time,
            ingested.deskewable ? "on" : "off");
        RCLCPP_INFO(
            this->get_logger(),
            "[sensor] profile: time_field=\"%s\" time_unit=\"%s\" time_ref=\"%s\"",
            lidar_ingest_.time_field.c_str(),
            lidar_ingest_.time_unit.c_str(),
            lidar_ingest_.time_ref.c_str());
    } else if (
        ingested.time.ok && lidar_time_locked_ && locked_lidar_span_s_ > 0.0
        && !warned_lidar_span_ratio_) {
        const double ratio = ingested.time.span_s / locked_lidar_span_s_;
        if (ratio > 10.0 || ratio < 0.1) {
            warned_lidar_span_ratio_ = true;
            RCLCPP_WARN(
                this->get_logger(),
                "[sensor] lidar span changed by more than 10x under the locked "
                "unit: locked=%.6fs now=%.6fs",
                locked_lidar_span_s_,
                ingested.time.span_s);
        }
    }
    if (ingested.time.ok && ingested.time.header_mismatch
        && !warned_lidar_header_mismatch_) {
        warned_lidar_header_mismatch_ = true;
        RCLCPP_WARN(
            this->get_logger(),
            "[sensor] lidar header_mismatch: header_offset=%+.6fs; "
            "using point time as absolute seconds",
            ingested.time.header_offset_s);
    }

    auto cloud = std::make_shared<sapphire::PointCloud>();
    cloud->points.reserve(ingested.points.size());
    for (const auto& source : ingested.points) {
        sapphire::Point point;
        point.x = source.x;
        point.y = source.y;
        point.z = source.z;
        point.data[3] = 1.0f;
        point.intensity = source.intensity;
        point.timestamp = source.stamp;
        cloud->points.push_back(point);
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
    if (result.converged) {
        const auto propagated = pipeline_->latestPropagatedResult();
        publish_odometry(propagated.value_or(result));
        publish_deskewed(result.stamp);
        publish_trajectory();
    }
}

void SapphireRos::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    sapphire::ImuData imu;
    imu.stamp = static_cast<double>(msg->header.stamp.sec)
              + static_cast<double>(msg->header.stamp.nanosec) * 1e-9;
    imu.accel = Eigen::Vector3d(
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

void SapphireRos::publish_trajectory() {
    if (!trajectory_pub_) {
        return;
    }

    // Lidar callbacks and the visualization timer both publish. Keep the
    // read, the freshness check, and the send under one lock so an older
    // snapshot cannot replace a path that already includes a later loop.
    std::lock_guard<std::mutex> lock(trajectory_publish_mutex_);
    const sapphire::MapTrajectory trajectory = pipeline_->mapTrajectory();
    if (trajectory.samples.empty()) {
        return;
    }
    const bool newer_revision =
        trajectory.revision > trajectory_revision_published_;
    const bool more_samples =
        trajectory.revision == trajectory_revision_published_
        && trajectory.samples.size() > trajectory_samples_published_;
    if (trajectory_published_ && !newer_revision && !more_samples) {
        return;
    }

    nav_msgs::msg::Path path;
    path.header.frame_id = world_frame_;
    path.header.stamp = rclcpp::Time(static_cast<int64_t>(
        std::llround(trajectory.samples.back().stamp * 1e9)));
    path.poses.reserve(trajectory.samples.size());
    for (const sapphire::MapTrajectorySample& sample : trajectory.samples) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = world_frame_;
        pose.header.stamp = rclcpp::Time(static_cast<int64_t>(
            std::llround(sample.stamp * 1e9)));
        const auto& T = sample.T_map_lidar;
        pose.pose.position.x = T.translation().x();
        pose.pose.position.y = T.translation().y();
        pose.pose.position.z = T.translation().z();
        Eigen::Quaterniond rotation(T.rotation());
        rotation.normalize();
        pose.pose.orientation.x = rotation.x();
        pose.pose.orientation.y = rotation.y();
        pose.pose.orientation.z = rotation.z();
        pose.pose.orientation.w = rotation.w();
        path.poses.push_back(std::move(pose));
    }
    trajectory_pub_->publish(path);
    trajectory_published_ = true;
    trajectory_samples_published_ = trajectory.samples.size();
    trajectory_revision_published_ = trajectory.revision;
}

void SapphireRos::publish_pgo_visualization() {
    publish_trajectory();
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
