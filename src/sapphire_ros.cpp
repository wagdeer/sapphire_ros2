#include <sapphire_ros2/sapphire_ros.hpp>

#include <sapphire/config.hpp>
#include <sapphire/io/tum.hpp>
#include <sapphire_ros2/conversion.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <chrono>
#include <cmath>
#include <exception>
#include <memory>
#include <stdexcept>
#include <string>

namespace sapphire_ros2 {

SapphireRos::SapphireRos()
    : Node("sapphire_ros")
{
    RCLCPP_INFO(this->get_logger(), "[sapphire_ros2] node starting");

    config_file_ = this->declare_parameter<std::string>(
        "config_file", "");
    if (config_file_.empty()) {
        throw std::invalid_argument("config_file must not be empty");
    }
    world_frame_ =
        this->declare_parameter<std::string>("world_frame", "map");
    lidar_frame_ =
        this->declare_parameter<std::string>("lidar_frame", "lidar");
    imu_frame_ =
        this->declare_parameter<std::string>("imu_frame", "imu");
    pose_frame_ =
        this->declare_parameter<std::string>("pose_frame", "lidar");
    lidar_time_offset_s_ =
        this->declare_parameter<double>("lidar_time_offset", 0.0);
    imu_time_offset_s_ =
        this->declare_parameter<double>("imu_time_offset", 0.0);
    if (
        world_frame_.empty() || lidar_frame_.empty() || imu_frame_.empty()) {
        throw std::invalid_argument(
            "world_frame, lidar_frame, and imu_frame must not be empty");
    }
    if (pose_frame_ != "lidar" && pose_frame_ != "imu") {
        throw std::invalid_argument(
            "pose_frame must be either 'lidar' or 'imu'");
    }
    if (
        !std::isfinite(lidar_time_offset_s_)
        || !std::isfinite(imu_time_offset_s_)) {
        throw std::invalid_argument("sensor time offsets must be finite");
    }

    const sapphire::Config config = sapphire::loadConfig(config_file_);
    trajectory_output_path_ = sapphire::trajectoryOutputPath(config);
    const std::string trajectory_output_override =
        this->declare_parameter<std::string>("trajectory_output", "");
    if (!trajectory_output_override.empty()) {
        trajectory_output_path_ = trajectory_output_override;
    }
    if (pose_frame_ == "imu") {
        T_pose_lidar_ = config.extrinsics.T_imu_lidar.inverse();
    }
    lidar_time_ = sapphire::makeLidarTimeSession(config.sensor.lidar);
    RCLCPP_INFO(
        this->get_logger(),
        "[sapphire_ros2] loaded algorithm config: %s",
        config_file_.c_str());
    if (trajectory_output_path_.empty()) {
        RCLCPP_INFO(
            this->get_logger(),
            "[sapphire_ros2] TUM trajectory dump is off "
            "(output.trajectory_dir is empty)");
    } else {
        RCLCPP_INFO(
            this->get_logger(),
            "[sapphire_ros2] TUM trajectory will be saved on exit: %s",
            trajectory_output_path_.c_str());
    }
    pipeline_ = std::make_unique<sapphire::OdometryPipeline>(config);
    tf_broadcaster_ =
        std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    lidar_cb_group_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    imu_cb_group_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);

    auto lidar_qos = rclcpp::SensorDataQoS();
    lidar_qos.keep_last(1);
    auto imu_qos = rclcpp::SensorDataQoS();
    // Cover the core's 500 ms scan-end IMU wait without forcing LiDAR
    // callbacks to retain stale point clouds.
    imu_qos.keep_last(100);
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
            [this]() { pgo_visualization_callback(); },
            lidar_cb_group_);
    }

    // Match DLIO's execution model: LiDAR may wait for scan-end IMU data,
    // therefore IMU callbacks must be able to run concurrently.
    rclcpp::SubscriptionOptions lidar_options;
    lidar_options.callback_group = lidar_cb_group_;
    lidar_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "lidar/points", lidar_qos,
        [this](const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
            lidar_callback(msg);
        },
        lidar_options);

    rclcpp::SubscriptionOptions imu_options;
    imu_options.callback_group = imu_cb_group_;
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "imu/data", imu_qos,
        [this](const sensor_msgs::msg::Imu::ConstSharedPtr msg) {
            imu_callback(msg);
        },
        imu_options);

    RCLCPP_INFO(this->get_logger(), "[sapphire_ros2] ready");
}

SapphireRos::~SapphireRos() {
    (void)saveTrajectory();
}

bool SapphireRos::saveTrajectory() {
    if (trajectory_saved_.exchange(true)) {
        return true;
    }
    if (trajectory_output_path_.empty() || !pipeline_) {
        return true;
    }

    sapphire::MapTrajectory trajectory = pipeline_->mapTrajectory();
    for (sapphire::MapTrajectorySample& sample : trajectory.samples) {
        sample.T_map_lidar = sample.T_map_lidar * T_pose_lidar_;
    }
    const sapphire::TrajectorySaveReport report = sapphire::saveMapTrajectory(
        trajectory_output_path_, trajectory);
    switch (report.status) {
    case sapphire::TrajectorySaveStatus::NotConfigured:
        return true;
    case sapphire::TrajectorySaveStatus::Empty:
        RCLCPP_WARN(
            this->get_logger(),
            "[sapphire_ros2] no TUM trajectory written to %s",
            trajectory_output_path_.c_str());
        return true;
    case sapphire::TrajectorySaveStatus::Written:
        RCLCPP_INFO(
            this->get_logger(),
            "[sapphire_ros2] wrote %zu TUM poses to %s",
            report.result.written,
            trajectory_output_path_.c_str());
        if (report.result.skipped > 0) {
            RCLCPP_WARN(
                this->get_logger(),
                "[sapphire_ros2] skipped %zu trajectory poses with "
                "non-finite values or non-increasing timestamps",
                report.result.skipped);
        }
        return true;
    case sapphire::TrajectorySaveStatus::IoFailure:
        RCLCPP_ERROR(
            this->get_logger(),
            "[sapphire_ros2] failed to write TUM trajectory: %s",
            report.error.c_str());
        return false;
    }
    return false;
}

void SapphireRos::lidar_callback(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
{
    try {
        if (msg->width == 0 || msg->height == 0) {
            return;
        }

        const sapphire::sensor::CloudView view =
            make_cloud_view(*msg);
        const auto ingested = lidar_time_.ingest(view);
        const double stamp = view.header_stamp;

        if (!std::isfinite(stamp)) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                5000,
                "[sensor] dropping lidar frame with non-finite stamp");
            return;
        }
        const sapphire::sensor::LidarTimeSession::Commit time_commit =
            lidar_time_.commit(ingested);
        if (!time_commit.accept) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                5000,
                "[sensor] dropping lidar frame: %s",
                time_commit.drop_reason.c_str());
            return;
        }

        if (time_commit.just_locked) {
            const sapphire::sensor::FieldDescriptor* time_field =
                sapphire::sensor::findField(view, ingested.time.field);
            RCLCPP_INFO(
                this->get_logger(),
                "[sensor] lidar field=%s(%s) unit=%s ref=%s span=%.6fs "
                "header_offset=%+.6fs kept=%zu dropped_time=%zu deskew=%s",
                ingested.time.field.c_str(),
                datatype_label(
                    time_field == nullptr ? 0 : time_field->datatype),
                lidar_time_.config().time_unit.c_str(),
                lidar_time_.config().time_ref.c_str(),
                ingested.time.span_s,
                ingested.time.header_offset_s,
                ingested.points.size(),
                ingested.dropped_time,
                ingested.deskewable ? "on" : "off");
            RCLCPP_INFO(
                this->get_logger(),
                "[sensor] profile: time_field=\"%s\" time_unit=\"%s\" "
                "time_ref=\"%s\"",
                lidar_time_.config().time_field.c_str(),
                lidar_time_.config().time_unit.c_str(),
                lidar_time_.config().time_ref.c_str());
        } else if (time_commit.warn_span_ratio) {
            RCLCPP_WARN(
                this->get_logger(),
                "[sensor] lidar span changed by more than 10x under the "
                "locked unit: locked=%.6fs now=%.6fs",
                time_commit.locked_span_s,
                time_commit.span_s);
        }
        if (time_commit.warn_header_mismatch) {
            RCLCPP_WARN(
                this->get_logger(),
                "[sensor] lidar header_mismatch: header_offset=%+.6fs; "
                "using point time as absolute seconds",
                time_commit.header_offset_s);
        }

        pipeline_->pushLidar(
            stamp + lidar_time_offset_s_,
            to_core_cloud(ingested, lidar_time_offset_s_));

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
    } catch (const std::exception& error) {
        RCLCPP_ERROR_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            5000,
            "[sapphire_ros2] LiDAR callback failed: %s",
            error.what());
    }
}

void SapphireRos::imu_callback(
    const sensor_msgs::msg::Imu::ConstSharedPtr msg)
{
    try {
        sapphire::ImuData imu = to_core_imu(*msg);
        imu.stamp += imu_time_offset_s_;
        pipeline_->pushImu(imu);
        if (const auto propagated = pipeline_->latestPropagatedResult()) {
            publish_odometry(*propagated);
        }
    } catch (const std::exception& error) {
        RCLCPP_ERROR_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            5000,
            "[sapphire_ros2] IMU callback failed: %s",
            error.what());
    }
}

void SapphireRos::publish_odometry(const sapphire::OdometryResult& result) {
    const std::string& child_frame =
        pose_frame_ == "imu" ? imu_frame_ : lidar_frame_;
    const auto msg = to_ros_odometry(
        result, world_frame_, child_frame, T_pose_lidar_);
    const int64_t stamp_ns =
        static_cast<int64_t>(msg.header.stamp.sec) * 1000000000LL
        + static_cast<int64_t>(msg.header.stamp.nanosec);

    std::lock_guard<std::mutex> lock(odometry_publish_mutex_);
    odom_pub_->publish(msg);
    if (stamp_ns <= last_tf_stamp_ns_) {
        return;
    }
    last_tf_stamp_ns_ = stamp_ns;

    geometry_msgs::msg::TransformStamped transform;
    transform.header = msg.header;
    transform.child_frame_id = msg.child_frame_id;
    transform.transform.translation.x = msg.pose.pose.position.x;
    transform.transform.translation.y = msg.pose.pose.position.y;
    transform.transform.translation.z = msg.pose.pose.position.z;
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
    path.header.stamp =
        stamp_from_seconds(trajectory.samples.back().stamp);
    path.poses.reserve(trajectory.samples.size());
    for (const sapphire::MapTrajectorySample& sample : trajectory.samples) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = world_frame_;
        pose.header.stamp = stamp_from_seconds(sample.stamp);
        pose.pose = to_ros_pose(sample.T_map_lidar * T_pose_lidar_);
        path.poses.push_back(std::move(pose));
    }
    trajectory_pub_->publish(path);
    trajectory_published_ = true;
    trajectory_samples_published_ = trajectory.samples.size();
    trajectory_revision_published_ = trajectory.revision;
}

void SapphireRos::publish_pgo_visualization() {
    publish_trajectory();
    const auto stamp =
        static_cast<builtin_interfaces::msg::Time>(this->now());

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

            auto graph_point = [](const sapphire::Isometry3d& pose) {
                geometry_msgs::msg::Point point;
                point.x = pose.translation().x();
                point.y = pose.translation().y();
                point.z = pose.translation().z();
                return point;
            };
            for (size_t i = 1; i < snapshot.optimized_poses.size(); ++i) {
                odometry_edges.points.push_back(
                    graph_point(snapshot.optimized_poses[i - 1]));
                odometry_edges.points.push_back(
                    graph_point(snapshot.optimized_poses[i]));
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
                    graph_point(snapshot.optimized_poses[edge.source]));
                loop_edges.points.push_back(
                    graph_point(snapshot.optimized_poses[edge.target]));
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
                nodes.points.push_back(graph_point(pose));
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
            pgo_map_pub_->publish(
                to_ros_point_cloud(*map, stamp, world_frame_));
        }
    }
}

void SapphireRos::pgo_visualization_callback() {
    try {
        publish_pgo_visualization();
    } catch (const std::exception& error) {
        RCLCPP_ERROR_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            5000,
            "[sapphire_ros2] PGO visualization failed: %s",
            error.what());
    }
}

void SapphireRos::publish_deskewed(double stamp) {
    if (deskewed_pub_->get_subscription_count() == 0) return;

    auto cloud = pipeline_->latestDeskewed();
    if (!cloud || cloud->points.empty()) return;

    deskewed_pub_->publish(
        to_ros_point_cloud(
            *cloud, stamp_from_seconds(stamp), world_frame_));
}

}  // namespace sapphire_ros2
