"""
Sapphire SLAM — ROS2 launch file

Usage:
  ros2 launch sapphire_ros2 sapphire.launch.py
  ros2 launch sapphire_ros2 sapphire.launch.py lidar_topic:=/velodyne_points imu_topic:=/imu_raw
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    # Arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('sapphire_ros2'),
            'cfg',
            'sapphire_mid360.toml',
        ]),
        description='Sapphire core algorithm TOML configuration'
    )
    lidar_topic_arg = DeclareLaunchArgument(
        'lidar_topic',
        default_value='/livox/lidar',
        description='LiDAR pointcloud topic'
    )
    imu_topic_arg = DeclareLaunchArgument(
        'imu_topic',
        default_value='/livox/imu',
        description='IMU data topic'
    )
    odom_topic_arg = DeclareLaunchArgument(
        'odom_topic',
        default_value='/sapphire/odometry',
        description='Odometry output topic'
    )
    acc_scale_arg = DeclareLaunchArgument(
        'acc_scale',
        default_value='9.80665',
        description='Scale raw IMU acceleration to m/s^2 (Mid-360 outputs g)'
    )
    world_frame_arg = DeclareLaunchArgument(
        'world_frame',
        default_value='map',
        description='World frame used by odometry, TF, and deskewed clouds'
    )
    lidar_frame_arg = DeclareLaunchArgument(
        'lidar_frame',
        default_value='lidar',
        description='LiDAR child frame used by odometry and TF'
    )
    virtual_depth_enabled_arg = DeclareLaunchArgument(
        'virtual_depth_enabled', default_value='true',
        description='Publish the virtual forward-facing Z-depth camera'
    )
    virtual_depth_projection_model_arg = DeclareLaunchArgument(
        'virtual_depth_projection_model',
        default_value='pinhole',
        description='Projection model: pinhole or panorama'
    )
    virtual_depth_width_arg = DeclareLaunchArgument(
        'virtual_depth_width', default_value='640',
        description='Virtual depth image width'
    )
    virtual_depth_height_arg = DeclareLaunchArgument(
        'virtual_depth_height', default_value='320',
        description='Virtual depth image height'
    )
    virtual_depth_horizontal_fov_arg = DeclareLaunchArgument(
        'virtual_depth_horizontal_fov_deg', default_value='120.0',
        description='Virtual camera horizontal field of view in degrees'
    )
    virtual_depth_vertical_fov_arg = DeclareLaunchArgument(
        'virtual_depth_vertical_fov_deg', default_value='60.0',
        description='Virtual camera vertical field of view in degrees'
    )
    virtual_depth_panorama_min_elevation_arg = DeclareLaunchArgument(
        'virtual_depth_panorama_min_elevation_deg',
        default_value='-10.0',
        description='Lowest panorama elevation retained for Mid-360'
    )
    virtual_depth_panorama_max_elevation_arg = DeclareLaunchArgument(
        'virtual_depth_panorama_max_elevation_deg',
        default_value='55.0',
        description='Highest panorama elevation retained for Mid-360'
    )
    virtual_depth_near_arg = DeclareLaunchArgument(
        'virtual_depth_near_depth', default_value='0.5',
        description='Virtual camera near clipping depth in metres'
    )
    virtual_depth_far_arg = DeclareLaunchArgument(
        'virtual_depth_far_depth', default_value='100.0',
        description='Virtual camera far clipping depth in metres'
    )
    virtual_depth_hole_fill_radius_arg = DeclareLaunchArgument(
        'virtual_depth_hole_fill_radius', default_value='2',
        description='Normalized inverse-depth convolution radius in pixels'
    )
    virtual_depth_far_fill_arg = DeclareLaunchArgument(
        'virtual_depth_far_fill_depth', default_value='100.0',
        description='Depth assigned to pixels without local point support'
    )
    virtual_depth_debug_arg = DeclareLaunchArgument(
        'virtual_depth_publish_debug', default_value='false',
        description='Publish a BGR8 pseudo-colour depth debug image'
    )
    virtual_depth_debug_color_mode_arg = DeclareLaunchArgument(
        'virtual_depth_debug_color_mode',
        default_value='log',
        description='Debug color scaling: linear, log, or inverse'
    )
    virtual_depth_debug_max_depth_arg = DeclareLaunchArgument(
        'virtual_depth_debug_max_depth',
        default_value='30.0',
        description='Maximum depth represented by the debug color scale'
    )
    virtual_depth_frame_arg = DeclareLaunchArgument(
        'virtual_depth_frame_id',
        default_value='virtual_camera_optical_frame',
        description='Optical frame used by virtual depth image messages'
    )
    virtual_depth_orientation_arg = DeclareLaunchArgument(
        'virtual_depth_orientation_mode',
        default_value='startup_calibrated',
        description='Depth orientation: lidar or startup_calibrated'
    )
    virtual_depth_yaw_offset_arg = DeclareLaunchArgument(
        'virtual_depth_yaw_offset_deg',
        default_value='0.0',
        description='Additional virtual forward yaw offset in degrees'
    )
    virtual_depth_history_frames_arg = DeclareLaunchArgument(
        'virtual_depth_history_frame_count',
        default_value='5',
        description='Number of registered historical scans used for hole fill'
    )
    virtual_depth_history_stride_arg = DeclareLaunchArgument(
        'virtual_depth_history_stride',
        default_value='5',
        description='Select every Nth accepted historical scan'
    )
    virtual_depth_history_max_age_arg = DeclareLaunchArgument(
        'virtual_depth_history_max_age_sec',
        default_value='1.5',
        description='Maximum age of a historical scan in seconds'
    )

    # Node
    sapphire_node = Node(
        package='sapphire_ros2',
        executable='sapphire_ros_node',
        name='sapphire_ros',
        output='screen',
        parameters=[{
            'config_file': LaunchConfiguration('config_file'),
            'acc_scale': ParameterValue(
                LaunchConfiguration('acc_scale'), value_type=float
            ),
            'world_frame': LaunchConfiguration('world_frame'),
            'lidar_frame': LaunchConfiguration('lidar_frame'),
            'virtual_depth.enabled': ParameterValue(
                LaunchConfiguration('virtual_depth_enabled'), value_type=bool
            ),
            'virtual_depth.projection_model':
                LaunchConfiguration('virtual_depth_projection_model'),
            'virtual_depth.width': ParameterValue(
                LaunchConfiguration('virtual_depth_width'), value_type=int
            ),
            'virtual_depth.height': ParameterValue(
                LaunchConfiguration('virtual_depth_height'), value_type=int
            ),
            'virtual_depth.horizontal_fov_deg': ParameterValue(
                LaunchConfiguration('virtual_depth_horizontal_fov_deg'),
                value_type=float
            ),
            'virtual_depth.vertical_fov_deg': ParameterValue(
                LaunchConfiguration('virtual_depth_vertical_fov_deg'),
                value_type=float
            ),
            'virtual_depth.panorama_min_elevation_deg': ParameterValue(
                LaunchConfiguration(
                    'virtual_depth_panorama_min_elevation_deg'
                ),
                value_type=float
            ),
            'virtual_depth.panorama_max_elevation_deg': ParameterValue(
                LaunchConfiguration(
                    'virtual_depth_panorama_max_elevation_deg'
                ),
                value_type=float
            ),
            'virtual_depth.near_depth': ParameterValue(
                LaunchConfiguration('virtual_depth_near_depth'),
                value_type=float
            ),
            'virtual_depth.far_depth': ParameterValue(
                LaunchConfiguration('virtual_depth_far_depth'),
                value_type=float
            ),
            'virtual_depth.hole_fill_radius': ParameterValue(
                LaunchConfiguration('virtual_depth_hole_fill_radius'),
                value_type=int
            ),
            'virtual_depth.far_fill_depth': ParameterValue(
                LaunchConfiguration('virtual_depth_far_fill_depth'),
                value_type=float
            ),
            'virtual_depth.publish_debug': ParameterValue(
                LaunchConfiguration('virtual_depth_publish_debug'),
                value_type=bool
            ),
            'virtual_depth.debug_color_mode':
                LaunchConfiguration('virtual_depth_debug_color_mode'),
            'virtual_depth.debug_max_depth': ParameterValue(
                LaunchConfiguration('virtual_depth_debug_max_depth'),
                value_type=float
            ),
            'virtual_depth.frame_id':
                LaunchConfiguration('virtual_depth_frame_id'),
            'virtual_depth.orientation_mode':
                LaunchConfiguration('virtual_depth_orientation_mode'),
            'virtual_depth.yaw_offset_deg': ParameterValue(
                LaunchConfiguration('virtual_depth_yaw_offset_deg'),
                value_type=float
            ),
            'virtual_depth.history_frame_count': ParameterValue(
                LaunchConfiguration('virtual_depth_history_frame_count'),
                value_type=int
            ),
            'virtual_depth.history_stride': ParameterValue(
                LaunchConfiguration('virtual_depth_history_stride'),
                value_type=int
            ),
            'virtual_depth.history_max_age_sec': ParameterValue(
                LaunchConfiguration('virtual_depth_history_max_age_sec'),
                value_type=float
            ),
        }],
        remappings=[
            ('lidar/points', LaunchConfiguration('lidar_topic')),
            ('imu/data', LaunchConfiguration('imu_topic')),
            ('sapphire/odometry', LaunchConfiguration('odom_topic')),
        ],
    )

    return LaunchDescription([
        config_file_arg,
        lidar_topic_arg,
        imu_topic_arg,
        odom_topic_arg,
        acc_scale_arg,
        world_frame_arg,
        lidar_frame_arg,
        virtual_depth_enabled_arg,
        virtual_depth_projection_model_arg,
        virtual_depth_width_arg,
        virtual_depth_height_arg,
        virtual_depth_horizontal_fov_arg,
        virtual_depth_vertical_fov_arg,
        virtual_depth_panorama_min_elevation_arg,
        virtual_depth_panorama_max_elevation_arg,
        virtual_depth_near_arg,
        virtual_depth_far_arg,
        virtual_depth_hole_fill_radius_arg,
        virtual_depth_far_fill_arg,
        virtual_depth_debug_arg,
        virtual_depth_debug_color_mode_arg,
        virtual_depth_debug_max_depth_arg,
        virtual_depth_frame_arg,
        virtual_depth_orientation_arg,
        virtual_depth_yaw_offset_arg,
        virtual_depth_history_frames_arg,
        virtual_depth_history_stride_arg,
        virtual_depth_history_max_age_arg,
        sapphire_node,
    ])
