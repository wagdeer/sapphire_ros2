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
        sapphire_node,
    ])
