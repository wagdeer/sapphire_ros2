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


def generate_launch_description():
    # Arguments
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

    # Node
    sapphire_node = Node(
        package='sapphire_ros2',
        executable='sapphire_ros_node',
        name='sapphire_ros',
        output='screen',
        remappings=[
            ('lidar/points', LaunchConfiguration('lidar_topic')),
            ('imu/data', LaunchConfiguration('imu_topic')),
            ('sapphire/odometry', LaunchConfiguration('odom_topic')),
        ],
    )

    return LaunchDescription([
        lidar_topic_arg,
        imu_topic_arg,
        odom_topic_arg,
        sapphire_node,
    ])
