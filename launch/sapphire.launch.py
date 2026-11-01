"""Sapphire ROS 2 launch file."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_file = DeclareLaunchArgument(
        'config_file',
        default_value='',
        description='Required path to a Sapphire core algorithm TOML',
    )
    lidar_topic = DeclareLaunchArgument(
        'lidar_topic',
        default_value='/front_lidar',
        description='LiDAR PointCloud2 topic',
    )
    imu_topic = DeclareLaunchArgument(
        'imu_topic',
        default_value='/front_lidar/imu',
        description='IMU topic',
    )
    odom_topic = DeclareLaunchArgument(
        'odom_topic',
        default_value='/sapphire/odometry',
        description='Odometry output topic',
    )
    deskewed_topic = DeclareLaunchArgument(
        'deskewed_topic',
        default_value='/sapphire/deskewed',
        description='Deskewed point-cloud output topic',
    )
    trajectory_topic = DeclareLaunchArgument(
        'trajectory_topic',
        default_value='/sapphire/trajectory',
        description='Trajectory path output topic',
    )
    pgo_map_topic = DeclareLaunchArgument(
        'pgo_map_topic',
        default_value='/sapphire/pgo/map',
        description='Pose-graph map output topic',
    )
    pgo_graph_topic = DeclareLaunchArgument(
        'pgo_graph_topic',
        default_value='/sapphire/pgo/graph',
        description='Pose-graph marker output topic',
    )
    world_frame = DeclareLaunchArgument(
        'world_frame',
        default_value='map',
        description='World frame used by poses and point clouds',
    )
    lidar_frame = DeclareLaunchArgument(
        'lidar_frame',
        default_value='lidar',
        description='LiDAR frame name',
    )
    imu_frame = DeclareLaunchArgument(
        'imu_frame',
        default_value='imu',
        description='IMU frame name',
    )
    pose_frame = DeclareLaunchArgument(
        'pose_frame',
        default_value='lidar',
        choices=['lidar', 'imu'],
        description='Frame used for odometry, TF, path, and TUM output',
    )
    lidar_time_offset = DeclareLaunchArgument(
        'lidar_time_offset',
        default_value='0.0',
        description='LiDAR clock offset in seconds',
    )
    imu_time_offset = DeclareLaunchArgument(
        'imu_time_offset',
        default_value='0.0',
        description='IMU clock offset in seconds',
    )
    trajectory_output = DeclareLaunchArgument(
        'trajectory_output',
        default_value='',
        description='TUM output path override; empty uses the TOML value',
    )
    use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use /clock during bag playback or simulation',
    )
    rviz = DeclareLaunchArgument(
        'rviz',
        default_value='false',
        description='Start RViz2',
    )
    rviz_config = DeclareLaunchArgument(
        'rviz_config',
        default_value=PathJoinSubstitution([
            FindPackageShare('sapphire_ros2'),
            'launch',
            'display.rviz',
        ]),
        description='RViz2 configuration file',
    )
    rviz_image_topic = DeclareLaunchArgument(
        'rviz_image_topic',
        default_value='/camera/image_raw',
        description='Image topic used by the RViz2 display',
    )

    sapphire_node = Node(
        package='sapphire_ros2',
        executable='sapphire_ros_node',
        name='sapphire_ros',
        output='screen',
        parameters=[{
            'config_file': LaunchConfiguration('config_file'),
            'world_frame': LaunchConfiguration('world_frame'),
            'lidar_frame': LaunchConfiguration('lidar_frame'),
            'imu_frame': LaunchConfiguration('imu_frame'),
            'pose_frame': LaunchConfiguration('pose_frame'),
            'lidar_time_offset': ParameterValue(
                LaunchConfiguration('lidar_time_offset'),
                value_type=float,
            ),
            'imu_time_offset': ParameterValue(
                LaunchConfiguration('imu_time_offset'),
                value_type=float,
            ),
            'trajectory_output': LaunchConfiguration('trajectory_output'),
            'use_sim_time': ParameterValue(
                LaunchConfiguration('use_sim_time'),
                value_type=bool,
            ),
        }],
        remappings=[
            ('lidar/points', LaunchConfiguration('lidar_topic')),
            ('imu/data', LaunchConfiguration('imu_topic')),
            ('sapphire/odometry', LaunchConfiguration('odom_topic')),
            ('sapphire/deskewed', LaunchConfiguration('deskewed_topic')),
            ('sapphire/trajectory', LaunchConfiguration('trajectory_topic')),
            ('sapphire/pgo/map', LaunchConfiguration('pgo_map_topic')),
            ('sapphire/pgo/graph', LaunchConfiguration('pgo_graph_topic')),
        ],
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        condition=IfCondition(LaunchConfiguration('rviz')),
        arguments=['-d', LaunchConfiguration('rviz_config')],
        parameters=[{
            'use_sim_time': ParameterValue(
                LaunchConfiguration('use_sim_time'),
                value_type=bool,
            ),
        }],
        remappings=[
            ('visualization_marker_array',
             LaunchConfiguration('pgo_graph_topic')),
            ('image_raw', LaunchConfiguration('rviz_image_topic')),
        ],
    )

    return LaunchDescription([
        config_file,
        lidar_topic,
        imu_topic,
        odom_topic,
        deskewed_topic,
        trajectory_topic,
        pgo_map_topic,
        pgo_graph_topic,
        world_frame,
        lidar_frame,
        imu_frame,
        pose_frame,
        lidar_time_offset,
        imu_time_offset,
        trajectory_output,
        use_sim_time,
        rviz,
        rviz_config,
        rviz_image_topic,
        sapphire_node,
        rviz_node,
    ])
