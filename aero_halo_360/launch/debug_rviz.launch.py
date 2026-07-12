from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


FORWARDED_ARGUMENTS = [
    'base_config',
    'profile',
    'profile_config',
    'installation_config',
    'input_topic',
    'target_frame',
    'use_static_lidar_tf',
    'lidar_parent_frame',
    'lidar_child_frame',
    'lidar_xyz',
    'lidar_rpy',
]


def generate_launch_description():
    package_share = FindPackageShare('aero_halo_360')
    main_launch = PathJoinSubstitution([
        package_share, 'launch', 'aero_halo_360.launch.py'
    ])
    rviz_config = PathJoinSubstitution([
        package_share, 'rviz', 'aero_halo_360.rviz'
    ])
    declarations = [
        DeclareLaunchArgument('base_config', default_value=PathJoinSubstitution([
            package_share, 'config', 'default.yaml'
        ])),
        DeclareLaunchArgument('profile', default_value='mission_planner_demo'),
        DeclareLaunchArgument('profile_config', default_value=''),
        DeclareLaunchArgument('installation_config', default_value=''),
        DeclareLaunchArgument('input_topic', default_value=''),
        DeclareLaunchArgument('target_frame', default_value=''),
        DeclareLaunchArgument('use_static_lidar_tf', default_value=''),
        DeclareLaunchArgument('lidar_parent_frame', default_value=''),
        DeclareLaunchArgument('lidar_child_frame', default_value=''),
        DeclareLaunchArgument('lidar_xyz', default_value=''),
        DeclareLaunchArgument('lidar_rpy', default_value=''),
    ]
    forwarded = {
        name: LaunchConfiguration(name)
        for name in FORWARDED_ARGUMENTS
    }
    forwarded.update({
        'start_mavlink': 'false',
        'publish_filtered_cloud': 'true',
        'publish_markers': 'true',
    })
    return LaunchDescription(declarations + [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(main_launch),
            launch_arguments=forwarded.items(),
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config],
            output='screen',
        ),
    ])
