import os
import sys

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

sys.path.insert(0, os.path.dirname(__file__))
from _replay_utils import validate_bag_path  # noqa: E402


def _value(context, name):
    return context.perform_substitution(LaunchConfiguration(name))


def _setup(context):
    package_share = context.perform_substitution(FindPackageShare('aero_halo_360'))
    bag_path = validate_bag_path(_value(context, 'bag_path'))
    main_launch = os.path.join(package_share, 'launch', 'aero_halo_360.launch.py')

    forwarded_names = [
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
    forwarded = {name: _value(context, name) for name in forwarded_names}
    forwarded.update({
        'start_mavlink': 'false',
        'timestamp_mode': 'replay',
        'use_sim_time': _value(context, 'use_sim_time'),
    })

    replay = Node(
        package='aero_halo_360',
        executable='replay_to_mavlink.py',
        name='aero_halo_360_replay_controller',
        arguments=[
            bag_path,
            '--no-launch',
            '--wait-node', '/cloud_processor_node',
            '--ready-timeout', _value(context, 'ready_timeout'),
            '--start-delay', _value(context, 'start_delay'),
            '--clock', _value(context, 'use_sim_time'),
            '--loop', _value(context, 'loop'),
        ],
        output='screen',
    )
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(main_launch),
            launch_arguments=forwarded.items(),
        ),
        replay,
        RegisterEventHandler(OnProcessExit(
            target_action=replay,
            on_exit=[EmitEvent(event=Shutdown(reason='rosbag 回放进程已结束'))],
        )),
    ]


def generate_launch_description():
    package_share = FindPackageShare('aero_halo_360')
    return LaunchDescription([
        DeclareLaunchArgument('bag_path', default_value=''),
        DeclareLaunchArgument('base_config', default_value=PathJoinSubstitution([
            package_share, 'config', 'default.yaml'])),
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
        DeclareLaunchArgument('ready_timeout', default_value='15.0'),
        DeclareLaunchArgument('start_delay', default_value='1.0'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('loop', default_value='false'),
        OpaqueFunction(function=_setup),
    ])
