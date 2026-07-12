import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

sys.path.insert(0, os.path.dirname(__file__))
from _config_layering import (  # noqa: E402
    build_effective_parameters,
    file_sha256,
    normalize_config_path,
    optional_cli_overrides,
    parse_bool,
    parse_vector,
)


OPTIONAL_ARGUMENTS = {
    'input_topic': 'input_topic',
    'target_frame': 'target_frame',
    'timestamp_mode': 'timestamp.mode',
    'use_sim_time': 'use_sim_time',
    'publish_filtered_cloud': 'debug.publish_filtered_cloud',
    'publish_markers': 'debug.publish_markers',
    'publish_diagnostics': 'debug.publish_diagnostics',
    'mavlink_connection': 'mavlink.connection',
    'mavlink_baud': 'mavlink.baud',
    'wait_heartbeat_on_connect': 'mavlink.wait_heartbeat_on_connect',
    'cloud_warn_timeout_ms': 'watchdog.cloud_warn_timeout_ms',
    'cloud_fail_timeout_ms': 'watchdog.cloud_fail_timeout_ms',
    'mavlink_input_warn_timeout_ms': 'mavlink.input_warn_timeout_ms',
    'mavlink_input_fail_timeout_ms': 'mavlink.input_fail_timeout_ms',
}

BOOL_PARAMETER_KEYS = {
    'debug.publish_filtered_cloud',
    'debug.publish_markers',
    'debug.publish_diagnostics',
    'mavlink.wait_heartbeat_on_connect',
    'use_sim_time',
}

INT_PARAMETER_KEYS = {
    'mavlink.baud',
    'watchdog.cloud_warn_timeout_ms',
    'watchdog.cloud_fail_timeout_ms',
    'mavlink.input_warn_timeout_ms',
    'mavlink.input_fail_timeout_ms',
}


def _value(context, name):
    return context.perform_substitution(LaunchConfiguration(name))


def _resolve_profile(context, package_share):
    profile = _value(context, 'profile').strip()
    explicit = _value(context, 'profile_config').strip()
    if profile and explicit:
        raise RuntimeError('profile 与 profile_config 不能同时设置')
    if explicit:
        return normalize_config_path(explicit, 'profile_config')
    if not profile:
        return None
    if '/' in profile or '\\' in profile or profile.endswith('.yaml'):
        raise RuntimeError('profile 只能是包内配置名称，例如 flight_low_speed')
    return normalize_config_path(
        os.path.join(package_share, 'config', f'{profile}.yaml'),
        'profile',
    )


def _resolve_optional_file(context, argument_name):
    value = _value(context, argument_name).strip()
    return normalize_config_path(value, argument_name) if value else None


def _configured_vector(parameters, key, default):
    value = parameters.get(key, default)
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        raise RuntimeError(f'{key} 必须包含 3 个数值')
    return tuple(float(item) for item in value)


def _launch_setup(context, *args, **kwargs):
    del args, kwargs
    package_share = get_package_share_directory('aero_halo_360')
    base_path = normalize_config_path(_value(context, 'base_config'), 'base_config')
    profile_path = _resolve_profile(context, package_share)
    installation_path = _resolve_optional_file(context, 'installation_config')

    raw_values = {
        parameter_name: _value(context, argument_name)
        for argument_name, parameter_name in OPTIONAL_ARGUMENTS.items()
    }
    overrides = optional_cli_overrides(
        raw_values,
        BOOL_PARAMETER_KEYS,
        INT_PARAMETER_KEYS,
    )
    parameters = build_effective_parameters(
        base_path,
        profile_path,
        overrides,
        installation_path=installation_path,
    )
    profile_name = _value(context, "profile").strip() or (
        os.path.basename(profile_path) if profile_path else "default")
    parameters["runtime.profile_name"] = profile_name
    runtime_parameters = {
        key: value for key, value in parameters.items()
        if not key.startswith('installation.')
    }

    profile_text = '未使用'
    if profile_path:
        profile_text = f'{profile_path} sha256={file_sha256(profile_path)}'
    installation_text = '未使用'
    if installation_path:
        installation_text = (
            f'{installation_path} sha256={file_sha256(installation_path)}')

    actions = [
        LogInfo(msg=(
            f'AeroHalo360 base_config={base_path} sha256={file_sha256(base_path)}')),
        LogInfo(msg=f'AeroHalo360 profile_config={profile_text}'),
        LogInfo(msg=f'AeroHalo360 installation_config={installation_text}'),
        LogInfo(msg=f'AeroHalo360 CLI overrides={sorted(overrides.keys()) or "无"}'),
    ]

    respawn = parse_bool('respawn_nodes', _value(context, 'respawn_nodes'))
    actions.append(Node(
        package='aero_halo_360',
        executable='cloud_processor_node',
        name='cloud_processor_node',
        output='screen',
        parameters=[runtime_parameters],
        respawn=respawn,
        respawn_delay=2.0,
    ))

    if parse_bool('start_mavlink', _value(context, 'start_mavlink')):
        actions.append(Node(
            package='aero_halo_360',
            executable='mavlink_obstacle_sender.py',
            name='mavlink_obstacle_sender',
            output='screen',
            parameters=[runtime_parameters],
            respawn=respawn,
            respawn_delay=2.0,
        ))

    static_tf_raw = _value(context, 'use_static_lidar_tf').strip()
    static_tf_enable = (
        parse_bool('use_static_lidar_tf', static_tf_raw)
        if static_tf_raw else bool(parameters.get('installation.static_tf.enable', False))
    )
    if static_tf_enable:
        xyz_raw = _value(context, 'lidar_xyz').strip()
        rpy_raw = _value(context, 'lidar_rpy').strip()
        xyz = (
            parse_vector('lidar_xyz', xyz_raw, 3)
            if xyz_raw else _configured_vector(
                parameters, 'installation.static_tf.xyz', (0.0, 0.0, 0.0))
        )
        rpy = (
            parse_vector('lidar_rpy', rpy_raw, 3)
            if rpy_raw else _configured_vector(
                parameters, 'installation.static_tf.rpy', (0.0, 0.0, 0.0))
        )
        parent = _value(context, 'lidar_parent_frame').strip() or str(
            parameters.get('installation.static_tf.parent_frame', 'base_link'))
        child = _value(context, 'lidar_child_frame').strip() or str(
            parameters.get('installation.static_tf.child_frame', 'livox_frame'))
        if not parent or not child or parent == child:
            raise RuntimeError('静态 TF 的 parent/child 必须非空且不能相同')
        actions.append(Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='aero_halo_360_static_lidar_tf',
            arguments=[
                '--x', str(xyz[0]), '--y', str(xyz[1]), '--z', str(xyz[2]),
                '--roll', str(rpy[0]), '--pitch', str(rpy[1]), '--yaw', str(rpy[2]),
                '--frame-id', parent, '--child-frame-id', child,
            ],
            output='screen',
            respawn=respawn,
            respawn_delay=2.0,
        ))

    return actions


def generate_launch_description():
    package_share = get_package_share_directory('aero_halo_360')
    default_base = os.path.join(package_share, 'config', 'default.yaml')

    declarations = [
        DeclareLaunchArgument('base_config', default_value=default_base),
        DeclareLaunchArgument('profile', default_value=''),
        DeclareLaunchArgument('profile_config', default_value=''),
        DeclareLaunchArgument('installation_config', default_value=''),
        DeclareLaunchArgument('start_mavlink', default_value='false'),
        DeclareLaunchArgument('respawn_nodes', default_value='true'),
        DeclareLaunchArgument('input_topic', default_value=''),
        DeclareLaunchArgument('target_frame', default_value=''),
        DeclareLaunchArgument('timestamp_mode', default_value=''),
        DeclareLaunchArgument('use_sim_time', default_value=''),
        DeclareLaunchArgument('publish_filtered_cloud', default_value=''),
        DeclareLaunchArgument('publish_markers', default_value=''),
        DeclareLaunchArgument('publish_diagnostics', default_value=''),
        DeclareLaunchArgument('mavlink_connection', default_value=''),
        DeclareLaunchArgument('mavlink_baud', default_value=''),
        DeclareLaunchArgument('wait_heartbeat_on_connect', default_value=''),
        DeclareLaunchArgument('cloud_warn_timeout_ms', default_value=''),
        DeclareLaunchArgument('cloud_fail_timeout_ms', default_value=''),
        DeclareLaunchArgument('mavlink_input_warn_timeout_ms', default_value=''),
        DeclareLaunchArgument('mavlink_input_fail_timeout_ms', default_value=''),
        DeclareLaunchArgument('use_static_lidar_tf', default_value=''),
        DeclareLaunchArgument('lidar_parent_frame', default_value=''),
        DeclareLaunchArgument('lidar_child_frame', default_value=''),
        DeclareLaunchArgument('lidar_xyz', default_value=''),
        DeclareLaunchArgument('lidar_rpy', default_value=''),
    ]
    return LaunchDescription(declarations + [OpaqueFunction(function=_launch_setup)])
