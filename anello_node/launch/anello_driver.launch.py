"""Typed overrides; unspecified values come from the parameter file or node."""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import yaml


DRIVER_PARAMETERS = {
    'com_type': str, 'uart_data_port': str, 'uart_config_port': str,
    'baud_rate': int, 'remote_ip': str, 'local_data_port': int,
    'local_config_port': int, 'local_odometer_port': int,
    'frame_id.imu': str, 'frame_id.ins': str, 'frame_id.gnss': str,
    'frame_id.gnss2': str, 'frame_id.hdg': str,
    'publish_tf': bool, 'tf_parent_frame': str, 'tf_child_frame': str,
    'poll_interval_ms': int, 'heading_baseline': float,
    'timestamp_source': str, 'flip_accel_sign': bool, 'use_fog_wz': bool,
    'covariance.angular_velocity': list, 'covariance.linear_acceleration': list,
    'covariance.device_convention': str, 'covariance.max_age': float,
    'covariance.unknown_variance': float, 'imu_max_age': float,
    'stream_timeout': float, 'expected_streams': list,
    'gps_utc_leap_seconds': int, 'gnss_service_mask': int, 'imu_output_rate_hz': float,
    'accel_sign_check_upright': bool, 'publish_custom_messages': bool,
    'use_sim_time': bool,
}
NTRIP_PARAMETERS = {
    'host': str, 'port': int, 'mountpoint': str, 'authenticate': bool,
    'username': str, 'password': str, 'ntrip_version': str, 'ssl': bool,
    'cert': str, 'key': str, 'ca_cert': str, 'rtcm_frame_id': str,
    'reconnect_attempt_wait_seconds': int, 'rtcm_timeout_seconds': int,
    'nmea_min_interval_seconds': float, 'nmea_max_age_seconds': float,
}


def argument_name(parameter, prefix=''):
    return prefix + parameter.replace('.', '_') if parameter != 'ntrip_version' else parameter


def parameter_overrides(context, definitions, prefix=''):
    values = {}
    for name, kind in definitions.items():
        raw = LaunchConfiguration(argument_name(name, prefix)).perform(context)
        if raw == '':
            continue
        if kind is list:
            value = yaml.safe_load(raw)
            if not isinstance(value, list):
                raise ValueError('{} must be a YAML list'.format(name))
            if name.startswith('covariance.'):
                value = [float(item) for item in value]
            elif not all(isinstance(item, str) for item in value):
                raise ValueError('{} must contain strings'.format(name))
            values[name] = value
        elif kind is bool:
            if raw.lower() not in ('true', 'false'):
                raise ValueError('{} must be true or false'.format(name))
            values[name] = raw.lower() == 'true'
        else:
            values[name] = ParameterValue(kind(raw), value_type=kind)
    return values


def configured_nodes(context):
    def parameters(file_arg, definitions, prefix=''):
        path = LaunchConfiguration(file_arg).perform(context)
        if path and not Path(path).is_file():
            raise ValueError('Parameter file does not exist: {}'.format(path))
        return ([path] if path else []) + [parameter_overrides(context, definitions, prefix)]

    nodes = [Node(
        package='anello_ros_driver', executable='anello_ros_driver_node',
        name='anello_ros_driver', output='screen',
        parameters=parameters('params_file', DRIVER_PARAMETERS))]
    enabled = LaunchConfiguration('ntrip_enable').perform(context).lower()
    if enabled not in ('true', 'false'):
        raise ValueError('ntrip_enable must be true or false')
    host = LaunchConfiguration('ntrip_host').perform(context)
    if enabled == 'true' or host:
        nodes.append(Node(
            package='ntrip_client', executable='ntrip_ros', name='ntrip_client', output='screen',
            parameters=parameters('ntrip_params_file', NTRIP_PARAMETERS, 'ntrip_')))
    return nodes


def generate_launch_description():
    arguments = [
        DeclareLaunchArgument('params_file', default_value=''),
        DeclareLaunchArgument('ntrip_params_file', default_value=''),
        DeclareLaunchArgument('ntrip_enable', default_value='false',
                              description='Start NTRIP using its parameter file; '
                                          'a host override also enables it'),
    ]
    for definitions, prefix in ((DRIVER_PARAMETERS, ''), (NTRIP_PARAMETERS, 'ntrip_')):
        arguments.extend(DeclareLaunchArgument(
            argument_name(name, prefix), default_value='',
            description='Optional override for {}'.format(name)) for name in definitions)
    return LaunchDescription(arguments + [OpaqueFunction(function=configured_nodes)])
