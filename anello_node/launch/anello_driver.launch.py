"""Launch file for the ANELLO ROS2 driver."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # ── Communication parameters ──
        DeclareLaunchArgument('com_type', default_value='UART',
                              description='Communication type: UART or ETH'),
        DeclareLaunchArgument('uart_data_port', default_value='AUTO',
                              description='UART data port path or AUTO'),
        DeclareLaunchArgument('uart_config_port', default_value='AUTO',
                              description='UART config port path, AUTO, or OFF'),
        DeclareLaunchArgument('baud_rate', default_value='230400',
                              description='Serial baud rate'),
        DeclareLaunchArgument('remote_ip', default_value='192.168.1.111',
                              description='Remote IP for ethernet mode'),
        DeclareLaunchArgument('local_data_port', default_value='1111',
                              description='Local UDP data port'),
        DeclareLaunchArgument('local_config_port', default_value='2222',
                              description='Local UDP config port'),
        DeclareLaunchArgument('local_odometer_port', default_value='3333',
                              description='Local UDP odometer port'),

        # ── Frame IDs ──
        DeclareLaunchArgument('frame_id_imu', default_value='imu_link',
                              description='Frame ID for IMU messages'),
        DeclareLaunchArgument('frame_id_ins', default_value='ins_link',
                              description='Frame ID for INS messages'),
        DeclareLaunchArgument('frame_id_gnss', default_value='gnss_link',
                              description='Frame ID for GNSS messages'),
        DeclareLaunchArgument('frame_id_hdg', default_value='gnss_link',
                              description='Frame ID for heading messages'),

        # ── TF ──
        DeclareLaunchArgument('publish_tf', default_value='true',
                              description='Publish the tf_parent_frame -> '
                                          'tf_child_frame transform'),
        DeclareLaunchArgument('tf_parent_frame', default_value='odom',
                              description='Parent frame for TF broadcast'),
        DeclareLaunchArgument('tf_child_frame', default_value='base_link',
                              description='Child frame for TF broadcast and '
                                          '/odom child_frame_id (REP-105 '
                                          'base_link; set ins_link for legacy '
                                          'behavior)'),

        # ── Polling ──
        DeclareLaunchArgument('poll_interval_ms', default_value='5',
                              description='Main loop poll interval (ms)'),

        # ── Health monitoring ──
        DeclareLaunchArgument('heading_baseline', default_value='0.0',
                              description='Dual-antenna baseline length (m) for '
                                          'heading health validation, 0.0 = skip'),

        # ── NTRIP parameters ──
        DeclareLaunchArgument('ntrip_host', default_value='',
                              description='NTRIP caster host'),
        DeclareLaunchArgument('ntrip_port', default_value='2101',
                              description='NTRIP caster port'),
        DeclareLaunchArgument('ntrip_mountpoint', default_value='',
                              description='NTRIP mountpoint'),
        DeclareLaunchArgument('ntrip_authenticate', default_value='false',
                              description='Enable NTRIP authentication'),
        DeclareLaunchArgument('ntrip_username', default_value='',
                              description='NTRIP username'),
        DeclareLaunchArgument('ntrip_password', default_value='',
                              description='NTRIP password'),

        # ── ANELLO driver node ──
        Node(
            package='anello_ros_driver',
            executable='anello_ros_driver_node',
            name='anello_ros_driver',
            output='screen',
            parameters=[{
                'com_type': LaunchConfiguration('com_type'),
                'uart_data_port': LaunchConfiguration('uart_data_port'),
                'uart_config_port': LaunchConfiguration('uart_config_port'),
                'baud_rate': LaunchConfiguration('baud_rate'),
                'remote_ip': LaunchConfiguration('remote_ip'),
                'local_data_port': LaunchConfiguration('local_data_port'),
                'local_config_port': LaunchConfiguration('local_config_port'),
                'local_odometer_port': LaunchConfiguration('local_odometer_port'),
                'frame_id.imu': LaunchConfiguration('frame_id_imu'),
                'frame_id.ins': LaunchConfiguration('frame_id_ins'),
                'frame_id.gnss': LaunchConfiguration('frame_id_gnss'),
                'frame_id.hdg': LaunchConfiguration('frame_id_hdg'),
                'publish_tf': LaunchConfiguration('publish_tf'),
                'tf_parent_frame': LaunchConfiguration('tf_parent_frame'),
                'tf_child_frame': LaunchConfiguration('tf_child_frame'),
                'poll_interval_ms': LaunchConfiguration('poll_interval_ms'),
                'heading_baseline': LaunchConfiguration('heading_baseline'),
            }],
        ),

        # ── NTRIP client node (only when a caster host is configured) ──
        Node(
            package='ntrip_client',
            executable='ntrip_ros',
            name='ntrip_client',
            output='screen',
            condition=IfCondition(PythonExpression(
                ["'", LaunchConfiguration('ntrip_host'), "' != ''"])),
            parameters=[{
                'host': LaunchConfiguration('ntrip_host'),
                'port': LaunchConfiguration('ntrip_port'),
                'mountpoint': LaunchConfiguration('ntrip_mountpoint'),
                'authenticate': LaunchConfiguration('ntrip_authenticate'),
                'username': LaunchConfiguration('ntrip_username'),
                'password': LaunchConfiguration('ntrip_password'),
            }],
        ),
    ])
