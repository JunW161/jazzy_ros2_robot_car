#!/usr/bin/env python3
"""Use existing ldlidar_sl_ros2 driver with a virtual serial port from socat."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    serial_port = LaunchConfiguration('serial_port')
    laser_x = LaunchConfiguration('laser_x')
    laser_y = LaunchConfiguration('laser_y')
    laser_z = LaunchConfiguration('laser_z')
    laser_yaw = LaunchConfiguration('laser_yaw')

    return LaunchDescription([
        DeclareLaunchArgument('serial_port', default_value='/tmp/ld14p_wifi'),
        DeclareLaunchArgument('laser_x', default_value='0.0'),
        DeclareLaunchArgument('laser_y', default_value='0.0'),
        DeclareLaunchArgument('laser_z', default_value='0.18'),
        DeclareLaunchArgument('laser_yaw', default_value='0.0'),
        Node(
            package='ldlidar_sl_ros2',
            executable='ldlidar_sl_ros2_node',
            name='ldlidar_publisher_ld14',
            output='screen',
            parameters=[{
                'product_name': 'LDLiDAR_LD14P',
                'laser_scan_topic_name': 'scan',
                'point_cloud_2d_topic_name': 'pointcloud2d',
                'frame_id': 'base_laser',
                'port_name': serial_port,
                # This is the POSIX virtual port baud setting, NOT UART1 input.
                'serial_baudrate': 230400,
                'laser_scan_dir': True,
                'enable_angle_crop_func': False,
                'angle_crop_min': 135.0,
                'angle_crop_max': 225.0,
            }],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_base_laser_ld14p',
            arguments=[
                '--x', laser_x, '--y', laser_y, '--z', laser_z,
                '--roll', '0', '--pitch', '0', '--yaw', laser_yaw,
                '--frame-id', 'base_link', '--child-frame-id', 'base_laser',
            ],
        ),
    ])
