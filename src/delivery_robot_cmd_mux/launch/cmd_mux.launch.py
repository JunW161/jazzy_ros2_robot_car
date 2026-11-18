from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    mode = LaunchConfiguration('initial_mode')
    return LaunchDescription([
        DeclareLaunchArgument('initial_mode', default_value='STOP'),
        Node(
            package='delivery_robot_cmd_mux', executable='cmd_mux',
            name='delivery_robot_cmd_mux', output='screen',
            parameters=[{'initial_mode': mode}],
        ),
    ])
