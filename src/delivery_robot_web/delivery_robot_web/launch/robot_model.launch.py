"""Use the user's original URDF. No joint_state_publisher GUI, no extra lidar TF."""
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_model(context):
    path = Path(LaunchConfiguration('urdf').perform(context)).expanduser()
    if not path.is_file():
        raise RuntimeError(f'URDF does not exist: {path}')
    return [Node(package='robot_state_publisher', executable='robot_state_publisher',
                 name='robot_state_publisher', output='screen',
                 parameters=[{'robot_description': path.read_text(), 'use_sim_time': False}])]


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('urdf'), OpaqueFunction(function=launch_model)])
