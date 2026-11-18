from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package='delivery_robot_mission', executable='mission_manager', name='delivery_robot_mission_manager', output='screen')
    ])
