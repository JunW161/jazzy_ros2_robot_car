from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([Node(package='delivery_robot_web_bridge', executable='web_bridge', output='screen')])
