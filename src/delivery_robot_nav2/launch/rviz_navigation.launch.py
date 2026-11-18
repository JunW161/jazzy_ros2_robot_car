import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    description_share = get_package_share_directory('delivery_robot_description')
    rviz_default = os.path.join(description_share, 'rviz', 'display.rviz')
    return LaunchDescription([
        DeclareLaunchArgument('rviz_config', default_value=rviz_default),
        Node(package='rviz2', executable='rviz2', name='rviz2',
             output='screen', arguments=['-d', LaunchConfiguration('rviz_config')]),
    ])
