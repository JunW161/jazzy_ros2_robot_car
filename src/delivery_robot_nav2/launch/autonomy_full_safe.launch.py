import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def _include(package, launch_filename, arguments=None):
    path = os.path.join(get_package_share_directory(package), 'launch', launch_filename)
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(path),
        launch_arguments=(arguments or {}).items(),
    )


def generate_launch_description():
    actions = [
        LogInfo(msg='AUTONOMY - SAFE: /cmd_vel_nav_test, no motor output; do not run parallel Nav2/localization/Web Bridge instances'),
    ]
    actions.append(_include('delivery_robot_nav2', 'navigation_safe.launch.py'))
    actions.append(_include('delivery_robot_mission', 'mission_manager.launch.py'))
    actions.append(_include('delivery_robot_web_bridge', 'web_bridge.launch.py'))
    return LaunchDescription(actions)
