import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    amcl_share = get_package_share_directory('delivery_robot_localization_v14')
    nav_share = get_package_share_directory('delivery_robot_nav2')
    amcl = os.path.join(amcl_share, 'launch', 'amcl_localization.launch.py')
    safe = os.path.join(nav_share, 'launch', 'navigation_safe.launch.py')
    return LaunchDescription([
        DeclareLaunchArgument('map', description='Full path to map YAML, required'),
        LogInfo(msg='LOCALIZATION + NAVIGATION SAFE: Start only if AMCL and map_server are not already running'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(amcl),
                                 launch_arguments={'map': LaunchConfiguration('map')}.items()),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(safe)),
    ])
