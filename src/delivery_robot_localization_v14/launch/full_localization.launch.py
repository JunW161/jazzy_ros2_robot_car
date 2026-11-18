from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    share = Path(get_package_share_directory('delivery_robot_localization_v14')) / 'launch'
    map_arg = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    return LaunchDescription([
        DeclareLaunchArgument('map', default_value='', description='Absolute map YAML path.'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(share / 'official_odometry.launch.py')),
            launch_arguments={'start_bridge': 'true', 'use_sim_time': use_sim_time}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(share / 'amcl_localization.launch.py')),
            launch_arguments={'map': map_arg, 'use_sim_time': use_sim_time}.items(),
        ),
    ])
