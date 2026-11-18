import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    share = get_package_share_directory('delivery_robot_nav2')
    nav_launch = os.path.join(
        share, 'launch', 'navigation_mux.launch.py'
    )

    mux = Node(
        package='delivery_robot_cmd_mux',
        executable='cmd_mux',
        name='delivery_robot_cmd_mux',
        output='screen',
        parameters=[{
            'initial_mode': 'STOP',
            'manual_topic': '/cmd_vel_manual',
            'nav_topic': '/cmd_vel_nav',
            'output_topic': '/cmd_vel',
            'manual_timeout': 0.35,
            'nav_timeout': 0.35,
            'publish_rate': 20.0,
            'max_linear': 0.10,
            'max_angular': 0.35,
        }],
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav_launch)
    )

    return LaunchDescription([
        LogInfo(
            msg='FULL NAV2 + CMD MUX: REAL OUTPUT, INITIAL STOP'
        ),
        mux,
        navigation,
    ])
