import os
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    share = get_package_share_directory('delivery_robot_nav2')
    params = os.path.join(share, 'config', 'planner_v1.yaml')
    planner = Node(
        package='nav2_planner', executable='planner_server',
        name='planner_server', output='screen', parameters=[params],
    )
    manager = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_planner', output='screen',
        parameters=[{
            'autostart': True,
            'bond_timeout': 4.0,
            'node_names': ['planner_server'],
        }],
    )
    return LaunchDescription([
        planner,
        TimerAction(period=1.0, actions=[manager]),
    ])
