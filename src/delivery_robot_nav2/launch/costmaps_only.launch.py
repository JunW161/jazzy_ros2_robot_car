import os
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    share = get_package_share_directory('delivery_robot_nav2')
    params = os.path.join(share, 'config', 'costmaps_v1.yaml')
    global_costmap = Node(
        package='nav2_costmap_2d', executable='nav2_costmap_2d',
        name='global_costmap', namespace='global_costmap',
        output='screen', parameters=[params],
    )
    local_costmap = Node(
        package='nav2_costmap_2d', executable='nav2_costmap_2d',
        name='local_costmap', namespace='local_costmap',
        output='screen', parameters=[params],
    )
    manager = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_costmaps', output='screen',
        parameters=[{
            'autostart': True,
            'bond_timeout': 4.0,
            'node_names': [
                '/global_costmap/global_costmap',
                '/local_costmap/local_costmap',
            ],
        }],
    )
    return LaunchDescription([
        global_costmap,
        local_costmap,
        TimerAction(period=1.0, actions=[manager]),
    ])
