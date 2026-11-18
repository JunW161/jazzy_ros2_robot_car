import os

from launch import LaunchDescription
from launch.actions import LogInfo, TimerAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    share = get_package_share_directory('delivery_robot_nav2')
    params = os.path.join(
        share, 'config', 'navigation_full_v1.yaml'
    )

    def nav_node(package, executable, name, remappings=None):
        return Node(
            package=package,
            executable=executable,
            name=name,
            output='screen',
            parameters=[params],
            remappings=remappings or [],
        )

    raw_velocity = [('cmd_vel', '/cmd_vel_nav_raw')]

    nodes = [
        nav_node(
            'nav2_planner', 'planner_server', 'planner_server'
        ),
        nav_node(
            'nav2_controller', 'controller_server',
            'controller_server', raw_velocity
        ),
        nav_node(
            'nav2_behaviors', 'behavior_server',
            'behavior_server', raw_velocity
        ),
        nav_node(
            'nav2_bt_navigator', 'bt_navigator', 'bt_navigator'
        ),
        nav_node(
            'nav2_waypoint_follower', 'waypoint_follower',
            'waypoint_follower'
        ),
        nav_node(
            'nav2_velocity_smoother', 'velocity_smoother',
            'velocity_smoother', raw_velocity
        ),
        nav_node(
            'nav2_collision_monitor', 'collision_monitor',
            'collision_monitor'
        ),
    ]

    lifecycle_nodes = [
        'controller_server',
        'planner_server',
        'behavior_server',
        'bt_navigator',
        'waypoint_follower',
        'velocity_smoother',
        'collision_monitor',
    ]

    manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation_safe',
        output='screen',
        parameters=[{
            'autostart': True,
            'bond_timeout': 4.0,
            'node_names': lifecycle_nodes,
        }],
    )

    return LaunchDescription([
        LogInfo(
            msg='NAVIGATION SAFE: output /cmd_vel_nav_test'
        ),
        *nodes,
        TimerAction(period=2.0, actions=[manager]),
    ])
