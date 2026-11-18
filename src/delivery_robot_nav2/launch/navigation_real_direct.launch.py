import os

from launch import LaunchDescription
from launch.actions import LogInfo, TimerAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def _real_navigation_actions():
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
        Node(
            package='nav2_collision_monitor',
            executable='collision_monitor',
            name='collision_monitor',
            output='screen',
            parameters=[
                params,
                {'cmd_vel_out_topic': '/cmd_vel'}
            ],
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
        name='lifecycle_manager_navigation_mux',
        output='screen',
        parameters=[{
            'autostart': True,
            'bond_timeout': 4.0,
            'node_names': lifecycle_nodes,
        }],
    )

    return LaunchDescription([
        LogInfo(
            msg='NAVIGATION DIRECT: output /cmd_vel'
        ),
        *nodes,
        TimerAction(period=2.0, actions=[manager]),
    ])


from launch.actions import DeclareLaunchArgument, GroupAction, LogWarning
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Direct mode is opt-in and bypasses the command authority mux.
    real = _real_navigation_actions()
    return LaunchDescription([
        DeclareLaunchArgument('enable_direct', default_value='false',
                              description='Explicitly allow direct velocity output to /cmd_vel'),
        LogWarning(msg='DIRECT MODE available only with enable_direct:=true; prefer navigation_integrated_mux.launch.py'),
        GroupAction(
            condition=IfCondition(LaunchConfiguration('enable_direct')),
            actions=list(real.entities),
        ),
    ])
