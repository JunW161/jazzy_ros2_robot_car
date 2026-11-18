import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, LogWarning, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    share = get_package_share_directory('delivery_robot_nav2')
    params = os.path.join(share, 'config', 'controller_v1.yaml')
    controller = Node(
        package='nav2_controller', executable='controller_server',
        name='controller_server', output='screen',
        parameters=[params], remappings=[('cmd_vel', '/cmd_vel')],
    )
    manager = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_controller', output='screen',
        parameters=[{'autostart': True, 'bond_timeout': 4.0,
                     'node_names': ['controller_server']}],
    )
    return LaunchDescription([
        DeclareLaunchArgument('enable_direct', default_value='false',
                              description='Explicitly enable direct controller output to /cmd_vel'),
        LogWarning(msg='DIRECT CONTROLLER bypasses Velocity Smoother and Collision Monitor; prefer integrated Mux'),
        GroupAction(
            condition=IfCondition(LaunchConfiguration('enable_direct')),
            actions=[controller, TimerAction(period=1.0, actions=[manager])],
        ),
    ])
