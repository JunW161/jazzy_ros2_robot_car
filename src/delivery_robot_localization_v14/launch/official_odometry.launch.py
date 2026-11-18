import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    home = Path.home()
    bridge = home / 'micro_ros_ws/micro_ros_espidf_component/examples/robot_motor_control/host_ros2/odom_tf_joint_bridge.py'
    v14_launch = Path(get_package_share_directory('delivery_robot_sensor_fusion_v14')) / 'launch' / 'ekf_compare.launch.py'

    start_bridge = LaunchConfiguration('start_bridge')
    use_sim_time = LaunchConfiguration('use_sim_time')
    publish_imu_tf = LaunchConfiguration('publish_imu_tf')
    imu_x = LaunchConfiguration('imu_x'); imu_y = LaunchConfiguration('imu_y'); imu_z = LaunchConfiguration('imu_z')
    imu_roll = LaunchConfiguration('imu_roll'); imu_pitch = LaunchConfiguration('imu_pitch'); imu_yaw = LaunchConfiguration('imu_yaw')

    return LaunchDescription([
        DeclareLaunchArgument('start_bridge', default_value='true', description='Start odom/joint bridge with publish_tf=false.'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('publish_imu_tf', default_value='true'),
        DeclareLaunchArgument('imu_x', default_value='0.0'),
        DeclareLaunchArgument('imu_y', default_value='0.0'),
        DeclareLaunchArgument('imu_z', default_value='0.065'),
        DeclareLaunchArgument('imu_roll', default_value='0.0'),
        DeclareLaunchArgument('imu_pitch', default_value='0.0'),
        DeclareLaunchArgument('imu_yaw', default_value='0.0'),

        ExecuteProcess(
            cmd=[
                '/usr/bin/python3', str(bridge), '--ros-args',
                '-p', 'left_joint:=wheel_joint_l',
                '-p', 'right_joint:=wheel_joint_r',
                '-p', 'publish_tf:=false',
                '-p', 'publish_joint_states:=true',
            ],
            name='delivery_odom_joint_bridge_no_tf',
            output='screen',
            condition=IfCondition(start_bridge),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(v14_launch)),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'publish_tf': 'true',
                'output_topic': '/odometry/filtered',
                'publish_imu_tf': publish_imu_tf,
                'imu_x': imu_x, 'imu_y': imu_y, 'imu_z': imu_z,
                'imu_roll': imu_roll, 'imu_pitch': imu_pitch, 'imu_yaw': imu_yaw,
            }.items(),
        ),
    ])
