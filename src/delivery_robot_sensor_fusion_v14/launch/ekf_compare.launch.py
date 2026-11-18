from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = str(Path(get_package_share_directory('delivery_robot_sensor_fusion_v14')) / 'config' / 'ekf_compare.yaml')
    use_sim_time = LaunchConfiguration('use_sim_time')
    publish_tf = LaunchConfiguration('publish_tf')
    output_topic = LaunchConfiguration('output_topic')
    publish_imu_tf = LaunchConfiguration('publish_imu_tf')
    imu_x = LaunchConfiguration('imu_x'); imu_y = LaunchConfiguration('imu_y'); imu_z = LaunchConfiguration('imu_z')
    imu_roll = LaunchConfiguration('imu_roll'); imu_pitch = LaunchConfiguration('imu_pitch'); imu_yaw = LaunchConfiguration('imu_yaw')
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('publish_tf', default_value='false', description='Keep false while odom_tf_joint_bridge.py owns odom->base_link.'),
        DeclareLaunchArgument('output_topic', default_value='/odometry/filtered_v14'),
        DeclareLaunchArgument('publish_imu_tf', default_value='true', description='Current project URDF has no imu_link.'),
        DeclareLaunchArgument('imu_x', default_value='0.0'), DeclareLaunchArgument('imu_y', default_value='0.0'),
        DeclareLaunchArgument('imu_z', default_value='0.065'), DeclareLaunchArgument('imu_roll', default_value='0.0'),
        DeclareLaunchArgument('imu_pitch', default_value='0.0'), DeclareLaunchArgument('imu_yaw', default_value='0.0'),
        Node(package='tf2_ros', executable='static_transform_publisher', name='imu_mount_tf_v14',
             arguments=['--x', imu_x, '--y', imu_y, '--z', imu_z, '--roll', imu_roll, '--pitch', imu_pitch, '--yaw', imu_yaw,
                        '--frame-id', 'base_link', '--child-frame-id', 'imu_link'],
             condition=IfCondition(publish_imu_tf), output='screen'),
        Node(package='robot_localization', executable='ekf_node', name='ekf_filter_node_v14', output='screen',
             parameters=[config, {'use_sim_time': use_sim_time, 'publish_tf': publish_tf}],
             remappings=[('odometry/filtered', output_topic)]),
    ])
