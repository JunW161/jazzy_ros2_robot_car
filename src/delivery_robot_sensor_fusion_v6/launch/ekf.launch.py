from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from pathlib import Path


def generate_launch_description():
    yaml = str(Path(get_package_share_directory('delivery_robot_sensor_fusion_v6')) / 'config' / 'ekf.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('publish_tf', default_value='false'),
        Node(package='tf2_ros', executable='static_transform_publisher', name='imu_mount_tf_v6',
             arguments=['--x','0.0','--y','0.0','--z','0.065','--roll','0','--pitch','0','--yaw','0',
                        '--frame-id','base_link','--child-frame-id','imu_link'],
             output='screen'),
        Node(package='robot_localization', executable='ekf_node', name='ekf_filter_node',
             parameters=[yaml, {'publish_tf': LaunchConfiguration('publish_tf')}], output='screen')
    ])
