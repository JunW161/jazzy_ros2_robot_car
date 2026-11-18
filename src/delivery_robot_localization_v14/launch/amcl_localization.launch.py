from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _nodes(context):
    map_file = LaunchConfiguration('map').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'
    if not map_file:
        raise RuntimeError('AMCL requires map:=/absolute/path/to/map.yaml')
    map_path = Path(map_file).expanduser().resolve()
    if not map_path.is_file() or map_path.suffix.lower() != '.yaml':
        raise RuntimeError(f'Map YAML does not exist: {map_path}')

    amcl_config = str(Path(get_package_share_directory('delivery_robot_localization_v14')) / 'config' / 'amcl.yaml')
    return [
        Node(
            package='nav2_map_server', executable='map_server', name='map_server', output='screen',
            parameters=[{'yaml_filename': str(map_path), 'topic_name': 'map', 'frame_id': 'map', 'use_sim_time': use_sim_time}],
        ),
        Node(
            package='nav2_amcl', executable='amcl', name='amcl', output='screen',
            parameters=[amcl_config, {'use_sim_time': use_sim_time}],
        ),
        Node(
            package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_localization', output='screen',
            parameters=[{'use_sim_time': use_sim_time, 'autostart': True, 'node_names': ['map_server', 'amcl']}],
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('map', default_value='', description='Absolute path to saved map YAML.'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        OpaqueFunction(function=_nodes),
    ])
