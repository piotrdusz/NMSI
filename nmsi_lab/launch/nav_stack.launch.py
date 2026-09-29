import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    package_path = get_package_share_directory("nmsi_lab")
    config_file_name = 'nav_config.yaml'
    config_path = os.path.join(package_path, "config", config_file_name)
    map_config_path = os.path.join(package_path, "config", "map.yaml")
    
    lifecycle_nodes = ['map_server', 'amcl', 'controller_server']

    return LaunchDescription([
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[
                config_path,
                {'yaml_filename': map_config_path}]
        ),
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[config_path]
        ),
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[config_path]
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager',
            output='screen',
            parameters=[
                {'autostart': True},
                {'node_names': lifecycle_nodes}
            ]
        )
    ])