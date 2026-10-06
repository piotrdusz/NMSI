import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# Robot spawn poses (x, y, yaw) as defined in each world file
INITIAL_POSES = {
    'cave': (-7.0, -7.0, 0.7854),
    'hallway': (-3.0, 0.0, 0.0),
    'lines': (0.0, 0.0, 0.7854),
}


def launch_setup(context):
    package_path = get_package_share_directory("nmsi_lab")
    world = LaunchConfiguration('world').perform(context)
    config_path = os.path.join(package_path, "config", "nav_config.yaml")
    map_config_path = os.path.join(package_path, "config", "maps", world + ".yaml")
    world_path = os.path.join(package_path, "world", world + ".world")
    x, y, yaw = INITIAL_POSES[world]
    rviz_path = os.path.join(package_path, "config", "rviz", "simulation.rviz")

    return [
        Node(
            package='stage_ros2',
            executable='stage_ros2',
            name='stage',
            output='screen',
            parameters=[{'world_file': world_path}]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_path],
            output='screen'
        ),
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[config_path, {'yaml_filename': map_config_path}]
        ),
        Node(
            package='nmsi_lab',
            executable='map_processor',
            name='map_processor',
            output='screen',
            parameters=[{'robot_radius': 0.25, 'max_cost_distance': 1.0}]
        ),
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[config_path, {
                'set_initial_pose': True,
                'initial_pose.x': x,
                'initial_pose.y': y,
                'initial_pose.z': 0.0,
                'initial_pose.yaw': yaw}]
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager',
            output='screen',
            parameters=[
                {'autostart': True},
                {'node_names': ['map_server', 'amcl']}
            ]
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'world', default_value='cave',
            description='World name: cave, hallway or lines'),
        OpaqueFunction(function=launch_setup),
    ])
