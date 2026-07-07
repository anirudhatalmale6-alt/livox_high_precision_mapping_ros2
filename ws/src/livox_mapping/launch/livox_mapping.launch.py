import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('livox_mapping')
    rviz_cfg = os.path.join(pkg, 'rviz', 'livox_mapping.rviz')

    rviz_arg = DeclareLaunchArgument('rviz', default_value='true')
    map_path_arg = DeclareLaunchArgument(
        'map_file_path', default_value='',
        description='Directory to write all_points.pcd on shutdown (empty = cwd).')

    mapping_node = Node(
        package='livox_mapping',
        executable='livox_mapping_node',
        name='livox_mapping',
        output='screen',
        parameters=[{
            'lidar_delta_time': 0.01,
            'map_file_path': LaunchConfiguration('map_file_path'),
            'save_pcd': True,
        }],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_cfg],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return LaunchDescription([rviz_arg, map_path_arg, mapping_node, rviz_node])
