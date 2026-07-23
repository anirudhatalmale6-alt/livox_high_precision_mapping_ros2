import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = get_package_share_directory('livox_mapping')
    rviz_cfg = os.path.join(pkg, 'rviz', 'livox_mapping.rviz')

    rviz_arg = DeclareLaunchArgument('rviz', default_value='true')
    map_path_arg = DeclareLaunchArgument(
        'map_file_path', default_value='',
        description='Directory to write all_points.pcd on shutdown (empty = cwd).')
    delta_arg = DeclareLaunchArgument(
        'lidar_delta_time', default_value='0.1',
        description='LiDAR frame period (s). Avia @10Hz = 0.1; set 1/publish_freq.')
    autosave_arg = DeclareLaunchArgument(
        'autosave_sec', default_value='15.0',
        description='Flush the map to its .pcd every N seconds while running so a '
                    'file always exists even after a hard kill. 0 disables.')
    gps_time_arg = DeclareLaunchArgument(
        'use_gps_time', default_value='false',
        description='Shift LiDAR/IMU stamps onto satellite time using the offset '
                    'the UM982 driver publishes (needs gps_time_sync:=true there).')

    mapping_node = Node(
        package='livox_mapping',
        executable='livox_mapping_node',
        name='livox_mapping',
        output='screen',
        parameters=[{
            'lidar_delta_time': ParameterValue(
                LaunchConfiguration('lidar_delta_time'), value_type=float),
            'map_file_path': LaunchConfiguration('map_file_path'),
            'save_pcd': True,
            'autosave_sec': ParameterValue(
                LaunchConfiguration('autosave_sec'), value_type=float),
            'use_gps_time': ParameterValue(
                LaunchConfiguration('use_gps_time'), value_type=bool),
        }],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_cfg],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return LaunchDescription(
        [rviz_arg, map_path_arg, delta_arg, autosave_arg, gps_time_arg,
         mapping_node, rviz_node])
