# Mapping-only bringup — just the livox_mapping node (+ optional RViz).
#
# Run this in a SECOND terminal AFTER sensors.launch.py is up and the GPS has
# an RTK fix (navsatfix status: 2). It subscribes to the sensor topics and the
# Livox LiDAR, builds the map, and writes a timestamped .pcd on Ctrl-C.
#
# Separating it from the sensors means you can power up, get your RTK lock and
# confirm everything is streaming, and only THEN start recording the map —
# and you can stop/restart the mapping without disturbing the RTK fix.
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def mapping_arguments():
    """Launch args for the mapping side (reused by mapping_online)."""
    return [
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument(
            'map_file_path', default_value='',
            description='Directory to write the timestamped .pcd (empty = cwd).'),
        DeclareLaunchArgument(
            'lidar_delta_time', default_value='0.1',
            description='LiDAR frame period (s). Avia @10Hz = 0.1.'),
        DeclareLaunchArgument(
            'use_gps_time', default_value='false',
            description='Shift LiDAR/IMU onto satellite time (set this true only '
                        'when sensors.launch.py was started with '
                        'gps_time_sync:=true).'),
    ]


def mapping_include():
    """The livox_mapping launch include (reused by mapping_online)."""
    mapping_share = get_package_share_directory('livox_mapping')
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(mapping_share, 'launch', 'livox_mapping.launch.py')),
        launch_arguments={
            'rviz': LaunchConfiguration('rviz'),
            'map_file_path': LaunchConfiguration('map_file_path'),
            'lidar_delta_time': LaunchConfiguration('lidar_delta_time'),
            'use_gps_time': LaunchConfiguration('use_gps_time'),
        }.items(),
    )


def generate_launch_description():
    return LaunchDescription(mapping_arguments() + [mapping_include()])
