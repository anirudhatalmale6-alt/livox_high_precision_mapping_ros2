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
    min_range_arg = DeclareLaunchArgument(
        'min_range', default_value='0.5',
        description='Drop returns closer than this (m). The Avia reports a '
                    'no-return as (0,0,0), which the pose transform then places '
                    'on the scanner itself. 0 disables the filter.')
    max_points_arg = DeclareLaunchArgument(
        'max_points', default_value='60000000',
        description='Ceiling on the accumulated map (points). The map only '
                    'grows; at 32 bytes a point the Avia fills 6 GB in about '
                    'ten minutes, faster in dual/triple return. 0 disables.')
    gps_time_arg = DeclareLaunchArgument(
        'use_gps_time', default_value='false',
        description='Shift LiDAR/IMU stamps onto satellite time using the offset '
                    'the UM982 driver publishes (needs gps_time_sync:=true there).')

    mapping_node = Node(
        package='livox_mapping',
        executable='livox_mapping_node',
        name='livox_mapping',
        output='screen',
        # Give the map time to be written before anything kills this.
        #
        # ros2 launch defaults to SIGINT, then SIGTERM 5 s later, then SIGKILL
        # 10 s after that. Saving the cloud on shutdown means writing tens of
        # megabytes to a USB stick, which on this hardware takes longer than
        # that - so the client's runs were repeatedly ending in
        #   process[livox_mapping_node-1] failed to terminate '10.0' seconds
        #   after receiving 'SIGTERM', escalating to 'SIGKILL'
        #   process has died [pid ..., exit code -9]
        # A SIGKILL mid-write is the one thing the atomic .tmp-then-rename in
        # saveMap() is there to survive, so no file was corrupted - but the
        # final save was lost and the run fell back to the last autosave.
        sigterm_timeout='45',
        sigkill_timeout='45',
        parameters=[{
            'lidar_delta_time': ParameterValue(
                LaunchConfiguration('lidar_delta_time'), value_type=float),
            'map_file_path': LaunchConfiguration('map_file_path'),
            'save_pcd': True,
            'autosave_sec': ParameterValue(
                LaunchConfiguration('autosave_sec'), value_type=float),
            'use_gps_time': ParameterValue(
                LaunchConfiguration('use_gps_time'), value_type=bool),
            'min_range': ParameterValue(
                LaunchConfiguration('min_range'), value_type=float),
            'max_points': ParameterValue(
                LaunchConfiguration('max_points'), value_type=int),
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
        [rviz_arg, map_path_arg, delta_arg, autosave_arg, min_range_arg,
         max_points_arg, gps_time_arg, mapping_node, rviz_node])
