# Full online high-precision mapping bringup.
#
# Starts the whole pipeline:
#   1. Livox Mid-40 driver  (livox_ros_driver2 - installed separately)
#   2. IM10A IMU driver      (this workspace - native, no vendor driver needed)
#   3. UM982 RTK driver      (this workspace)
#   4. IM10A -> /gnss_inertial/imu adapter (this workspace)
#   5. livox_mapping node    (this workspace)
#
# The Livox driver is the only external package. The IM10A is handled by our own
# im10a_driver (publishes /imu/data); set start_im10a:=false if you would rather
# run a different IMU source that already publishes on imu_input_topic.
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            GroupAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    um982_share = get_package_share_directory('um982_driver')
    mapping_share = get_package_share_directory('livox_mapping')

    args = [
        DeclareLaunchArgument('um982_port', default_value='/dev/gps'),
        DeclareLaunchArgument('um982_baud', default_value='115200'),
        DeclareLaunchArgument('im10a_port', default_value='/dev/imu'),
        DeclareLaunchArgument('im10a_baud', default_value='115200'),
        DeclareLaunchArgument('start_im10a', default_value='true'),
        DeclareLaunchArgument('imu_input_topic', default_value='/imu/data'),
        DeclareLaunchArgument('use_gnss_heading', default_value='true'),
        DeclareLaunchArgument('heading_offset_deg', default_value='0.0'),
        DeclareLaunchArgument('map_file_path', default_value=''),
        DeclareLaunchArgument('lidar_delta_time', default_value='0.1'),
        DeclareLaunchArgument('rviz', default_value='true'),
    ]

    im10a = Node(
        package='im10a_driver',
        executable='im10a_driver_node',
        name='im10a_driver',
        output='screen',
        condition=IfCondition(LaunchConfiguration('start_im10a')),
        parameters=[{
            'port': LaunchConfiguration('im10a_port'),
            'baud': ParameterValue(LaunchConfiguration('im10a_baud'), value_type=int),
            'frame_id': 'imu',
            'imu_topic': LaunchConfiguration('imu_input_topic'),
        }],
    )

    um982 = Node(
        package='um982_driver',
        executable='um982_driver_node',
        name='um982_driver',
        output='screen',
        parameters=[{
            'port': LaunchConfiguration('um982_port'),
            'baud': ParameterValue(LaunchConfiguration('um982_baud'), value_type=int),
            'frame_id': 'gnss',
            'publish_heading': True,
        }],
    )

    imu_adapter = Node(
        package='imu_gnss_adapter',
        executable='imu_gnss_adapter_node',
        name='imu_gnss_adapter',
        output='screen',
        parameters=[{
            'input_imu_topic': LaunchConfiguration('imu_input_topic'),
            'use_gnss_heading': LaunchConfiguration('use_gnss_heading'),
            'gnss_heading_topic': '/um982_driver/heading',
            'heading_offset_deg': LaunchConfiguration('heading_offset_deg'),
            'output_frame_id': 'imu',
        }],
    )

    mapping = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(mapping_share, 'launch', 'livox_mapping.launch.py')),
        launch_arguments={
            'rviz': LaunchConfiguration('rviz'),
            'map_file_path': LaunchConfiguration('map_file_path'),
            'lidar_delta_time': LaunchConfiguration('lidar_delta_time'),
        }.items(),
    )

    return LaunchDescription(args + [im10a, um982, imu_adapter, mapping])
