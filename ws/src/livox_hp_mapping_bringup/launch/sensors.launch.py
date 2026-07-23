# Sensors + RTK bringup — everything EXCEPT the mapping node.
#
# Starts:
#   1. IM10A IMU driver                (this workspace)
#   2. UM982 RTK GNSS driver           (this workspace, incl. NTRIP/RTCM)
#   3. IM10A -> /gnss_inertial/imu adapter (this workspace)
#
# Run this FIRST, in its own terminal, and leave it running. Wait until the
# GPS has an RTK fix (navsatfix status: 2) before you start the mapping. Then
# launch mapping.launch.py in a second terminal when you're ready to record.
#
# The Livox LiDAR driver is started separately (its own launch), same as before.
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def sensor_arguments():
    """Launch args shared by the sensors bringup (reused by mapping_online)."""
    return [
        DeclareLaunchArgument('um982_port', default_value='/dev/gps'),
        DeclareLaunchArgument('um982_baud', default_value='115200'),
        DeclareLaunchArgument('im10a_port', default_value='/dev/imu'),
        DeclareLaunchArgument('im10a_baud', default_value='115200'),
        # Default rig uses the Avia's BUILT-IN IMU (/livox/imu), so the separate
        # IM10A is OFF by default. To use an external IM10A instead, pass
        #   start_im10a:=true imu_input_topic:=/imu/data
        DeclareLaunchArgument('start_im10a', default_value='false'),
        DeclareLaunchArgument('imu_input_topic', default_value='/livox/imu'),
        DeclareLaunchArgument('use_gnss_heading', default_value='true'),
        DeclareLaunchArgument('heading_offset_deg', default_value='0.0'),
        # Auto-ask the UM982 to stream its dual-antenna heading on connect.
        DeclareLaunchArgument('configure_heading', default_value='true'),
        # Stamp GPS data in satellite (GPS/UTC) time and publish the clock
        # offset so the mapper can put LiDAR/IMU on the same clock. Off by
        # default. If you set this true, also pass use_gps_time:=true to
        # mapping.launch.py in the other terminal.
        DeclareLaunchArgument('gps_time_sync', default_value='false'),
        # RTK corrections. rtcm_source: none | ntrip | serial | mavlink.
        DeclareLaunchArgument('rtcm_source', default_value='none'),
        DeclareLaunchArgument('ntrip_host', default_value=''),
        DeclareLaunchArgument('ntrip_port', default_value='2101'),
        DeclareLaunchArgument('ntrip_mountpoint', default_value=''),
        DeclareLaunchArgument('ntrip_user', default_value=''),
        DeclareLaunchArgument('ntrip_password', default_value=''),
        DeclareLaunchArgument('rtcm_serial_port', default_value='/dev/rtcm'),
        DeclareLaunchArgument('rtcm_serial_baud', default_value='57600'),
        DeclareLaunchArgument('mavlink_serial_port', default_value='/dev/pixhawk'),
        DeclareLaunchArgument('mavlink_serial_baud', default_value='57600'),
    ]


def sensor_nodes():
    """The IM10A, UM982 and IMU-adapter nodes (reused by mapping_online)."""
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
            'configure_heading': LaunchConfiguration('configure_heading'),
            'gps_time_sync': LaunchConfiguration('gps_time_sync'),
            'rtcm_source': LaunchConfiguration('rtcm_source'),
            'ntrip_host': LaunchConfiguration('ntrip_host'),
            'ntrip_port': ParameterValue(LaunchConfiguration('ntrip_port'), value_type=int),
            'ntrip_mountpoint': LaunchConfiguration('ntrip_mountpoint'),
            'ntrip_user': LaunchConfiguration('ntrip_user'),
            'ntrip_password': LaunchConfiguration('ntrip_password'),
            'rtcm_serial_port': LaunchConfiguration('rtcm_serial_port'),
            'rtcm_serial_baud': ParameterValue(
                LaunchConfiguration('rtcm_serial_baud'), value_type=int),
            'mavlink_serial_port': LaunchConfiguration('mavlink_serial_port'),
            'mavlink_serial_baud': ParameterValue(
                LaunchConfiguration('mavlink_serial_baud'), value_type=int),
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

    return [im10a, um982, imu_adapter]


def generate_launch_description():
    return LaunchDescription(sensor_arguments() + sensor_nodes())
