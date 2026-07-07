from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('input_imu_topic', default_value='/imu/data'),
        DeclareLaunchArgument('use_gnss_heading', default_value='false'),
        DeclareLaunchArgument('heading_offset_deg', default_value='0.0'),
        Node(
            package='imu_gnss_adapter',
            executable='imu_gnss_adapter_node',
            name='imu_gnss_adapter',
            output='screen',
            parameters=[{
                'input_imu_topic': LaunchConfiguration('input_imu_topic'),
                'use_gnss_heading': LaunchConfiguration('use_gnss_heading'),
                'gnss_heading_topic': '/um982_driver/heading',
                'heading_offset_deg': LaunchConfiguration('heading_offset_deg'),
                'output_frame_id': 'imu',
            }],
        ),
    ])
