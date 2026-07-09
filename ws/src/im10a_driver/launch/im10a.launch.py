import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory('im10a_driver'), 'config', 'im10a.yaml')

    params_arg = DeclareLaunchArgument(
        'params_file', default_value=default_params,
        description='Path to the IM10A driver parameter file.')

    return LaunchDescription([
        params_arg,
        Node(
            package='im10a_driver',
            executable='im10a_driver_node',
            name='im10a_driver',
            output='screen',
            parameters=[LaunchConfiguration('params_file')],
        ),
    ])
