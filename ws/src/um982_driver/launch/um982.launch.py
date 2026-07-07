import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory('um982_driver'), 'config', 'um982.yaml')

    params_arg = DeclareLaunchArgument(
        'params_file', default_value=default_params,
        description='Path to the UM982 driver parameter file.')

    return LaunchDescription([
        params_arg,
        Node(
            package='um982_driver',
            executable='um982_driver_node',
            name='um982_driver',
            output='screen',
            parameters=[LaunchConfiguration('params_file')],
        ),
    ])
