# Launch the mapper web dashboard.
#
#   ros2 launch mapper_web dashboard.launch.py
#   ros2 launch mapper_web dashboard.launch.py led_gpio:=19 on_fail:=abort
#
# Then open http://<pi-ip>:8080 from any phone or laptop on the same network.
# The Livox LiDAR driver and the sensors bringup are started as usual - this
# dashboard drives the logging on top of them.
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    args = [
        DeclareLaunchArgument('port', default_value='8080'),
        DeclareLaunchArgument('mount_point', default_value='/media/log'),
        DeclareLaunchArgument('button_gpio', default_value='26'),
        DeclareLaunchArgument('led_gpio', default_value=''),
        DeclareLaunchArgument('workspace', default_value='/opt/mapper/ws'),
        DeclareLaunchArgument('require_rtk', default_value='true'),
        DeclareLaunchArgument('on_fail', default_value='wait'),
        DeclareLaunchArgument('simulate', default_value='false'),
    ]

    argv = [
        '--port', LaunchConfiguration('port'),
        '--mount-point', LaunchConfiguration('mount_point'),
        '--button-gpio', LaunchConfiguration('button_gpio'),
        '--workspace', LaunchConfiguration('workspace'),
        '--on-fail', LaunchConfiguration('on_fail'),
    ]

    node = Node(
        package='mapper_web',
        executable='mapper_web',
        name='mapper_web',
        output='screen',
        arguments=argv,
    )
    return LaunchDescription(args + [node])
