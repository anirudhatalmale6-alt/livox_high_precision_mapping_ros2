# Field bringup - ONE command starts everything for a screenless field unit:
#   1. the Livox Avia LiDAR driver
#   2. the sensors + RTK bringup (UM982 + NTRIP + IMU adapter)
#   3. the web dashboard (+ GPIO button + RGB LED)
#
# Pair this with the systemd service (mapper-field.service) so it all comes up
# automatically when the Pi powers on - then you just wait ~30 s, and drive it
# from the browser or the button. No terminals.
#
#   ros2 launch mapper_web field.launch.py \
#       ntrip_host:=... ntrip_mountpoint:=... ntrip_user:=... ntrip_password:=...
#
# The LiDAR driver include is best-effort: if livox_ros2_driver isn't found (or
# you launch it yourself), pass start_lidar_driver:=false.
import os

from ament_index_python.packages import (get_package_share_directory,
                                          PackageNotFoundError)
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            LogInfo)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    args = [
        DeclareLaunchArgument('start_lidar_driver', default_value='true'),
        # RTK / NTRIP (fill these in via the systemd env file - see field.env)
        DeclareLaunchArgument('rtcm_source', default_value='ntrip'),
        DeclareLaunchArgument('ntrip_host', default_value=''),
        DeclareLaunchArgument('ntrip_port', default_value='2101'),
        DeclareLaunchArgument('ntrip_mountpoint', default_value=''),
        # Default the login from the environment (systemd sets these from
        # /etc/mapper/field.env). Passing them on the command line instead put
        # the password in the process list, where `ps` and `systemctl status`
        # showed it in the clear to every user on the machine - and into any
        # log or diagnostic anyone pasted. Taken from the environment they stay
        # out of the command line. An explicit launch argument still wins.
        DeclareLaunchArgument('ntrip_user',
                              default_value=os.environ.get('NTRIP_USER', '')),
        DeclareLaunchArgument('ntrip_password',
                              default_value=os.environ.get('NTRIP_PASS', '')),
        DeclareLaunchArgument('use_gnss_heading', default_value='true'),
        # dashboard
        DeclareLaunchArgument('port', default_value='8080'),
        DeclareLaunchArgument('button_gpio', default_value='26'),
        DeclareLaunchArgument('led_red', default_value='16'),
        DeclareLaunchArgument('led_green', default_value='20'),
        DeclareLaunchArgument('led_blue', default_value='21'),
        DeclareLaunchArgument('require_rtk', default_value='false'),
        DeclareLaunchArgument('on_fail', default_value='wait'),
    ]
    actions = list(args)

    # 1. Livox LiDAR driver (best-effort - skip cleanly if not installed).
    try:
        livox_launch = os.path.join(
            get_package_share_directory('livox_ros2_driver'),
            'launch', 'livox_lidar_launch.py')
        if os.path.isfile(livox_launch):
            actions.append(IncludeLaunchDescription(
                PythonLaunchDescriptionSource(livox_launch),
                condition=IfCondition(LaunchConfiguration('start_lidar_driver'))))
        else:
            actions.append(LogInfo(msg='[field] livox_lidar_launch.py not found '
                                       '- start the LiDAR driver yourself.'))
    except PackageNotFoundError:
        actions.append(LogInfo(msg='[field] livox_ros2_driver not installed '
                                   '- start the LiDAR driver yourself.'))

    # 2. Sensors + RTK bringup (UM982 + NTRIP + IMU adapter).
    # Also best-effort. If this package is missing (e.g. a build that ran out of
    # memory part-way on a Pi), a hard include would abort the WHOLE launch and
    # take the dashboard down with it - which reads as "the web page is gone"
    # when the real fault is a half-built workspace. The dashboard must always
    # come up, because it is how you see what is wrong.
    try:
        sensors_launch = os.path.join(
            get_package_share_directory('livox_hp_mapping_bringup'),
            'launch', 'sensors.launch.py')
        if os.path.isfile(sensors_launch):
            actions.append(IncludeLaunchDescription(
                PythonLaunchDescriptionSource(sensors_launch),
                launch_arguments={
                    'rtcm_source': LaunchConfiguration('rtcm_source'),
                    'ntrip_host': LaunchConfiguration('ntrip_host'),
                    'ntrip_port': LaunchConfiguration('ntrip_port'),
                    'ntrip_mountpoint': LaunchConfiguration('ntrip_mountpoint'),
                    'ntrip_user': LaunchConfiguration('ntrip_user'),
                    'ntrip_password': LaunchConfiguration('ntrip_password'),
                    'use_gnss_heading': LaunchConfiguration('use_gnss_heading'),
                }.items()))
        else:
            actions.append(LogInfo(msg='[field] sensors.launch.py not found - '
                                       'GNSS/IMU will not start. Re-run '
                                       'scripts/update_field_unit.sh.'))
    except PackageNotFoundError:
        actions.append(LogInfo(msg='[field] livox_hp_mapping_bringup not built - '
                                   'GNSS/IMU will not start. Re-run '
                                   'scripts/update_field_unit.sh.'))

    # 3. The dashboard (serves the page, GPIO button, RGB LED, logging).
    actions.append(Node(
        package='mapper_web', executable='mapper_web', name='mapper_web',
        output='screen',
        arguments=[
            '--port', LaunchConfiguration('port'),
            '--button-gpio', LaunchConfiguration('button_gpio'),
            '--led-red', LaunchConfiguration('led_red'),
            '--led-green', LaunchConfiguration('led_green'),
            '--led-blue', LaunchConfiguration('led_blue'),
            '--on-fail', LaunchConfiguration('on_fail'),
        ]))

    return LaunchDescription(actions)
