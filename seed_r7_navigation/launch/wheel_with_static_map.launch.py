"""
wheel_with_static_map.launch.py  (ROS2 Jazzy)

既存マップを使ったホイール走行 + 自律ナビゲーション。
ROS1 wheel_with_static_map.launch の移植。

構成:
  1. (RUN_BRINGUP=true の場合) wheel_bringup.launch.py
       → URG レーザー + ジョイスティック
  2. static_map_navigation.launch.py
       → Nav2 (AMCL + BT Navigator + Controller + Planner ...)

変更点:
  - <include file="...wheel_bringup.launch"> → IncludeLaunchDescription (条件付き)
  - <include file="...static_map_navigation.launch"> → IncludeLaunchDescription
  - map_localization_file / map_keepout_file の引数は Nav2 の 'map' 引数に統合

依存パッケージ (要インストールの場合):
  sudo apt install ros-jazzy-urg-node
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                             OpaqueFunction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def launch_setup(context, *args, **kwargs):
    nav_share = get_package_share_directory('seed_r7_navigation')

    run_bringup      = LaunchConfiguration('run_bringup').perform(context)
    map_file         = LaunchConfiguration('map').perform(context)
    params_file      = LaunchConfiguration('params_file').perform(context)
    use_sim_time     = LaunchConfiguration('use_sim_time').perform(context)
    cmd_vel_topic    = LaunchConfiguration('cmd_vel_topic').perform(context)
    ip_address       = LaunchConfiguration('ip_address').perform(context)
    ip_address_rear  = LaunchConfiguration('ip_address_rear').perform(context)
    use_rear_laser   = LaunchConfiguration('use_rear_laser').perform(context)

    actions = []

    # ---- 1. wheel bringup (URG + joy) ----
    if run_bringup.lower() == 'true':
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav_share, 'launch', 'wheel_bringup.launch.py')
                ),
                launch_arguments={
                    'ip_address':      ip_address,
                    'ip_address_rear': ip_address_rear,
                    'use_rear_laser':  use_rear_laser,
                }.items(),
            )
        )

    # ---- 2. Nav2 static map navigation ----
    actions.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav_share, 'launch', 'static_map_navigation.launch.py')
            ),
            launch_arguments={
                'map':          map_file,
                'params_file':  params_file,
                'use_sim_time': use_sim_time,
                'cmd_vel_topic': cmd_vel_topic,
            }.items(),
        )
    )

    return actions


def generate_launch_description():
    nav_share = get_package_share_directory('seed_r7_navigation')

    return LaunchDescription([
        DeclareLaunchArgument(
            'run_bringup',
            default_value='true',
            description='If true, launch URG laser + joystick nodes',
        ),
        DeclareLaunchArgument(
            'map',
            default_value=os.path.join(nav_share, 'maps', 'map.yaml'),
            description='Full path to the localization map yaml',
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(nav_share, 'config', 'nav2_params.yaml'),
            description='Full path to Nav2 params yaml',
        ),
        DeclareLaunchArgument(
            'cmd_vel_topic',
            default_value='cmd_vel',
            description='cmd_vel output topic for Nav2',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
        ),
        DeclareLaunchArgument(
            'ip_address',
            default_value='192.168.0.10',
            description='IP address of the front URG laser (used when run_bringup:=true)',
        ),
        DeclareLaunchArgument(
            'ip_address_rear',
            default_value='192.168.0.11',
            description='IP address of the rear URG laser (used when use_rear_laser:=true)',
        ),
        DeclareLaunchArgument(
            'use_rear_laser',
            default_value='false',
            description='Launch a second URG for rear scan',
        ),
        OpaqueFunction(function=launch_setup),
    ])
