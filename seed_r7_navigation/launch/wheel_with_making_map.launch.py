"""
wheel_with_making_map.launch.py  (ROS2 Jazzy)

ホイール走行 + slam_toolbox によるマップ作成。
ROS1 wheel_with_making_map.launch の移植。

構成:
  1. (RUN_BRINGUP=true の場合) wheel_bringup.launch.py
       → URG レーザー + ジョイスティック
  2. making_map_navigation.launch.py
       → slam_toolbox (online async SLAM)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def launch_setup(context, *args, **kwargs):
    nav_share     = get_package_share_directory('seed_r7_navigation')
    run_bringup   = LaunchConfiguration('run_bringup').perform(context)
    cmd_vel_topic = LaunchConfiguration('cmd_vel_topic').perform(context)
    use_sim_time  = LaunchConfiguration('use_sim_time').perform(context)

    actions = []

    # ---- 1. wheel bringup (URG + joy) ----
    if run_bringup.lower() == 'true':
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav_share, 'launch', 'wheel_bringup.launch.py')
                ),
            )
        )

    # ---- 2. slam_toolbox によるマップ作成 ----
    actions.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav_share, 'launch', 'making_map_navigation.launch.py')
            ),
            launch_arguments={
                'use_sim_time': use_sim_time,
            }.items(),
        )
    )

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'run_bringup',
            default_value='true',
            description='If true, launch URG laser + joystick nodes',
        ),
        DeclareLaunchArgument(
            'cmd_vel_topic',
            default_value='cmd_vel',
            description='cmd_vel output topic',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
        ),
        OpaqueFunction(function=launch_setup),
    ])
