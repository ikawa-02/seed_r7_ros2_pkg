"""
static_map_navigation.launch.py
既存マップを使った自律ナビゲーション (AMCL + Nav2)
  rosbag:  move_base.launch + amcl.launch  →  Nav2 bringup
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                             IncludeLaunchDescription)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    nav_share  = get_package_share_directory('seed_r7_navigation')
    nav2_share = get_package_share_directory('nav2_bringup')

    # ---- launch arguments ----
    map_file_arg = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(nav_share, 'maps', 'map.yaml'),
        description='Full path to map yaml file',
    )
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(nav_share, 'config', 'nav2_params.yaml'),
        description='Full path to nav2 params yaml',
    )
    cmd_vel_topic_arg = DeclareLaunchArgument(
        'cmd_vel_topic',
        default_value='cmd_vel',
        description='cmd_vel output topic',
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
    )

    map_file    = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # ---- Nav2 bringup (localization + navigation) ----
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_share, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map':           map_file,
            'params_file':   params_file,
            'use_sim_time':  use_sim_time,
        }.items(),
    )

    return LaunchDescription([
        map_file_arg,
        params_file_arg,
        cmd_vel_topic_arg,
        use_sim_time_arg,
        nav2_launch,
    ])
