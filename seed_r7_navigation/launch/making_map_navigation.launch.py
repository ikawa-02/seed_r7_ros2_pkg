"""
making_map_navigation.launch.py
slam_toolbox によるマップ作成 (gmapping.launch の ROS2 版)
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    nav_share   = get_package_share_directory('seed_r7_navigation')
    slam_share  = get_package_share_directory('slam_toolbox')

    slam_params_arg = DeclareLaunchArgument(
        'slam_params_file',
        default_value=os.path.join(nav_share, 'config', 'slam_toolbox_params.yaml'),
        description='Full path to slam_toolbox params yaml',
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='false')

    slam_params_file = LaunchConfiguration('slam_params_file')
    use_sim_time     = LaunchConfiguration('use_sim_time')

    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_share, 'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'slam_params_file': slam_params_file,
            'use_sim_time':     use_sim_time,
        }.items(),
    )

    return LaunchDescription([
        slam_params_arg,
        use_sim_time_arg,
        slam_toolbox_launch,
    ])
