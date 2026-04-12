"""
map_saver.launch.py  —  現在の地図を保存する (map_saver.launch の ROS2 版)
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    nav_share = get_package_share_directory('seed_r7_navigation')

    map_name_arg = DeclareLaunchArgument(
        'map_name',
        default_value=os.path.join(nav_share, 'maps', 'map'),
        description='Output map file path (without extension)',
    )

    map_saver_node = Node(
        package='nav2_map_server',
        executable='map_saver_cli',
        output='screen',
        arguments=['-f', LaunchConfiguration('map_name')],
        parameters=[{'use_sim_time': False}],
    )

    return LaunchDescription([
        map_name_arg,
        map_saver_node,
    ])
