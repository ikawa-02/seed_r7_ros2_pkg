"""
wheel_with_dummy.launch.py  (ROS2 Jazzy)

ダミースキャンを使った静的マップナビゲーション。
実機のURGレーザーがなくてもマップ上の障害物情報から疑似LaserScanを生成してNav2を動かす。
ROS1 wheel_with_dummy.launch の移植。

構成:
  1. dummy_scan ノード
       マップ (OccupancyGrid) から疑似 LaserScan を /scan に配信
  2. static_map_navigation.launch.py
       Nav2 (AMCL + BT Navigator + Controller + Planner ...)
       デフォルトマップ: maps/dummy.yaml
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    nav_share     = get_package_share_directory('seed_r7_navigation')
    scan_topic    = LaunchConfiguration('scan_topic').perform(context)
    map_file      = LaunchConfiguration('map').perform(context)
    params_file   = LaunchConfiguration('params_file').perform(context)
    use_sim_time  = LaunchConfiguration('use_sim_time').perform(context)
    cmd_vel_topic = LaunchConfiguration('cmd_vel_topic').perform(context)

    # ---- 1. dummy_scan node ----
    dummy_scan_node = Node(
        package='seed_r7_navigation',
        executable='dummy_scan.py',
        name='dummy_scan',
        output='screen',
        remappings=[('map', scan_topic)],
    )

    # ---- 2. Nav2 static map navigation ----
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav_share, 'launch', 'static_map_navigation.launch.py')
        ),
        launch_arguments={
            'map':           map_file,
            'params_file':   params_file,
            'use_sim_time':  use_sim_time,
            'cmd_vel_topic': cmd_vel_topic,
        }.items(),
    )

    return [dummy_scan_node, nav2_launch]


def generate_launch_description():
    nav_share = get_package_share_directory('seed_r7_navigation')

    return LaunchDescription([
        DeclareLaunchArgument(
            'scan_topic',
            default_value='map',
            description='Topic name that dummy_scan subscribes for OccupancyGrid input',
        ),
        DeclareLaunchArgument(
            'map',
            default_value=os.path.join(nav_share, 'maps', 'dummy.yaml'),
            description='Full path to the map yaml for navigation',
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
        OpaqueFunction(function=launch_setup),
    ])
