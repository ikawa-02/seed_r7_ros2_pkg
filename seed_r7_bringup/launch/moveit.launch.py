"""
moveit.launch.py
ROS1 moveit.launch を ROS2 Jazzy 向けに移植。

主な変更点:
- <include file="$(find ...)"> -> IncludeLaunchDescription + PythonLaunchDescriptionSource
- $(find seed_r7_moveit_config)/../seed_r7_$(arg robot_model)_moveit_config
  -> get_package_share_directory(f'seed_r7_{robot_model}_moveit_config')
- moveit_rviz.launch の rviz_config 引数もそのまま引き継ぎ

TODO: seed_r7_typef_moveit_config / seed_r7_typeg_moveit_config の
      ROS2 移植が完了してから本ファイルのテストを行うこと。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def launch_setup(context, *args, **kwargs):
    robot_model = LaunchConfiguration('robot_model').perform(context)

    bringup_share = get_package_share_directory('seed_r7_bringup')

    # MoveIt2 config パッケージ名: seed_r7_typef_moveit_config 等
    moveit_config_pkg = f'seed_r7_{robot_model}_moveit_config'
    try:
        moveit_config_share = get_package_share_directory(moveit_config_pkg)
    except Exception:
        raise RuntimeError(
            f'[moveit.launch.py] MoveIt2 config package not found: {moveit_config_pkg}\n'
            f'Please migrate {moveit_config_pkg} to ROS2 first.'
        )

    return [
        # seed_r7_bringup.launch.py をインクルード
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup_share, 'launch', 'seed_r7_bringup.launch.py')
            ),
            launch_arguments={'robot_model': robot_model}.items(),
        ),

        # MoveIt2 move_group
        # ROS1: move_group.launch -> ROS2: move_group.launch.py
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(moveit_config_share, 'launch', 'move_group.launch.py')
            ),
        ),

        # MoveIt2 RViz
        # ROS1: moveit_rviz.launch -> ROS2: moveit_rviz.launch.py
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(moveit_config_share, 'launch', 'moveit_rviz.launch.py')
            ),
            launch_arguments={
                'rviz_config': os.path.join(
                    moveit_config_share, 'launch', 'moveit.rviz'
                )
            }.items(),
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_model',
            default_value='typef',
            description='Robot model type: typef / typeg / typearm / etc.',
        ),
        OpaqueFunction(function=launch_setup),
    ])
