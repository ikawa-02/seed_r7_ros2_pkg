"""
moveit_rviz.launch.py  —  RViz2 with MoveIt2 MotionPlanning plugin for SEED-Arm-Mover-typeG2
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def launch_setup(context, *args, **kwargs):
    robot_model     = LaunchConfiguration('robot_model').perform(context)
    controller_rate = LaunchConfiguration('controller_rate').perform(context)
    overlap_scale   = LaunchConfiguration('overlap_scale').perform(context)
    port_lower      = LaunchConfiguration('port_lower').perform(context)
    port_upper      = LaunchConfiguration('port_upper').perform(context)
    rviz_config     = LaunchConfiguration('rviz_config').perform(context)

    desc_share   = get_package_share_directory('seed_r7_description')
    moveit_share = get_package_share_directory('seed_r7_typeg2_arm_moveit_config')

    xacro_file = os.path.join(desc_share, robot_model, 'arm.urdf.xacro')

    xacro_mappings = {
        'port_upper':          port_upper,
        'port_lower':          port_lower,
        'robot_model_plugin':  f'seed_r7_robot_interface/{robot_model}',
        'controller_rate':     controller_rate,
        'overlap_scale':       overlap_scale,
        'joint_settings_file': '',
    }

    moveit_config = (
        MoveItConfigsBuilder(
            'SEED-Arm-Mover-typeG2',
            package_name='seed_r7_typeg2_arm_moveit_config',
        )
        .robot_description(file_path=xacro_file, mappings=xacro_mappings)
        .robot_description_semantic(file_path='config/SEED-Arm-Mover-typeG2.srdf')
        .robot_description_kinematics(file_path='config/kinematics.yaml')
        .planning_pipelines(pipelines=['ompl', 'pilz_industrial_motion_planner'])
        .to_moveit_configs()
    )

    rviz_config_file = rviz_config if rviz_config else \
        os.path.join(moveit_share, 'launch', 'moveit.rviz')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[moveit_config.to_dict()],
    )

    return [rviz_node]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot_model',     default_value='typeg2_arm'),
        DeclareLaunchArgument('controller_rate', default_value='50'),
        DeclareLaunchArgument('overlap_scale',   default_value='2.0'),
        DeclareLaunchArgument('port_lower',      default_value='/dev/aero_lower'),
        DeclareLaunchArgument('port_upper',      default_value='/dev/aero_upper'),
        DeclareLaunchArgument('rviz_config',     default_value='',
                              description='Path to RViz config file (default: moveit.rviz)'),
        OpaqueFunction(function=launch_setup),
    ])
