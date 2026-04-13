"""
move_group.launch.py  —  MoveIt2 move_group node for SEED-Noid-Mover-typeG
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

    desc_share = get_package_share_directory('seed_r7_description')
    xacro_file = os.path.join(desc_share, robot_model, 'noid.urdf.xacro')

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
            'SEED-Noid-Mover-typeG',
            package_name='seed_r7_typeg_moveit_config',
        )
        .robot_description(file_path=xacro_file, mappings=xacro_mappings)
        .robot_description_semantic(file_path='config/SEED-Noid-Mover-typeG.srdf')
        .robot_description_kinematics(file_path='config/kinematics.yaml')
        .joint_limits(file_path='config/joint_limits.yaml')
        .planning_pipelines(pipelines=['ompl', 'pilz_industrial_motion_planner'])
        .trajectory_execution(file_path='config/moveit_controllers.yaml')
        .to_moveit_configs()
    )

    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[moveit_config.to_dict()],
    )

    return [move_group_node]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot_model',     default_value='typeg'),
        DeclareLaunchArgument('controller_rate', default_value='50'),
        DeclareLaunchArgument('overlap_scale',   default_value='2.0'),
        DeclareLaunchArgument('port_lower',      default_value='/dev/aero_lower'),
        DeclareLaunchArgument('port_upper',      default_value='/dev/aero_upper'),
        OpaqueFunction(function=launch_setup),
    ])
