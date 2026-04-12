#!/usr/bin/env python3
"""Launch SEED-Noid-Mover typeF in Gazebo Sim (Harmonic) empty world."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ros_gz_sim_share = get_package_share_directory('ros_gz_sim')
    description_share = get_package_share_directory('seed_r7_description')
    gazebo_pkg_share  = get_package_share_directory('seed_r7_gazebo')

    # ---- launch arguments ----
    gui_arg  = DeclareLaunchArgument('GUI',    default_value='true')
    world_arg = DeclareLaunchArgument(
        'WORLD_FILE',
        default_value=os.path.join(gazebo_pkg_share, 'worlds', 'empty.world'),
    )
    init_x_arg = DeclareLaunchArgument('init_position_x', default_value='0.0')
    init_y_arg = DeclareLaunchArgument('init_position_y', default_value='0.0')

    gui        = LaunchConfiguration('GUI')
    world_file = LaunchConfiguration('WORLD_FILE')
    init_x     = LaunchConfiguration('init_position_x')
    init_y     = LaunchConfiguration('init_position_y')

    controller_params = os.path.join(
        gazebo_pkg_share, 'config', 'typef', 'gazebo_controller.yaml'
    )
    xacro_file = os.path.join(description_share, 'typef', 'noid.urdf.xacro')

    # ---- Gazebo Sim server ----
    gz_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_share, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': ['-r -s ', world_file],
            'on_exit_shutdown': 'true',
        }.items(),
    )

    # ---- Gazebo Sim GUI (optional) ----
    gz_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_share, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': '-g',
        }.items(),
        condition=IfCondition(gui),
    )

    # ---- Robot description via xacro (with Gazebo hardware) ----
    robot_description_content = Command([
        FindExecutable(name='xacro'), ' ', xacro_file,
        ' use_gazebo:=true',
        ' controller_params_file:=', controller_params,
    ])
    robot_description = {'robot_description': robot_description_content}

    # ---- Robot State Publisher ----
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': True}],
    )

    # ---- Spawn robot in Gazebo Sim ----
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'seed_r7',
            '-topic', 'robot_description',
            '-x', init_x,
            '-y', init_y,
        ],
        output='screen',
    )

    # ---- Bridge: gz-sim <-> ROS2 clock ----
    gz_ros_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen',
    )

    # ---- Controller spawner (delayed to let Gazebo start) ----
    joint_state_broadcaster_spawner = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['joint_state_broadcaster', '--controller-manager-timeout', '30'],
                output='screen',
            )
        ],
    )

    controller_spawner = TimerAction(
        period=7.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=[
                    'head_controller',
                    'waist_controller',
                    'lifter_controller',
                    'larm_controller',
                    'rarm_controller',
                    'lhand_controller',
                    'rhand_controller',
                    '--controller-manager-timeout', '30',
                ],
                output='screen',
            )
        ],
    )

    return LaunchDescription([
        gui_arg,
        world_arg,
        init_x_arg,
        init_y_arg,
        gz_server,
        gz_gui,
        robot_state_publisher,
        spawn_entity,
        gz_ros_bridge,
        joint_state_broadcaster_spawner,
        controller_spawner,
    ])
