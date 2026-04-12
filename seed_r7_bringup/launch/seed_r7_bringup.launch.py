"""
seed_r7_bringup.launch.py  (ROS2 Jazzy)

構成:
  1. xacro で noid.urdf.xacro を処理 → robot_description 文字列を生成
  2. ros2_control_node (controller_manager) を起動
       - robot_description パラメータを渡す
       - ros2_controllers.yaml (コントローラ定義) を渡す
  3. robot_state_publisher を起動
  4. joint_state_broadcaster と各アームコントローラを spawner で起動

元の seed_r7_bringup.launch との主な変更点:
  - seed_r7_ros_controller ノード (存在しない executable) → ros2_control_node
  - planning_context.launch → xacro モジュールで直接 URDF 生成
  - joint_settings は URDF hardware_parameter 経由で hw_node_ に渡す
  - joint_state_controller → joint_state_broadcaster
"""

import os
import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    # --- Launch 引数の取得 ---
    robot_model      = LaunchConfiguration('robot_model').perform(context)
    controller_rate  = LaunchConfiguration('controller_rate').perform(context)
    overlap_scale    = LaunchConfiguration('overlap_scale').perform(context)
    port_lower       = LaunchConfiguration('port_lower').perform(context)
    port_upper       = LaunchConfiguration('port_upper').perform(context)

    # --- パッケージパスの解決 ---
    description_share = get_package_share_directory('seed_r7_description')

    try:
        interface_share = get_package_share_directory('seed_r7_robot_interface')
    except Exception:
        interface_share = None
        print('[seed_r7_bringup] WARNING: seed_r7_robot_interface not found')

    # --- ファイルパス ---
    xacro_file = os.path.join(description_share, robot_model, 'noid.urdf.xacro')

    ros2_controllers_yaml = os.path.join(
        description_share, robot_model, 'ros2_controllers.yaml')

    hw_settings_file = ''
    if interface_share:
        hw_settings_path = os.path.join(
            interface_share, robot_model, 'config', 'hw_settings.yaml')
        if os.path.isfile(hw_settings_path):
            hw_settings_file = hw_settings_path
        else:
            print(f'[seed_r7_bringup] WARNING: hw_settings.yaml not found: {hw_settings_path}')

    # --- xacro でロボット記述を生成 ---
    xacro_mappings = {
        'port_upper':          port_upper,
        'port_lower':          port_lower,
        'robot_model_plugin':  f'seed_r7_robot_interface/{robot_model}',
        'controller_rate':     controller_rate,
        'overlap_scale':       overlap_scale,
        'joint_settings_file': hw_settings_file,
    }

    if not os.path.isfile(xacro_file):
        raise FileNotFoundError(
            f'[seed_r7_bringup] URDF xacro not found: {xacro_file}')

    robot_desc = xacro.process_file(xacro_file, mappings=xacro_mappings).toxml()

    # --- コントローラリスト (robot_model によって切替) ---
    if robot_model.endswith('arm'):
        controllers = [
            'arm_controller',
            'hand_controller',
            'lifter_controller',
        ]
    else:
        controllers = [
            'larm_controller',
            'rarm_controller',
            'head_controller',
            'waist_controller',
            'lifter_controller',
            'lhand_controller',
            'rhand_controller',
        ]

    return [
        # ---- controller_manager (= ros2_control_node) ----
        # hardware plugin はここで robot_description から読み込まれる
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[
                {'robot_description': robot_desc},
                ros2_controllers_yaml,
            ],
            output='screen',
        ),

        # ---- robot_state_publisher ----
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='seed_r7_state_publisher',
            parameters=[{'robot_description': robot_desc}],
            output='screen',
        ),

        # ---- joint_state_broadcaster (必須: 他コントローラより先に起動) ----
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
            output='screen',
        ),

        # ---- アーム・ハンド・腰・首・リフタコントローラ ----
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=controllers + ['--controller-manager', '/controller_manager'],
            output='screen',
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_model',
            default_value='typef',
            description='Robot model: typef / typeg / typeg_arm / typeg2_arm',
        ),
        DeclareLaunchArgument(
            'controller_rate',
            default_value='50',
            description='Controller read/write cycle rate [Hz]',
        ),
        DeclareLaunchArgument(
            'overlap_scale',
            default_value='2.0',
            description='Scaling factor for target time',
        ),
        DeclareLaunchArgument(
            'use_encoder_odom',
            default_value='false',
            description='Use encoder odometry',
        ),
        DeclareLaunchArgument(
            'pub_robot_info',
            default_value='false',
            description='Publish robot info topic',
        ),
        DeclareLaunchArgument(
            'port_lower',
            default_value='/dev/aero_lower',
            description='Serial port for lower body',
        ),
        DeclareLaunchArgument(
            'port_upper',
            default_value='/dev/aero_upper',
            description='Serial port for upper body',
        ),
        OpaqueFunction(function=launch_setup),
    ])
