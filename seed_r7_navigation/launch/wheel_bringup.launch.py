"""
wheel_bringup.launch.py  (ROS2 Jazzy)

URG レーザースキャナ + ジョイスティック + teleop を起動する。
ROS1 wheel_bringup.launch の移植。

変更点:
  - urg_node (ROS1) → urg_node (ROS2, executable: urg_node_driver)
  - <rosparam command="load"> → Node の parameters= に yaml を渡す
  - <node pkg="joy" type="joy_node"> → joy joy_node (同名)
  - <node pkg="teleop_twist_joy"> → teleop_twist_joy teleop_node

依存パッケージ (未インストールの場合):
  sudo apt install ros-jazzy-urg-node
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    bringup_share = get_package_share_directory('seed_r7_bringup')

    use_rear_laser  = LaunchConfiguration('use_rear_laser').perform(context)
    ip_address      = LaunchConfiguration('ip_address').perform(context)
    ip_address_rear = LaunchConfiguration('ip_address_rear').perform(context)
    joy_config      = LaunchConfiguration('joy_config').perform(context)
    joy_dev         = LaunchConfiguration('joy_dev').perform(context)

    joy_config_file = os.path.join(bringup_share, 'config', f'{joy_config}.config.yaml')
    if not os.path.isfile(joy_config_file):
        print(f'[wheel_bringup] WARNING: joy config not found: {joy_config_file}')
        joy_config_file = None

    # ---- URG front laser ----
    urg_node = Node(
        package='urg_node',
        executable='urg_node_driver',
        name='urg_node',
        output='screen',
        parameters=[{
            'ip_address': ip_address,
            'frame_id':   'wheels_base_laser_link',
            'angle_min':  -2.01,
            'angle_max':   2.01,
        }],
    )

    # ---- URG rear laser (optional) ----
    urg_node_rear = Node(
        package='urg_node',
        executable='urg_node_driver',
        name='urg_node_rear',
        output='screen',
        parameters=[{
            'ip_address': ip_address_rear,
            'frame_id':   'wheels_rear_laser_link',
            'angle_min':  -2.01,
            'angle_max':   2.01,
        }],
        remappings=[('scan', 'scan_rear')],
    ) if use_rear_laser.lower() == 'true' else None

    # ---- Joystick ----
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        parameters=[{
            'dev':             joy_dev,
            'deadzone':        0.3,
            'autorepeat_rate': 50.0,
        }],
    )

    # ---- teleop_twist_joy ----
    teleop_params = [{'publish_stamped_twist': False}]
    if joy_config_file:
        teleop_params.append(joy_config_file)

    teleop_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy',
        output='screen',
        parameters=teleop_params,
    )

    nodes = [urg_node, joy_node, teleop_node]
    if urg_node_rear:
        nodes.append(urg_node_rear)
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_rear_laser',
            default_value='false',
            description='Launch a second URG for rear scan',
        ),
        DeclareLaunchArgument(
            'ip_address',
            default_value='192.168.0.10',
            description='IP address of the front URG laser',
        ),
        DeclareLaunchArgument(
            'ip_address_rear',
            default_value='192.168.0.11',
            description='IP address of the rear URG laser',
        ),
        DeclareLaunchArgument(
            'joy_config',
            default_value='ps3-holonomic',
            description='Joystick config name (seed_r7_bringup/config/<name>.config.yaml)',
        ),
        DeclareLaunchArgument(
            'joy_dev',
            default_value='/dev/input/js0',
            description='Joystick device path',
        ),
        OpaqueFunction(function=launch_setup),
    ])
