#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory('motion-control-mecanum-pkg')

    # ─── パラメータファイル ─────────────────────────────────────────────
    params_file = os.path.join(pkg_share,  'motion_control_mecanum_params.yaml')
    teleop_params_file = os.path.join(pkg_share,  'dualshock.yaml')
    # もし dualshock.yaml を joy_teleop パッケージ側に置いたなら:
    # teleop_params_file = os.path.join(
    #     get_package_share_directory('joy_teleop'), 'config', 'dualshock.yaml')

    # ─── ノード定義 ─────────────────────────────────────────────────────
    motion_controller_node = Node(
        package='motion-control-mecanum-pkg',
        executable='motion_controller_node',
        name='motion_controller_node',
        output='screen',
        parameters=[params_file],
    )

    # Joy ドライバ
    #joy_node = Node(
    #    package='joy_linux',
    #    executable='joy_linux_node',
    #    name='joy',
    #    output='screen',
    #    parameters=[{
    #        'dev_name': 'Wireless Controller',      # DualShock を自動検出
    #    }],
    #)

    # Joy → Twist 変換
    #joy_teleop_node = Node(
    #    package='joy_teleop',
    #    executable='joy_teleop',
    #    name='joy_teleop',
    #    output='screen',
    #    parameters=[teleop_params_file],
    #)

    # ─── LaunchDescription ────────────────────────────────────────────
    return LaunchDescription([
        motion_controller_node,
    #    joy_node,
    #    joy_teleop_node,
    ])
