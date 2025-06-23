import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory('motion-control-mecanum-pkg')
    params_file = os.path.join(
        pkg_share,
        'config',
        'motion_control_mecanum_params.yaml',
    )

    # teleop_twist_joyのパラメータファイル
    teleop_pkg_share = get_package_share_directory('teleop_twist_joy')
    teleop_params_file = os.path.join(
        teleop_pkg_share,
        'config',
        'teleop_twist_joy_ps4.yaml',
    )

    motion_controller_node = Node(
        package='motion-control-mecanum-pkg',
        executable='motion_controller_node',
        name='motion_controller_node',
        output='screen',
        parameters=[params_file],
    )

    # Joyノード（ジョイスティック入力を読み取る）
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
    )

    # teleop_twist_joyノード（ジョイスティック入力をTwistメッセージに変換）
    teleop_twist_joy_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy_node',
        output='screen',
        parameters=[teleop_params_file],
    )

    return LaunchDescription([
        motion_controller_node,
        joy_node,
        teleop_twist_joy_node,
    ])
