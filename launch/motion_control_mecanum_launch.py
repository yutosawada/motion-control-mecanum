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

    motion_controller_node = Node(
        package='motion-control-mecanum-pkg',
        executable='motion_controller_node',
        name='motion_controller_node',
        output='screen',
        parameters=[params_file],
    )

    return LaunchDescription([motion_controller_node])
