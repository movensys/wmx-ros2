from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('wmx_ros2_navigation2_package'),
        'config',
        'diff_drive_config.yaml'
    )

    return LaunchDescription([
        Node(
            package='wmx_ros2_navigation2_package',
            executable='diff_drive_interface',
            name='diff_drive_interface',
            parameters=[config],
            output='screen'
        )
    ])
