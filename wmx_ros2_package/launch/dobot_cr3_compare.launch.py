from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('wmx_ros2_package'),
        'config',
        'cr3a_config.yaml'
    )

    return LaunchDescription([
        Node(
            package='wmx_ros2_package',
            executable='dobot_cr3_compare',
            name='dobot_cr3_compare',
            parameters=[config],
            output='screen'
        )
    ])
