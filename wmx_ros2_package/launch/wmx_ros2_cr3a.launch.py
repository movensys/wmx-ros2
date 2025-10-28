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
            executable='cr3a_state',
            name='cr3a_state',
            parameters=[config],
            output='screen'
        ),
        Node(
            package='wmx_ros2_package',
            executable='cr3a_action',
            name='cr3a_action',
            output='screen'
        ),
    ])
