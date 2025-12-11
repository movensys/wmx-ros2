from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('wmx_ros2_package'),
        'config',
        'manipulator_config.yaml'
    )

    return LaunchDescription([
        Node(
            package='wmx_ros2_package',
            executable='manipulator_state',
            name='manipulator_state',
            parameters=[config],
            output='screen'
        ),
        Node(
            package='wmx_ros2_package',
            executable='manipulator_action',
            name='manipulator_action',
            parameters=[config],
            output='screen'
        ),
    ])
