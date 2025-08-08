from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    start_general_node = Node(package='wmx_ros2_package', executable='wmx_ros2_general_node', name='wmx_ros2_general_node',
            output='screen')

    return LaunchDescription([
        start_general_node
    ])