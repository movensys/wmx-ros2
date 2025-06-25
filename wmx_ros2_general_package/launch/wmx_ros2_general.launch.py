from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    start_engine_node = Node(package='wmx_ros2_general_package', executable='wmx_ros2_engine_node', name='wmx_ros2_engine_node',
            output='screen')
            
    start_core_motion_node = Node(package='wmx_ros2_general_package', executable='wmx_ros2_core_motion_node', name='wmx_ros2_core_motion_node',
            output='screen')

    #start_axis_pose_server = Node(package='wmx_ros2_general_package', executable='wmx_axis_pose_server', name='wmx_axis_pose_server',
    #        output='screen')

    return LaunchDescription([
        start_engine_node,
        start_core_motion_node,
        #start_axis_pose_server
    ])