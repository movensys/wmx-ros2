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

    start_manipulator_state = Node(package='wmx_ros2_package', executable='manipulator_state', name='manipulator_state',
                                parameters=[config], output='screen')
    
    start_follow_joint_trajectory_server = Node(package='wmx_ros2_package', executable='follow_joint_trajectory_server', 
                                name='follow_joint_trajectory_server', parameters=[config], output='screen')

    return LaunchDescription([
        #start_manipulator_state,
        start_follow_joint_trajectory_server
    ])
