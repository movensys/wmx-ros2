from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    config = os.path.join(
        get_package_share_directory('wmx_ros2_package'),
        'config',
        'orin_manipulator_config_cr3a.yaml'
    )

    start_manipulator_state = Node(package='wmx_ros2_package', executable='manipulator_state', name='manipulator_state',
                                parameters=[config, {'use_sim_time': use_sim_time}], output='screen')

    start_follow_joint_trajectory_server = Node(package='wmx_ros2_package', executable='follow_joint_trajectory_server',
                                name='follow_joint_trajectory_server', parameters=[config, {'use_sim_time': use_sim_time}], output='screen')

    start_wmx_ros2_general = Node(package='wmx_ros2_package', executable='wmx_ros2_general_node', name='wmx_ros2_general_node',
                                parameters=[config, {'use_sim_time': use_sim_time}], output='screen')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false', description='Use simulation clock if true'),
        start_manipulator_state,
        start_follow_joint_trajectory_server,
        start_wmx_ros2_general
    ])
