import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch.event_handlers import OnProcessExit
from ament_index_python.packages import get_package_share_directory

ekf_config_file = os.path.join(FindPackageShare(package='wmx_ros2_navigation2_package').find('wmx_ros2_navigation2_package'), 'config', 'ekf.yaml')

start_robot_localization = Node(package='robot_localization', executable='ekf_node', name='ekf_filter_node', output='screen', 
	parameters=[ekf_config_file, {'use_sim_time': True}])

def generate_launch_description():
    return LaunchDescription([
        start_robot_localization,
    ])