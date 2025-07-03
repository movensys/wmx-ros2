import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch.event_handlers import OnProcessExit
from ament_index_python.packages import get_package_share_directory

ekf_config_file = os.path.join(FindPackageShare(package='foxy_navigation').find('foxy_navigation'), 'config', 'ekf.yaml')

start_gazebo_simulation = IncludeLaunchDescription(PythonLaunchDescriptionSource(
    os.path.join(FindPackageShare(package='baymax_description').find('baymax_description'), 'launch', 'baymax_gazebo.launch.py')))

start_controller = Node(package='foxy_navigation', executable='diff_drive_controller', name='diff_drive_controller', output='screen')

start_robot_localization = Node(package='robot_localization', executable='ekf_node', name='ekf_filter_node', output='screen', 
	parameters=[ekf_config_file, {'use_sim_time': True}])

def generate_launch_description():
    return LaunchDescription([
        start_gazebo_simulation,
        
        start_controller,
        
        start_robot_localization,
    ])