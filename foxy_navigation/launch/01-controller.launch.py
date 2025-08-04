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

start_hil_sim_time = Node(package='foxy_navigation', executable='hil_sim_time.py', name='hil_sim_time', output='screen',
    parameters=[{'use_sim_time': True}])

start_robot_localization = Node(package='robot_localization', executable='ekf_node', name='ekf_filter_node', output='screen', 
	parameters=[ekf_config_file, {'use_sim_time': True}])

def generate_launch_description():
    return LaunchDescription([
        start_gazebo_simulation,

        start_hil_sim_time,
        
        start_robot_localization,
    ])