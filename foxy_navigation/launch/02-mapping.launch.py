import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch.event_handlers import OnProcessExit
from ament_index_python.packages import get_package_share_directory

slam_toolbox_file = os.path.join(get_package_share_directory('foxy_navigation'), 'config', 'mapper_params_online_async.yaml')
default_rviz_config_path = os.path.join(FindPackageShare(package='foxy_navigation').find('foxy_navigation'), 'rviz/mapping.rviz')

start_simulation_controller = IncludeLaunchDescription(PythonLaunchDescriptionSource(
    os.path.join(FindPackageShare(package='foxy_navigation').find('foxy_navigation'), 'launch', '01-controller.launch.py')))

start_slam_toolbox = IncludeLaunchDescription(PythonLaunchDescriptionSource(
    os.path.join(FindPackageShare(package='slam_toolbox').find('slam_toolbox'), 'launch', 'online_async_launch.py')),
    launch_arguments={'slam_params_file': slam_toolbox_file}.items())

start_rviz = Node(package='rviz2', executable='rviz2', name='rviz2', output='screen', arguments=['-d', default_rviz_config_path])
    
def generate_launch_description():
    return LaunchDescription([
        start_simulation_controller,
        
        start_slam_toolbox,
        
        start_rviz,
    ])