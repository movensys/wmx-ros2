import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch.event_handlers import OnProcessExit
from ament_index_python.packages import get_package_share_directory

params_file = os.path.join(FindPackageShare(package='foxy_navigation').find('foxy_navigation'), 'config', 'navigation.yaml')
map_file_path = os.path.join(FindPackageShare('foxy_navigation').find('foxy_navigation'), 'maps', 'map.yaml')
default_rviz_config_path = os.path.join(FindPackageShare(package='foxy_navigation').find('foxy_navigation'), 'rviz/navigation.rviz')

start_simulation_controller = IncludeLaunchDescription(PythonLaunchDescriptionSource(
    os.path.join(FindPackageShare(package='foxy_navigation').find('foxy_navigation'), 'launch', '01-controller.launch.py')))

start_map_server = Node(package='nav2_map_server', executable='map_server', name='map_server', output='screen',  
    parameters=[{'yaml_filename': map_file_path}])
start_amcl_localization = Node(package='nav2_amcl', executable='amcl', name='amcl', output='screen',
    parameters=[{'use_sim_time': True}, {'yaml_filename': params_file}])

start_rviz = Node(package='rviz2', executable='rviz2', name='rviz2', output='screen', arguments=['-d', default_rviz_config_path])
start_lifecycle_manager = Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_localization', output='screen',
	parameters=[{'use_sim_time': True}, {'autostart': True}, {'node_names': ['map_server', 'amcl']}])

start_navigation = IncludeLaunchDescription(PythonLaunchDescriptionSource(
    os.path.join(FindPackageShare(package='foxy_navigation').find('foxy_navigation'), 'launch', 'navigation.launch.py')))

def generate_launch_description():
    return LaunchDescription([
        start_simulation_controller,
        
        start_map_server,
        start_amcl_localization,
         
        start_rviz,
        start_lifecycle_manager,

        start_navigation
    ])