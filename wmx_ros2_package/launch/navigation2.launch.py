import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch.event_handlers import OnProcessExit
from ament_index_python.packages import get_package_share_directory
from nav2_common.launch import RewrittenYaml

ekf_config_file = os.path.join(FindPackageShare(package='wmx_ros2_package').find('wmx_ros2_package'), 'config', 'ekf.yaml')
params_file = os.path.join(FindPackageShare(package='wmx_ros2_package').find('wmx_ros2_package'), 'config', 'navigation.yaml')
map_file_path = os.path.join(FindPackageShare('wmx_ros2_package').find('wmx_ros2_package'), 'maps', 'map.yaml')
default_rviz_config_path = os.path.join(FindPackageShare(package='wmx_ros2_package').find('wmx_ros2_package'), 'rviz/navigation.rviz')

start_robot_localization = Node(package='robot_localization', executable='ekf_node', name='ekf_filter_node', output='screen', 
	parameters=[ekf_config_file])

start_map_server = Node(package='nav2_map_server', executable='map_server', name='map_server', output='screen', parameters=[{'yaml_filename': map_file_path}])

start_amcl_localization = Node(package='nav2_amcl', executable='amcl', name='amcl', output='screen', 
                parameters=[{'base_frame_id': "base_link"}, {'yaml_filename': params_file}])

start_lifecycle_manager = Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_localization', output='screen',
	parameters=[{'autostart': True}, {'node_names': ['map_server', 'amcl']}])

start_navigation = IncludeLaunchDescription(PythonLaunchDescriptionSource(
    os.path.join(FindPackageShare(package='wmx_ros2_package').find('wmx_ros2_package'), 'launch', 'nav2_stack.launch.py')))

start_rviz = Node(package='rviz2', executable='rviz2', name='rviz2', output='screen', arguments=['-d', default_rviz_config_path])

def generate_launch_description():
    return LaunchDescription([
        start_robot_localization,
        start_map_server,
        start_amcl_localization,
        start_lifecycle_manager,
        start_navigation,
        start_rviz
    ])
