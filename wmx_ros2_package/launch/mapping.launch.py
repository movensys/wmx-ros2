import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch.event_handlers import OnProcessExit
from ament_index_python.packages import get_package_share_directory

ekf_config_file = os.path.join(FindPackageShare(package='wmx_ros2_package').find('wmx_ros2_package'), 'config', 'ekf.yaml')
slam_toolbox_file = os.path.join(get_package_share_directory('wmx_ros2_package'), 'config', 'mapper_params_online_async.yaml')
default_rviz_config_path = os.path.join(FindPackageShare(package='wmx_ros2_package').find('wmx_ros2_package'), 'rviz/mapping.rviz')

start_robot_localization = Node(package='robot_localization', executable='ekf_node', name='ekf_filter_node', output='screen', 
	parameters=[ekf_config_file])

start_slam_toolbox = Node(package='slam_toolbox', executable='async_slam_toolbox_node', name='slam_toolbox', output='screen',
    parameters=[slam_toolbox_file])

start_rviz = Node(package='rviz2', executable='rviz2', name='rviz2', output='screen', arguments=['-d', default_rviz_config_path])

def generate_launch_description():
    return LaunchDescription([
        start_robot_localization,
        start_slam_toolbox,
        start_rviz
    ])
