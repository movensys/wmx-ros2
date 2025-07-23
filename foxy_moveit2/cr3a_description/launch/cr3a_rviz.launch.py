import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import launch_ros.actions
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch.event_handlers import OnProcessExit
from ament_index_python.packages import get_package_share_directory

xacro_file = os.path.join(get_package_share_directory('cr3a_description'), 'urdf', 'cr3a.xacro')
rviz_config_path = os.path.join(get_package_share_directory('cr3a_description'), 'rviz', 'cr3a.rviz')
    
rviz_node = launch_ros.actions.Node(package='rviz2', executable='rviz2', name='rviz2', output='screen', arguments=['-d', rviz_config_path])

robot_state_publisher = launch_ros.actions.Node(package='robot_state_publisher', executable='robot_state_publisher',
                        parameters=[{'robot_description': Command(['xacro ', xacro_file])}])

start_joint_gui = Node(package='joint_state_publisher_gui', executable='joint_state_publisher_gui', 
                        name='joint_state_publisher_gui', output='screen', parameters=[{'use_gui': True}])

def generate_launch_description():
    return LaunchDescription([
        rviz_node,
        robot_state_publisher,
        start_joint_gui,
    ])