import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import GroupAction
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml

params_file = os.path.join(FindPackageShare(package='wmx_ros2_navigation2_package').find('wmx_ros2_navigation2_package'), 'config', 'navigation.yaml')
default_bt_xml_filename = os.path.join(FindPackageShare(package='wmx_ros2_navigation2_package').find('wmx_ros2_navigation2_package'), 'config','navigate_w_replanning_and_recovery.xml')

param_substitutions = {
    'use_sim_time': 'true',
    'default_bt_xml_filename': default_bt_xml_filename}

configured_params = RewrittenYaml(
	source_file=params_file,
	root_key='',
	param_rewrites=param_substitutions,
	convert_types=True)
            
lifecycle_nodes = [
    'controller_server',
    'planner_server',
    'recoveries_server',
    'bt_navigator',
    'waypoint_follower'
]

start_load_nodes = GroupAction(
    actions=[
        Node(package='nav2_controller', executable='controller_server', name='controller_server', output='screen', parameters=[configured_params]),
        Node(package='nav2_planner', executable='planner_server', name='planner_server', output='screen', parameters=[configured_params]),
        Node(package='nav2_recoveries', executable='recoveries_server', name='recoveries_server', output='screen', parameters=[configured_params]),
        Node(package='nav2_bt_navigator', executable='bt_navigator', name='bt_navigator',output='screen', parameters=[configured_params]),
        Node(package='nav2_waypoint_follower', executable='waypoint_follower', name='waypoint_follower', output='screen', parameters=[configured_params])
    ]
)

start_lifecycle_manager = Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_navigation', output='screen',
    parameters=[{'use_sim_time': True}, {'autostart': True}, {'node_names': lifecycle_nodes}])

def generate_launch_description():
    return LaunchDescription([
        start_load_nodes,
        start_lifecycle_manager,
    ])