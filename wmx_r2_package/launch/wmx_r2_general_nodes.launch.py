from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node


def launch_general_nodes(context):
    use_sim_time = LaunchConfiguration('use_sim_time')
    config_file = LaunchConfiguration('config_file').perform(context)
    engine_config = [config_file] if config_file else []

    start_wmx_engine_node = Node(
        package='wmx_r2_package',
        executable='wmx_engine_node',
        name='wmx_engine_node',
        parameters=engine_config + [{'use_sim_time': use_sim_time}],
        output='screen',
    )

    start_wmx_lifecycle_manager_node = Node(
        package='wmx_r2_package',
        executable='wmx_lifecycle_manager_node',
        name='wmx_lifecycle_manager_node',
        parameters=engine_config + [{'use_sim_time': use_sim_time}],
        output='screen',
    )

    start_wmx_core_motion_node = LifecycleNode(
        package='wmx_r2_package',
        executable='wmx_core_motion_node',
        name='wmx_core_motion_node',
        namespace='',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    start_wmx_io_node = LifecycleNode(
        package='wmx_r2_package',
        executable='wmx_io_node',
        name='wmx_io_node',
        namespace='',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    start_wmx_ethercat_node = LifecycleNode(
        package='wmx_r2_package',
        executable='wmx_ethercat_node',
        name='wmx_ethercat_node',
        namespace='',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    return [
        start_wmx_engine_node,
        start_wmx_lifecycle_manager_node,
        start_wmx_core_motion_node,
        start_wmx_io_node,
        start_wmx_ethercat_node,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock'
        ),
        DeclareLaunchArgument(
            'config_file',
            default_value='',
            description='YAML with the wmx_engine_node and '
                        'wmx_lifecycle_manager_node parameters. '
                        'Empty keeps the node defaults'
        ),

        OpaqueFunction(function=launch_general_nodes),
    ])
