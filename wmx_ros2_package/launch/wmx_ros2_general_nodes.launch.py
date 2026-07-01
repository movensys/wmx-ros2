from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    start_wmx_engine_node = Node(
        package='wmx_ros2_package',
        executable='wmx_engine_node',
        name='wmx_engine_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    start_wmx_core_motion_node = Node(
        package='wmx_ros2_package',
        executable='wmx_core_motion_node',
        name='wmx_core_motion_node',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    start_wmx_io_node = Node(
        package='wmx_ros2_package',
        executable='wmx_io_node',
        name='wmx_io_node',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    start_wmx_ethercat_node = Node(
        package='wmx_ros2_package',
        executable='wmx_ethercat_node',
        name='wmx_ethercat_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock'
        ),

        start_wmx_engine_node,
        start_wmx_core_motion_node,
        start_wmx_io_node,
        start_wmx_ethercat_node
    ])
