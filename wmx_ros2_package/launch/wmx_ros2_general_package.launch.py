from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock'
        ),

        Node(
            package='wmx_ros2_package',
            executable='wmx_engine_node',
            name='wmx_engine_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),

        Node(
            package='wmx_ros2_package',
            executable='wmx_core_motion_node',
            name='wmx_core_motion_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),
        Node(
            package='wmx_ros2_package',
            executable='wmx_io_node',
            name='wmx_io_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),

        Node(
            package='wmx_ros2_package',
            executable='wmx_ethercat_node',
            name='wmx_ethercat_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),
    ])
