from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    pkg_share = get_package_share_directory('wmx_ros2_package')

    config = os.path.join(
        pkg_share, 'config', 'cr3a_manipulator_config.yaml')

    wmx_param_file_path = os.path.join(
        pkg_share, 'config', 'cr3a_wmx_parameters.xml')

    start_manipulator_state = Node(
        package='wmx_ros2_package',
        executable='manipulator_state',
        name='manipulator_state',
        parameters=[
            config,
            {
                'use_sim_time': use_sim_time,
                'wmx_param_file_path': wmx_param_file_path,
            },
        ],
        output='screen',
    )

    start_follow_joint_trajectory_server = Node(
        package='wmx_ros2_package',
        executable='follow_joint_trajectory_server',
        name='follow_joint_trajectory_server',
        parameters=[config, {'use_sim_time': use_sim_time}],
        output='screen',
    )

    start_wmx_engine_node = Node(
        package='wmx_ros2_package',
        executable='wmx_engine_node',
        name='wmx_engine_node',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
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

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock if true',
        ),
        start_manipulator_state,
        start_follow_joint_trajectory_server,
        start_wmx_engine_node,
        start_wmx_core_motion_node,
        start_wmx_io_node,
    ])
