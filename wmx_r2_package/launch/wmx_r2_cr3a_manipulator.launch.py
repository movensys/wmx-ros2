import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    pkg_share = get_package_share_directory('wmx_r2_package')
    manipulator_config = os.path.join(pkg_share, 'config', 'cr3a_manipulator_config.yaml')
    wmx_param_file = os.path.join(pkg_share, 'config', 'cr3a_wmx_parameters.xml')

    start_wmx_r2_general_nodes = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'wmx_r2_general_nodes.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'config_file': manipulator_config,
            'wmx_param_file': wmx_param_file,
        }.items(),
    )

    start_joint_state_broadcaster = LifecycleNode(
        package='wmx_r2_package',
        executable='joint_state_broadcaster',
        name='joint_state_broadcaster',
        namespace='',
        parameters=[manipulator_config, {'use_sim_time': use_sim_time}],
        output='screen',
    )

    start_joint_trajectory_controller = LifecycleNode(
        package='wmx_r2_package',
        executable='joint_trajectory_controller',
        name='joint_trajectory_controller',
        namespace='',
        parameters=[manipulator_config, {'use_sim_time': use_sim_time}],
        output='screen',
    )

    start_joint_position_controller = LifecycleNode(
        package='wmx_r2_package',
        executable='joint_position_controller',
        name='joint_position_controller',
        namespace='',
        parameters=[manipulator_config, {'use_sim_time': use_sim_time}],
        output='screen',
    )

    start_gripper_controller = LifecycleNode(
        package='wmx_r2_package',
        executable='gripper_controller',
        name='gripper_controller',
        namespace='',
        parameters=[manipulator_config, {'use_sim_time': use_sim_time}],
        additional_env={'MANIPULATOR_MODEL': 'dobot_cr3a'},
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock if true',
        ),
        start_wmx_r2_general_nodes,
        start_joint_state_broadcaster,
        start_joint_trajectory_controller,
        start_joint_position_controller,
        start_gripper_controller
    ])
