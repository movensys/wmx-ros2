from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    pkg_share = get_package_share_directory('wmx_ros2_package')
    manipulator_config = os.path.join(pkg_share, 'config', 'cr3a_manipulator_config.yaml')
    wmx_param_file_path = os.path.join(pkg_share, 'config', 'cr3a_wmx_parameters.xml')

    start_wmx_ros2_general_nodes = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'wmx_ros2_general_nodes.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    start_joint_state_broadcaster = Node(
        package='wmx_ros2_package',
        executable='joint_state_broadcaster',
        name='joint_state_broadcaster',
        parameters=[
            manipulator_config,
            {
                'use_sim_time': use_sim_time,
                'wmx_param_file_path': wmx_param_file_path,
            },
        ],
        output='screen',
    )

    start_joint_trajectory_controller = Node(
        package='wmx_ros2_package',
        executable='joint_trajectory_controller',
        name='joint_trajectory_controller',
        parameters=[manipulator_config, {'use_sim_time': use_sim_time}],
        output='screen',
    )

    start_gripper_controller = Node(
        package='wmx_ros2_package',
        executable='gripper_controller',
        name='gripper_controller',
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
        start_wmx_ros2_general_nodes,
        start_joint_state_broadcaster,
        start_joint_trajectory_controller,
        start_gripper_controller
    ])
