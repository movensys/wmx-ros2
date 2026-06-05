import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    ctrl_share = get_package_share_directory('wmx_ros2_control')
    wmx_share = get_package_share_directory('wmx_ros2_package')

    urdf_xacro = os.path.join(ctrl_share, 'urdf', 'cr3a.wmx.urdf.xacro')
    controllers = os.path.join(ctrl_share, 'config', 'cr3a_controllers.yaml')
    wmx_param_file = os.path.join(wmx_share, 'config', 'cr3a_wmx_parameters.xml')
    manipulator_config = os.path.join(wmx_share, 'config', 'cr3a_manipulator_config.yaml')

    robot_description = {
        'robot_description': ParameterValue(
            Command(['xacro ', urdf_xacro, ' wmx_param_file:=', wmx_param_file]),
            value_type=str,
        )
    }

    general_nodes = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(wmx_share, 'launch', 'wmx_ros2_general_nodes.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': use_sim_time}],
    )

    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        output='screen',
        parameters=[robot_description, controllers],
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster',
                   '--controller-manager', '/controller_manager'],
        output='screen',
    )

    joint_trajectory_controller = Node(
        package='wmx_ros2_package',
        executable='joint_trajectory_controller',
        name='joint_trajectory_controller',
        parameters=[
            manipulator_config,
            {
                'use_sim_time': use_sim_time,
                'wmx_param_file_path': wmx_param_file,
            },
        ],
        output='screen',
    )

    gripper_controller = Node(
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
        general_nodes,
        robot_state_publisher,
        controller_manager,
        joint_state_broadcaster_spawner,
        joint_trajectory_controller,
        gripper_controller,
    ])
