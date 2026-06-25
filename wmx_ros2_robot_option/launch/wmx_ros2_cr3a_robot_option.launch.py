import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    pkg_share = get_package_share_directory('wmx_ros2_robot_option')
    wmx_pkg_share = get_package_share_directory('wmx_ros2_package')

    option_config = os.path.join(pkg_share, 'config', 'cr3a_robot_option_config.yaml')

    # Robot kinematics model shipped with this package.
    default_robot_params = os.path.join(pkg_share, 'config', 'cr3a_robot_option_parameters.xml')

    # The WMX3 system/axis parameters are owned by joint_trajectory_controller: that
    # node loads them into the shared engine (ImportAndSetAll). The robot option then
    # attaches to the already-configured engine and does not import them itself.
    manipulator_config = os.path.join(wmx_pkg_share, 'config', 'cr3a_manipulator_config.yaml')
    default_wmx_param = os.path.join(wmx_pkg_share, 'config', 'cr3a_wmx_parameters.xml')

    robot_option_parameters_path = LaunchConfiguration(
        'robot_option_parameters_path', default=default_robot_params)
    wmx_param_file_path = LaunchConfiguration('wmx_param_file_path', default=default_wmx_param)

    # Bring up the WMX3 engine (publishes wmx/engine/ready, awaited by both nodes).
    general_nodes = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(wmx_pkg_share, 'launch', 'wmx_ros2_general_nodes.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    # Owns the WMX3 system/axis parameter configuration for the shared engine.
    joint_trajectory_controller = Node(
        package='wmx_ros2_package',
        executable='joint_trajectory_controller',
        name='joint_trajectory_controller',
        parameters=[
            manipulator_config,
            {
                'use_sim_time': use_sim_time,
                'wmx_param_file_path': wmx_param_file_path,
            },
        ],
        output='screen',
    )

    # Robot arm option: attaches to the engine configured by the controller above.
    start_wmx_robot_option_node = Node(
        package='wmx_ros2_robot_option',
        executable='wmx_robot_option_node',
        name='wmx_robot_option_node',
        parameters=[
            option_config,
            {
                'use_sim_time': use_sim_time,
                'robot_option_parameters_path': robot_option_parameters_path,
            },
        ],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock if true',
        ),
        DeclareLaunchArgument(
            'robot_option_parameters_path',
            default_value=default_robot_params,
            description='Absolute path to the robot option parameters XML (kinematics model)',
        ),
        DeclareLaunchArgument(
            'wmx_param_file_path',
            default_value=default_wmx_param,
            description='WMX3 system/axis parameter XML loaded by joint_trajectory_controller',
        ),
        general_nodes,
        joint_trajectory_controller,
        start_wmx_robot_option_node,
    ])
