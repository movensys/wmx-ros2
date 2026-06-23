import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    pkg_share = get_package_share_directory('wmx_ros2_robot_option')
    option_config = os.path.join(pkg_share, 'config', 'cr3a_robot_option_example.yaml')

    # Robot kinematics model shipped with this package.
    default_robot_xml = os.path.join(pkg_share, 'config', 'cr3a_robot_option_example.xml')

    # Reuse the system/axis parameter file maintained in wmx_ros2_package.
    message_pkg_share = get_package_share_directory('wmx_ros2_package')
    default_wmx_param = os.path.join(message_pkg_share, 'config', 'cr3a_wmx_parameters.xml')

    robot_xml_path = LaunchConfiguration('robot_xml_path', default=default_robot_xml)
    wmx_param_file_path = LaunchConfiguration('wmx_param_file_path', default=default_wmx_param)

    start_wmx_robot_option_node = Node(
        package='wmx_ros2_robot_option',
        executable='wmx_robot_option_node',
        name='wmx_robot_option_node',
        parameters=[
            option_config,
            {
                'use_sim_time': use_sim_time,
                'robot_xml_path': robot_xml_path,
                'wmx_param_file_path': wmx_param_file_path,
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
            'robot_xml_path',
            default_value=default_robot_xml,
            description='Absolute path to the robot kinematics XML model',
        ),
        DeclareLaunchArgument(
            'wmx_param_file_path',
            default_value=default_wmx_param,
            description='Absolute path to the WMX3 system/axis parameter XML',
        ),
        start_wmx_robot_option_node,
    ])
