# WMX R2 bring-up for the Movensys 4-axis cartesian robot.
#
# Same shape as wmx_r2_cr3a_manipulator.launch.py: no ros2_control anywhere.
# WMX R2 owns the motion, and MoveIt talks to it through a plain
# FollowJointTrajectory action (a ROS action = a long-running request with
# feedback and the option to cancel).
#
#   wmx_r2_general_nodes      : WMX engine, core motion, IO, EtherCAT master
#   joint_state_broadcaster   : WMX encoder positions -> /joint_states
#                               (and -> /isaacsim/joint_command for HiL)
#   joint_trajectory_controller: FollowJointTrajectory action server -> WMX spline
#
# Usage (needs root for the real-time EtherCAT master, see
#        wmx-r2/doc/launch_cartesian.md):
#     ros2 launch wmx_r2_package wmx_r2_cartesian.launch.py use_sim_time:=true   # HiL
#     ros2 launch wmx_r2_package wmx_r2_cartesian.launch.py                      # real
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    pkg_share = get_package_share_directory('wmx_r2_package')
    cartesian_config = os.path.join(pkg_share, 'config', 'cartesian_config.yaml')
    wmx_param_file_path = os.path.join(pkg_share, 'config', 'cartesian_wmx_parameters.xml')

    start_wmx_r2_general_nodes = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'wmx_r2_general_nodes.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    start_joint_state_broadcaster = Node(
        package='wmx_r2_package',
        executable='joint_state_broadcaster',
        name='joint_state_broadcaster',
        parameters=[cartesian_config, {'use_sim_time': use_sim_time}],
        output='screen',
    )

    start_joint_trajectory_controller = Node(
        package='wmx_r2_package',
        executable='joint_trajectory_controller',
        name='joint_trajectory_controller',
        parameters=[
            cartesian_config,
            {
                'use_sim_time': use_sim_time,
                'wmx_param_file_path': wmx_param_file_path,
            },
        ],
        output='screen',
    )

    # No gripper_controller: the cartesian machine has no gripper.
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock if true',
        ),
        start_wmx_r2_general_nodes,
        start_joint_state_broadcaster,
        start_joint_trajectory_controller,
    ])
