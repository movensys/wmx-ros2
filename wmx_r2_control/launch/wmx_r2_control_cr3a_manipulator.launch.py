"""
Bring up the Dobot CR3A arm under ros2_control on WMX3.

Arm motion goes through a stock JointTrajectoryController writing 'position'
command interfaces, which WmxSystemHardware streams to the servos via the WMX3
cyclic buffer. That gives both endpoints MoveIt already expects on the same names
it uses in simulation -- the follow_joint_trajectory action for planned motion and
the joint_trajectory topic for MoveIt Servo streaming -- so nothing on the
manipulator side needs reconfiguring.

Two arguments exist for bringing this up safely on real hardware:

  stream_position:=false      Fall back to the previous behaviour: joints are
                              feedback-only and the arm controller is not loaded.
                              Motion must then come from the standalone
                              wmx_r2_package/joint_trajectory_controller node
                              (see wmx_r2_package/launch/wmx_r2_cr3a_manipulator.launch.py).

  activate_arm_controller:=false
                              Load the arm controller but leave it inactive, so
                              nothing writes to the command interfaces. The
                              hardware still opens the cyclic buffer and holds
                              position, which is the first validation step: watch
                              for zero drift and a stable buffer depth before
                              letting a controller command anything.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    AndSubstitution,
    Command,
    LaunchConfiguration,
    NotSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    stream_position = LaunchConfiguration('stream_position')
    activate_arm_controller = LaunchConfiguration('activate_arm_controller')

    ctrl_share = get_package_share_directory('wmx_r2_control')
    wmx_share = get_package_share_directory('wmx_r2_package')

    urdf_xacro = os.path.join(ctrl_share, 'urdf', 'cr3a.wmx.urdf.xacro')
    controllers = os.path.join(ctrl_share, 'config', 'cr3a_controllers.yaml')
    wmx_param_file = os.path.join(wmx_share, 'config', 'cr3a_wmx_parameters.xml')
    manipulator_config = os.path.join(wmx_share, 'config', 'cr3a_manipulator_config.yaml')

    robot_description = {
        'robot_description': ParameterValue(
            Command([
                'xacro ', urdf_xacro,
                ' wmx_param_file:=', wmx_param_file,
                ' stream_position:=', stream_position,
            ]),
            value_type=str,
        )
    }

    general_nodes = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(wmx_share, 'launch', 'wmx_r2_general_nodes.launch.py')
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
        parameters=[robot_description, controllers, {'use_sim_time': use_sim_time}],
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster',
                   '--controller-manager', '/controller_manager'],
        output='screen',
    )

    # Two nodes rather than one with a conditional flag: a substitution that
    # resolves to '' would still be passed to the spawner as an empty argv entry,
    # which its argument parser reads as a second controller name.
    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['movensys_manipulator_arm_controller',
                   '--controller-manager', '/controller_manager'],
        condition=IfCondition(AndSubstitution(stream_position, activate_arm_controller)),
        output='screen',
    )

    # '--inactive' loads and configures the controller without activating it, so
    # it claims no command interfaces until someone switches it on.
    arm_controller_spawner_inactive = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['movensys_manipulator_arm_controller',
                   '--controller-manager', '/controller_manager',
                   '--inactive'],
        condition=IfCondition(
            AndSubstitution(stream_position, NotSubstitution(activate_arm_controller))
        ),
        output='screen',
    )

    # Superseded by the arm controller above when streaming positions: once the
    # cyclic buffer runs the axes are in DirectControl and this node's
    # StartCSplinePos would be rejected. Still the motion path when
    # stream_position:=false.
    standalone_joint_trajectory_controller = Node(
        package='wmx_r2_package',
        executable='joint_trajectory_controller',
        name='joint_trajectory_controller',
        parameters=[
            manipulator_config,
            {
                'use_sim_time': use_sim_time,
                'wmx_param_file_path': wmx_param_file,
            },
        ],
        condition=UnlessCondition(stream_position),
        output='screen',
    )

    gripper_controller = Node(
        package='wmx_r2_package',
        executable='gripper_controller',
        name='gripper_controller',
        parameters=[manipulator_config, {'use_sim_time': use_sim_time}],
        additional_env={'MANIPULATOR_MODEL': 'dobot_cr3a'},
        output='screen',
    )

    isaacsim_joint_command_relay = Node(
        package='topic_tools',
        executable='relay',
        name='isaacsim_joint_command_relay',
        arguments=['/joint_states', '/isaacsim/joint_command'],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock if true',
        ),
        DeclareLaunchArgument(
            'stream_position',
            default_value='true',
            description='Export position command interfaces and stream them through the '
                        'WMX3 cyclic buffer, driven by a JointTrajectoryController. Set '
                        'false for the previous feedback-only hardware plus the standalone '
                        'joint_trajectory_controller node.',
        ),
        DeclareLaunchArgument(
            'activate_arm_controller',
            default_value='true',
            description='Activate the arm controller on start. Set false to load it '
                        'inactive so the hardware holds position with nothing commanding '
                        'it (first hardware validation step). Ignored when '
                        'stream_position:=false.',
        ),
        general_nodes,
        robot_state_publisher,
        controller_manager,
        joint_state_broadcaster_spawner,
        arm_controller_spawner,
        arm_controller_spawner_inactive,
        standalone_joint_trajectory_controller,
        gripper_controller,
        isaacsim_joint_command_relay,
    ])
