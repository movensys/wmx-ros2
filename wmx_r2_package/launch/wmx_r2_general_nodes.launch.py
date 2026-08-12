from typing import List

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    engine_core = LaunchConfiguration('engine_core', default='-1')
    engine_affinity_mask = LaunchConfiguration('engine_affinity_mask', default='0')
    auto_manage_nodes = LaunchConfiguration('auto_manage_nodes', default='true')
    managed_nodes = LaunchConfiguration(
        'managed_nodes',
        default="['wmx_core_motion_node', 'wmx_io_node', 'wmx_ethercat_node']")

    start_wmx_engine_node = Node(
        package='wmx_r2_package',
        executable='wmx_engine_node',
        name='wmx_engine_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'engine_core': ParameterValue(engine_core, value_type=int),
            'engine_affinity_mask': ParameterValue(engine_affinity_mask, value_type=int),
            'auto_manage_nodes': ParameterValue(auto_manage_nodes, value_type=bool),
            'managed_nodes': ParameterValue(managed_nodes, value_type=List[str]),
        }],
    )

    # The three nodes below are lifecycle nodes. They start unconfigured and are
    # configured/activated by wmx_engine_node once the engine is communicating
    # (auto_manage_nodes), or on demand via wmx/engine/set_node_state.
    start_wmx_core_motion_node = LifecycleNode(
        package='wmx_r2_package',
        executable='wmx_core_motion_node',
        name='wmx_core_motion_node',
        namespace='',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    start_wmx_io_node = LifecycleNode(
        package='wmx_r2_package',
        executable='wmx_io_node',
        name='wmx_io_node',
        namespace='',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    start_wmx_ethercat_node = LifecycleNode(
        package='wmx_r2_package',
        executable='wmx_ethercat_node',
        name='wmx_ethercat_node',
        namespace='',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock'
        ),
        DeclareLaunchArgument(
            'engine_core',
            default_value='-1',
            description='CPU core for the WMX real-time engine (-1 = SDK default)'
        ),
        DeclareLaunchArgument(
            'engine_affinity_mask',
            default_value='0',
            description='CPU affinity bitmask for the WMX engine threads, '
                        'one bit per core e.g. 12 for cores 2 and 3 (0 = SDK default)'
        ),
        DeclareLaunchArgument(
            'managed_nodes',
            default_value="['wmx_core_motion_node', 'wmx_io_node', 'wmx_ethercat_node']",
            description='Lifecycle nodes wmx_engine_node brings up, in order. Robot launch '
                        'files extend this list with their controllers'
        ),
        DeclareLaunchArgument(
            'auto_manage_nodes',
            default_value='true',
            description='Let wmx_engine_node configure and activate the managed '
                        'lifecycle nodes (core motion, IO, EtherCAT) once the engine '
                        'is communicating. Set false to drive them manually with '
                        'wmx/engine/set_node_state or ros2 lifecycle set'
        ),

        start_wmx_engine_node,
        start_wmx_core_motion_node,
        start_wmx_io_node,
        start_wmx_ethercat_node
    ])
