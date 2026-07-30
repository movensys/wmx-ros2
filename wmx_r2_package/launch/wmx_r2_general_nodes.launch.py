from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    engine_core = LaunchConfiguration('engine_core', default='-1')
    engine_affinity_mask = LaunchConfiguration('engine_affinity_mask', default='0')

    start_wmx_engine_node = Node(
        package='wmx_r2_package',
        executable='wmx_engine_node',
        name='wmx_engine_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'engine_core': ParameterValue(engine_core, value_type=int),
            'engine_affinity_mask': ParameterValue(engine_affinity_mask, value_type=int),
        }],
    )

    start_wmx_core_motion_node = Node(
        package='wmx_r2_package',
        executable='wmx_core_motion_node',
        name='wmx_core_motion_node',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    start_wmx_io_node = Node(
        package='wmx_r2_package',
        executable='wmx_io_node',
        name='wmx_io_node',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    start_wmx_ethercat_node = Node(
        package='wmx_r2_package',
        executable='wmx_ethercat_node',
        name='wmx_ethercat_node',
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

        start_wmx_engine_node,
        start_wmx_core_motion_node,
        start_wmx_io_node,
        start_wmx_ethercat_node
    ])
