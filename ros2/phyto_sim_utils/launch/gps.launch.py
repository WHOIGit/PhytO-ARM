from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import ExecutableInPackage


def generate_launch_description():
    gpsd = LaunchConfiguration('gpsd')
    port = LaunchConfiguration('gpsd_port')
    fix_topic = ParameterValue(LaunchConfiguration('fix_topic'), value_type=str)
    wait = ExecutableInPackage(package='phyto_sim_utils', executable='wait_for_socket')
    gps = ComposableNode(
        package='gpsd_client', plugin='gpsd_client::GPSDClientComponent', name='gps',
        parameters=[{'check_fix_by_variance': False, 'host': '127.0.0.1',
                     'port': ParameterValue(port, value_type=int), 'use_gps_time': False,
                     'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)}],
        remappings=[('extended_fix', '~/extended_fix'), ('fix', '~/fix')])

    return LaunchDescription([
        DeclareLaunchArgument('fix_topic', default_value=''),
        DeclareLaunchArgument('gpsd', default_value='false'),
        DeclareLaunchArgument('gpsd_port', default_value='29470'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(package='phyto_sim_utils', executable='gps_sim.py', name='gps_sim',
             condition=IfCondition(gpsd), parameters=[{'gps_sim.fix_topic': fix_topic, 'gps_sim.port': port}],
             on_exit=[Shutdown()]),
        ComposableNodeContainer(package='rclcpp_components', executable='component_container',
                                name='gps_container', namespace='', prefix=[wait, ' ', port],
                                condition=IfCondition(gpsd), composable_node_descriptions=[gps],
                                on_exit=[Shutdown()]),
        Node(package='phyto_sim_utils', executable='gpsd_client_mock', name='gps',
             condition=UnlessCondition(gpsd), parameters=[{'gps.fix_topic': fix_topic}],
             on_exit=[Shutdown()]),
    ])
