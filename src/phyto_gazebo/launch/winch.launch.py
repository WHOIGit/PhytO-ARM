"""Run the winch, CTD and instruments against the Gazebo barge."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction, Shutdown
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetParameter
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import ExecutableInPackage, FindPackageShare


def include(package, filename, *, condition=None, **arguments):
    path = PathJoinSubstitution([FindPackageShare(package), 'launch', filename])
    return IncludeLaunchDescription(PythonLaunchDescriptionSource(path),
                                    condition=condition, launch_arguments=arguments.items())


def start_server(context, world):
    args = ['ign', 'gazebo', '-s']
    if not IfCondition(LaunchConfiguration('paused')).evaluate(context):
        args.append('-r')
    return [ExecuteProcess(cmd=[*args, world], output='screen', on_exit=[Shutdown()])]


def generate_launch_description():
    dli_port = LaunchConfiguration('dli_port')
    gpsd_port = LaunchConfiguration('gpsd_port')
    motor_modbus_port = LaunchConfiguration('motor_modbus_port')
    wait = ExecutableInPackage(package='phyto_sim_utils', executable='wait_for_socket')
    world = PathJoinSubstitution([FindPackageShare('phyto_gazebo'), 'worlds', 'winch.world'])
    geometry = {'gear_ratio': 60, 'spool_circumference': 0.6604}

    return LaunchDescription([
        DeclareLaunchArgument('ctd', default_value='direct', choices=['direct', 'aml', 'rbr']),
        DeclareLaunchArgument('dli_port', default_value='18080'),
        DeclareLaunchArgument('gpsd_port', default_value='29470'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('gpsd', default_value='false'),
        DeclareLaunchArgument('mock_ifcb', default_value='false'),
        DeclareLaunchArgument('motor_modbus_port', default_value='1502'),
        DeclareLaunchArgument('paused', default_value='false'),

        SetParameter(name='use_sim_time', value=True),
        OpaqueFunction(function=start_server, args=[world]),
        ExecuteProcess(cmd=['ign', 'gazebo', '-g'], output='screen',
                       condition=IfCondition(LaunchConfiguration('gui')), on_exit=[Shutdown()]),
        Node(package='ros_gz_bridge', executable='parameter_bridge',
             arguments=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock']),
        Node(package='phyto_gazebo', executable='barge_gps_bridge'),
        include('ifcb', 'ifcb_mock.launch.py',
                condition=IfCondition(LaunchConfiguration('mock_ifcb'))),

        Node(package='dli_power_switch', executable='dli_sim.py', name='dli_sim',
             parameters=[{'port': dli_port}], on_exit=[Shutdown()]),
        Node(package='dli_power_switch', executable='dli_power_switch_node.py',
             name='digital_logger', prefix=[wait, ' ', dli_port],
             parameters=[{'address': ['127.0.0.1:', dli_port],
                          'password': ParameterValue('', value_type=str),
                          'username': ParameterValue('', value_type=str),
                          'outlets.0.name': 'starlink', 'outlets.0.outlet': 8}]),

        include('phyto_sim_utils', 'gps.launch.py', fix_topic='/sim/gps',
                gpsd=LaunchConfiguration('gpsd'), gpsd_port=gpsd_port, use_sim_time='true'),
        include('jvl_motor', 'sim_motor.launch.py', motor_modbus_port=motor_modbus_port,
                refresh_rate='20'),
        include('phyto_sim_utils', 'ctd.launch.py', ctd=LaunchConfiguration('ctd')),

        Node(package='phyto_sim_utils', executable='winch_drum.py', name='winch_drum',
             parameters=[geometry], on_exit=[Shutdown()]),
        Node(package='phyto_arm', executable='winch_node.py', name='winch',
             parameters=[{'ctd_topic': '/ctd/depth',
                          'gear_ratio': geometry['gear_ratio'],
                          'spool_circumference': geometry['spool_circumference'],
                          'epsilon': 0.025, 'half_speed_dist': 0.12,
                          'max_speed': 0.25, 'safety_envelopes.position': 0.20}],
             on_exit=[Shutdown()]),
    ])
