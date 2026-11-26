from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description():
    params_file = LaunchConfiguration('params_file')
    ifcb_winch = LaunchConfiguration('ifcb_winch')

    return LaunchDescription(
        [
            DeclareLaunchArgument('params_file'),
            DeclareLaunchArgument('ifcb_winch', default_value='false'),
            Node(
                package='ifcb',
                executable='ifcb',
                name='ifcb',
                parameters=[params_file],
            ),
            Node(
                package='ifcb',
                executable='ifcb_logfilter',
                name='ifcb_logfilter',
                parameters=[params_file],
            ),
            # AML winch namespace. Important for separating similar nodes on different arms
            GroupAction(
                actions=[
                    PushRosNamespace('arm_ifcb'),
                    Node(
                        package='phyto_arm',
                        executable='arm_ifcb.py',
                        name='arm',
                        parameters=[params_file, {'winch_enabled': ifcb_winch}],
                    ),
                    # Conditionally run the motor, winch, and profiler nodes based on the winch_enabled argument
                    GroupAction(
                        condition=IfCondition(ifcb_winch),
                        actions=[
                            Node(
                                package='wr2_asio',
                                executable='adapter_node',
                                name='ctd_comms',
                                parameters=[
                                    {'matcher.match_type': 'match_char', 'matcher.character': 'newline'},
                                    params_file,
                                ],
                                remappings=[
                                    ('ros_connection/receive', 'ctd_comms/out'),
                                    ('ros_connection/transmit', 'ctd_comms/in'),
                                ],
                            ),
                            Node(
                                package='aml_ctd',
                                executable='aml_ctd_node.py',
                                name='ctd',
                                parameters=[params_file],
                                remappings=[('~/in', 'ctd_comms/in')],
                            ),
                            Node(
                                package='phyto_arm',
                                executable='profiler_node.py',
                                name='profiler',
                                parameters=[params_file],
                            ),
                            Node(
                                package='jvl_motor',
                                executable='jvl_motor_node.py',
                                name='motor',
                                parameters=[params_file],
                            ),
                            Node(
                                package='phyto_arm',
                                executable='winch_node.py',
                                name='winch',
                                parameters=[params_file],
                            ),
                        ],
                    ),
                    Node(
                        package='phyto_arm',
                        executable='ifcb_runner.py',
                        name='ifcb_runner',
                        parameters=[params_file],
                    ),
                ],
            ),
        ]
    )
