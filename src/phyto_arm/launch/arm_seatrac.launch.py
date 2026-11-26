from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = LaunchConfiguration('params_file')

    return LaunchDescription(
        [
            DeclareLaunchArgument('params_file'),
            DeclareLaunchArgument('ifcb_winch', default_value='false'),
            IncludeLaunchDescription(
                PathJoinSubstitution(
                    [FindPackageShare('phyto_arm'), 'launch', 'seatrac.launch.py']
                ),
                launch_arguments={'params_file': params_file}.items(),
            ),
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
            # Match namespace from arm_ifcb.launch
            GroupAction(
                actions=[
                    PushRosNamespace('arm_ifcb'),
                    Node(
                        package='phyto_arm',
                        executable='arm_ifcb.py',
                        name='arm',
                        parameters=[
                            params_file,
                            {'winch_enabled': LaunchConfiguration('ifcb_winch')},
                        ],
                    ),
                    Node(
                        package='wr2_asio',
                        executable='adapter_node',
                        name='ctd_comms',
                        parameters=[
                            {'matcher.match_type': 'match_char', 'matcher.character': 'newline'},
                            params_file,
                        ],
                        remappings=[
                            ('ros_connection/receive', '~/out'),
                            ('ros_connection/transmit', '~/in'),
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
                        executable='ifcb_runner.py',
                        name='ifcb_runner',
                        parameters=[params_file],
                    ),
                ],
            ),
        ]
    )
