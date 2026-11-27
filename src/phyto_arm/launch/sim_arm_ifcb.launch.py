from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    mock_ifcb = LaunchConfiguration('mock_ifcb')
    params_file = LaunchConfiguration('params_file')
    ifcb_winch = LaunchConfiguration('ifcb_winch')

    return LaunchDescription(
        [
            DeclareLaunchArgument('ctd', default_value='direct'),
            DeclareLaunchArgument('params_file'),
            DeclareLaunchArgument('ifcb_winch'),
            DeclareLaunchArgument('mock_ifcb', default_value='true'),
            DeclareLaunchArgument('motor_modbus_port', default_value='1503'),
            IncludeLaunchDescription(
                PathJoinSubstitution([FindPackageShare('ifcb'), 'launch', 'ifcb_mock.launch.py']),
                launch_arguments={'params_file': params_file}.items(),
                condition=IfCondition(mock_ifcb),
            ),
            GroupAction(
                actions=[
                    PushRosNamespace('arm_ifcb'),
                    IncludeLaunchDescription(
                        PathJoinSubstitution(
                            [FindPackageShare('phyto_arm'), 'launch', 'sim_cast.launch.py']
                        ),
                        launch_arguments={
                            'arm': 'arm_ifcb',
                            'ctd': LaunchConfiguration('ctd'),
                            'params_file': params_file,
                            'motor_modbus_port': LaunchConfiguration('motor_modbus_port'),
                            'winch_enabled': ifcb_winch,
                        }.items(),
                    ),
                    Node(
                        package='phyto_arm',
                        executable='arm_ifcb.py',
                        name='arm',
                        parameters=[params_file, {'winch_enabled': ifcb_winch}],
                    ),
                    Node(
                        package='phyto_arm',
                        executable='ifcb_runner.py',
                        name='ifcb_runner',
                        parameters=[params_file],
                        condition=IfCondition(mock_ifcb),
                    ),
                    Node(
                        package='phyto_arm',
                        executable='profiler_node.py',
                        name='profiler',
                        parameters=[
                            params_file,
                            {
                                'ctd_topic': 'ctd/depth',
                                'data_topic': 'ctd/ctd',
                                'data_field': 'temperature',
                            },
                        ],
                        condition=IfCondition(ifcb_winch),
                    ),
                ],
            ),
        ]
    )
