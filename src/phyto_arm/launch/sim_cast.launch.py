from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = LaunchConfiguration('params_file')
    initial_depth = LaunchConfiguration('initial_depth')

    return LaunchDescription(
        [
            DeclareLaunchArgument('arm'),
            DeclareLaunchArgument('ctd', default_value='direct'),
            DeclareLaunchArgument('params_file'),
            DeclareLaunchArgument('initial_depth', default_value='0.5'),
            DeclareLaunchArgument('motor_modbus_port'),
            DeclareLaunchArgument('winch_enabled'),
            Node(
                package='phyto_sim_utils',
                executable='vertical_line.py',
                name='vertical_line',
                parameters=[params_file, {'initial_depth': initial_depth}],
            ),
            IncludeLaunchDescription(
                PathJoinSubstitution(
                    [FindPackageShare('phyto_sim_utils'), 'launch', 'ctd.launch.py']
                ),
                launch_arguments={
                    'ctd': LaunchConfiguration('ctd'),
                    'params_file': params_file,
                }.items(),
            ),
            GroupAction(
                condition=IfCondition(LaunchConfiguration('winch_enabled')),
                actions=[
                    IncludeLaunchDescription(
                        PathJoinSubstitution(
                            [FindPackageShare('jvl_motor'), 'launch', 'sim_motor.launch.py']
                        ),
                        launch_arguments={
                            'motor_modbus_port': LaunchConfiguration('motor_modbus_port'),
                            'refresh_rate': '20',
                            'params_file': params_file,
                        }.items(),
                    ),
                    Node(
                        package='phyto_sim_utils',
                        executable='winch_drum.py',
                        name='winch_drum',
                        parameters=[params_file, {'initial_line_length': initial_depth}],
                    ),
                    Node(
                        package='phyto_arm',
                        executable='winch_node.py',
                        name='winch',
                        parameters=[params_file, {'ctd_topic': 'ctd/depth'}],
                    ),
                ],
            ),
        ]
    )
