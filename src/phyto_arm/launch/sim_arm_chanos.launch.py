from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = LaunchConfiguration('params_file')
    chanos_winch = LaunchConfiguration('chanos_winch')

    return LaunchDescription(
        [
            DeclareLaunchArgument('ctd', default_value='rbr'),
            DeclareLaunchArgument('params_file'),
            DeclareLaunchArgument('chanos_winch'),
            DeclareLaunchArgument('motor_modbus_port', default_value='1504'),
            GroupAction(
                actions=[
                    PushRosNamespace('arm_chanos'),
                    IncludeLaunchDescription(
                        PathJoinSubstitution(
                            [FindPackageShare('phyto_arm'), 'launch', 'sim_cast.launch.py']
                        ),
                        launch_arguments={
                            'arm': 'arm_chanos',
                            'ctd': LaunchConfiguration('ctd'),
                            'params_file': params_file,
                            'motor_modbus_port': LaunchConfiguration('motor_modbus_port'),
                            'winch_enabled': chanos_winch,
                        }.items(),
                    ),
                    Node(
                        package='phyto_arm',
                        executable='arm_chanos.py',
                        name='arm',
                        parameters=[params_file, {'winch_enabled': chanos_winch}],
                        condition=IfCondition(chanos_winch),
                    ),
                ],
            ),
        ]
    )
