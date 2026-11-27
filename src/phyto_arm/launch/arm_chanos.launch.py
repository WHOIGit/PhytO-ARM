from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description():
    params_file = LaunchConfiguration('params_file')
    chanos_winch = LaunchConfiguration('chanos_winch')

    return LaunchDescription(
        [
            DeclareLaunchArgument('params_file'),
            DeclareLaunchArgument('chanos_winch', default_value='false'),
            GroupAction(
                actions=[
                    PushRosNamespace('arm_chanos'),
                    Node(
                        package='rbr_maestro3_ctd',
                        executable='rbr_maestro3_node.py',
                        name='ctd',
                        parameters=[params_file],
                        remappings=[('~/in', 'udp_stream')],
                    ),
                    Node(
                        package='phyto_arm',
                        executable='arm_chanos.py',
                        name='arm',
                        parameters=[params_file, {'winch_enabled': chanos_winch}],
                    ),
                    GroupAction(
                        condition=IfCondition(chanos_winch),
                        actions=[
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
                ],
            ),
        ]
    )
