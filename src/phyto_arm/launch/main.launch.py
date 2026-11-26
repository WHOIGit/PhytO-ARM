"""Launch core PhytO-ARM processes. Arm launch files configure each payload."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = LaunchConfiguration('params_file')

    return LaunchDescription(
        [
            DeclareLaunchArgument('config_file'),
            DeclareLaunchArgument('params_file'),
            DeclareLaunchArgument('digital_logger', default_value='true'),
            DeclareLaunchArgument('network_data_capture', default_value='false'),
            IncludeLaunchDescription(
                PathJoinSubstitution(
                    [
                        FindPackageShare('rosbridge_server'),
                        'launch',
                        'rosbridge_websocket_launch.xml',
                    ]
                ),
                launch_arguments={'use_compression': 'true'}.items(),
            ),
            Node(
                package='phyto_arm',
                executable='lock_manager.py',
                name='lock_manager',
                parameters=[params_file],
            ),
            Node(
                package='phyto_arm',
                executable='network_data_capture.py',
                name='network_data_capture',
                parameters=[params_file],
                condition=IfCondition(LaunchConfiguration('network_data_capture')),
            ),
            # The triton_classifier and compressed-to-raw image republisher are disabled.
            IncludeLaunchDescription(
                PathJoinSubstitution(
                    [FindPackageShare('phyto_arm'), 'launch', 'main_devices.launch.py']
                ),
                launch_arguments={'config_file': LaunchConfiguration('config_file')}.items(),
            ),
            Node(
                package='phyto_arm',
                executable='web_node.py',
                name='web',
                parameters=[params_file],
            ),
            Node(
                package='dli_power_switch',
                executable='dli_power_switch_node.py',
                name='digital_logger',
                parameters=[params_file],
                condition=IfCondition(LaunchConfiguration('digital_logger')),
            ),
        ]
    )
