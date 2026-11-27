from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    params_file = LaunchConfiguration('params_file')

    return LaunchDescription(
        [
            DeclareLaunchArgument('params_file'),
            Node(
                package='seatrac',
                executable='seatrac_node',
                name='seatrac',
                parameters=[params_file],
                remappings=[('~/in', '~/comms/in'), ('~/out', '~/comms/out')],
            ),
            Node(
                package='wr2_asio',
                executable='adapter_node',
                name='comms',
                namespace='seatrac',
                parameters=[params_file],
                remappings=[
                    ('ros_connection/transmit', '~/in'),
                    ('ros_connection/receive', '~/out'),
                ],
            ),
            Node(
                package='phyto_arm',
                executable='seatrac_starlink_manager.py',
                name='seatrac_starlink_manager',
                parameters=[params_file],
            ),
        ]
    )
