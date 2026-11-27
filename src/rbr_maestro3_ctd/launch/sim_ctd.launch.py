from launch import LaunchDescription
from launch.actions import Shutdown
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package='rbr_maestro3_ctd',
                executable='rbr_sim.py',
                name='rbr_sim',
                on_exit=[Shutdown()],
            ),
            Node(
                package='rbr_maestro3_ctd',
                executable='rbr_maestro3_node.py',
                name='ctd',
                remappings=[('~/in', 'sim/rbr'), ('ctd', 'ctd/ctd')],
                parameters=[
                    {
                        'channels': [
                            'conductivity(mS/cm)',
                            'temperature(C)',
                            'pressure(dbar)',
                            'depth(m)',
                        ]
                    }
                ],
            ),
        ]
    )
