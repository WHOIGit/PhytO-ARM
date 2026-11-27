from launch import LaunchDescription
from launch.actions import Shutdown
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package='aml_ctd',
                executable='aml_sim.py',
                name='aml_sim',
                on_exit=[Shutdown()],
            ),
            Node(
                package='aml_ctd',
                executable='aml_ctd_node.py',
                name='ctd',
                remappings=[('ctd', 'ctd/ctd'), ('~/in', 'sim/aml')],
            ),
        ]
    )
