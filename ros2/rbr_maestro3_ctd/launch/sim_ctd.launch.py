from launch import LaunchDescription
from launch.actions import OpaqueFunction, Shutdown
from launch_ros.actions import Node


def launch_ctd(context):
    namespace = context.launch_configurations.get('ros_namespace', '').strip('/').replace('/', '.')
    channels = '.'.join(part for part in (namespace, 'ctd', 'channels') if part)
    return [
        Node(package='rbr_maestro3_ctd', executable='rbr_sim.py', name='rbr_sim', on_exit=[Shutdown()]),
        Node(package='rbr_maestro3_ctd', executable='rbr_maestro3_node.py', name='ctd',
             remappings=[('~/in', 'sim/rbr'), ('ctd', 'ctd/ctd')],
             parameters=[{channels: ['conductivity(mS/cm)', 'temperature(C)', 'pressure(dbar)', 'depth(m)']}]),
    ]


def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=launch_ctd)])
