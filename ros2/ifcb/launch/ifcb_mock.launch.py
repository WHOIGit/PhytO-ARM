"""Run the IFCB node with its in-memory instrument."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_ifcb(context):
    params_file = LaunchConfiguration('params_file').perform(context)
    params = [params_file] if params_file else []
    return [Node(package='ifcb', executable='ifcb_mock', name='ifcb',
                 parameters=params + [{'data_dir': '/sim/ifcb'}],
                 on_exit=[Shutdown(reason='IFCB mock exited')])]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=''),
        OpaqueFunction(function=launch_ifcb),
    ])
