from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_ctd(context):
    driver = LaunchConfiguration('ctd').perform(context)
    params = LaunchConfiguration('params_file').perform(context)
    remappings = [] if driver == 'direct' else [('ctd/ctd', 'sim/ctd'), ('ctd/depth', 'sim/depth')]
    nodes = [Node(package='phyto_sim_utils', executable='ctd_sim.py', name='ctd_sim',
                  parameters=[params] if params else [], remappings=remappings,
                  on_exit=[Shutdown()])]
    if driver != 'direct':
        package = {'aml': 'aml_ctd', 'rbr': 'rbr_maestro3_ctd'}[driver]
        launch = PathJoinSubstitution([FindPackageShare(package), 'launch', 'sim_ctd.launch.py'])
        nodes.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(launch)))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('ctd', default_value='direct', choices=['direct', 'aml', 'rbr']),
        DeclareLaunchArgument('params_file', default_value=''),
        OpaqueFunction(function=launch_ctd),
    ])
