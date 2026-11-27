import atexit
import os
import tempfile

import yaml

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetLaunchConfiguration,
    Shutdown,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from phyto_arm.deployment import convert_to_ros2_params


def prepare_params(context):
    if LaunchConfiguration('params_file').perform(context):
        return []
    with open(LaunchConfiguration('config_file').perform(context)) as source:
        params = convert_to_ros2_params(yaml.safe_load(source), 'simulator')
    with tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False) as result:
        yaml.safe_dump(params, result)
    atexit.register(os.unlink, result.name)
    return [SetLaunchConfiguration('params_file', result.name)]


def generate_launch_description():
    gazebo = LaunchConfiguration('gazebo')
    ctd = LaunchConfiguration('ctd')
    gpsd = LaunchConfiguration('gpsd')
    gpsd_port = LaunchConfiguration('gpsd_port')
    mock_ifcb = LaunchConfiguration('mock_ifcb')
    motor_port = LaunchConfiguration('motor_modbus_port')
    config = PathJoinSubstitution([FindPackageShare('phyto_arm'), 'configs', 'example.yaml'])
    replay = PathJoinSubstitution([EnvironmentVariable('HOME'), 'ifcb-replay'])

    return LaunchDescription(
        [
            DeclareLaunchArgument('config_file', default_value=config),
            DeclareLaunchArgument('params_file', default_value=''),
            DeclareLaunchArgument('gazebo', default_value='false'),
            DeclareLaunchArgument('ctd', default_value='direct', choices=['direct', 'aml', 'rbr']),
            DeclareLaunchArgument('gpsd', default_value='false'),
            DeclareLaunchArgument('gui', default_value='true'),
            DeclareLaunchArgument('ifcb_replay_dir', default_value=replay),
            DeclareLaunchArgument('mock_ifcb', default_value='true'),
            DeclareLaunchArgument('dli_port', default_value='18080'),
            DeclareLaunchArgument('gpsd_port', default_value='29470'),
            DeclareLaunchArgument('motor_modbus_port', default_value='1503'),
            OpaqueFunction(function=prepare_params, condition=UnlessCondition(gazebo)),
            Node(
                package='phyto_arm',
                executable='lock_manager.py',
                name='lock_manager',
                parameters=[{'max_moving_winches': 1}],
            ),
            IncludeLaunchDescription(
                PathJoinSubstitution(
                    [FindPackageShare('phyto_sim_utils'), 'launch', 'gps.launch.py']
                ),
                launch_arguments={'gpsd': gpsd, 'gpsd_port': gpsd_port}.items(),
                condition=UnlessCondition(gazebo),
            ),
            IncludeLaunchDescription(
                PathJoinSubstitution(
                    [FindPackageShare('phyto_arm'), 'launch', 'sim_arm_ifcb.launch.py']
                ),
                launch_arguments={
                    'ctd': ctd,
                    'ifcb_winch': 'true',
                    'mock_ifcb': mock_ifcb,
                    'motor_modbus_port': motor_port,
                    'params_file': LaunchConfiguration('params_file'),
                }.items(),
                condition=UnlessCondition(gazebo),
            ),
            IncludeLaunchDescription(
                PathJoinSubstitution(
                    [FindPackageShare('phyto_gazebo'), 'launch', 'winch.launch.py']
                ),
                launch_arguments={
                    'ctd': ctd,
                    'dli_port': LaunchConfiguration('dli_port'),
                    'gpsd': gpsd,
                    'gpsd_port': gpsd_port,
                    'gui': LaunchConfiguration('gui'),
                    'mock_ifcb': mock_ifcb,
                    'motor_modbus_port': motor_port,
                }.items(),
                condition=IfCondition(gazebo),
            ),
            Node(
                package='phyto_sim_utils',
                executable='run_ifcb_emulator',
                name='ifcb_emulator',
                condition=UnlessCondition(mock_ifcb),
                arguments=[LaunchConfiguration('ifcb_replay_dir')],
                output='screen',
                on_exit=[Shutdown()],
            ),
        ]
    )
