from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('address'),
        DeclareLaunchArgument('data_dir', default_value='/data/ifcbdata/testing'),
        DeclareLaunchArgument('routines_dir'),
        DeclareLaunchArgument('serial'),
        Node(package='ifcb', executable='ifcb', name='ifcb', on_exit=[Shutdown()],
             parameters=[{key: LaunchConfiguration(key)
                          for key in ('address', 'data_dir', 'routines_dir', 'serial')}]),
        Node(package='phyto_arm', executable='ifcb_runner.py', name='ifcb_runner',
             namespace='arm_ifcb', output='screen', on_exit=[Shutdown()],
             parameters=[{
                 'ifcb_maintenance.bead_interval': 0,
                 'ifcb_maintenance.cartridge_debubble_interval': 0,
                 'ifcb_maintenance.clean_interval': 0,
                 'ifcb_maintenance.flush_after_sample': False,
             }]),
    ])
