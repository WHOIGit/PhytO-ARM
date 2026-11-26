"""Record ROS messages, rolling over to a new bag at regular intervals."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'rosbag_prefix', default_value='/home/hablab/PhytO-ARM/ros/rosbags/phyto-arm'
            ),
            DeclareLaunchArgument('rosbag_duration', default_value='3600'),
            ExecuteProcess(
                cmd=[
                    'ros2',
                    'bag',
                    'record',
                    '--all',
                    '--regex-exclude',
                    '/camera/.*|/ifcb/image|/ifcb/image/.*|/ifcb/in|/ifcb/roi|/ifcb/roi/.*|/rosout',
                    '--output',
                    LaunchConfiguration('rosbag_prefix'),
                    '--max-bag-duration',
                    LaunchConfiguration('rosbag_duration'),
                    '--compression-mode',
                    'file',
                    '--compression-format',
                    'zstd',
                ],
                output='screen',
            ),
        ]
    )
