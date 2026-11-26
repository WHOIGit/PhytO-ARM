"""Record ROS messages, rolling over to a new bag at regular intervals."""

from datetime import datetime

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    timestamp = datetime.now().strftime('%Y-%m-%d-%H-%M-%S-%f')
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
                    '--include-hidden-topics',
                    '--exclude',
                    '/camera/.*|/ifcb/image|/ifcb/image/.*|/ifcb/in|/ifcb/roi|/ifcb/roi/.*'
                    # Keep action feedback and status among otherwise hidden topics
                    '|.*/_(?!action/(feedback|status)$).*',
                    '--output',
                    [LaunchConfiguration('rosbag_prefix'), '_', timestamp],
                    '--max-bag-duration',
                    LaunchConfiguration('rosbag_duration'),
                    '--storage',
                    'mcap',
                    '--storage-preset-profile',
                    'zstd_fast',
                ],
                output='screen',
            ),
        ]
    )
