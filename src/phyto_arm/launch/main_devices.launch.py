"""Launch the GPS component and the configured RTSP cameras."""
import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue


def launch_devices(context):
    with open(LaunchConfiguration('config_file').perform(context)) as f:
        config = yaml.safe_load(f)

    gps = ComposableNode(package='gpsd_client', plugin='gpsd_client::GPSDClientComponent',
                         name='gps', parameters=[config.get('gps') or {}],
                         remappings=[('fix', '~/fix'), ('extended_fix', '~/extended_fix')])
    nodes = [ComposableNodeContainer(package='rclcpp_components', executable='component_container',
                                     name='gps_container', namespace='',
                                     composable_node_descriptions=[gps])]
    cameras = config.get('camera') or {}
    for name, camera in cameras.items():
        url = (camera or {}).get('video_stream_url', '')
        if not url:
            continue
        nodes.append(
            Node(
                package='gscam',
                executable='gscam_node',
                name=name,
                namespace='camera',
                parameters=[{
                    'camera_name': ParameterValue(name, value_type=str),
                    'frame_id': ParameterValue(name, value_type=str),
                    'gscam_config': f'uridecodebin uri="{url}" caps=video/x-raw ! videoconvert',
                }],
                remappings=[
                    ('camera/image_raw', '~/image/do_not_use'),
                    ('camera/image_raw/compressed', '~/image/compressed'),
                    # Keep uncalibrated camera info out of Foxglove's image panels.
                    ('camera/camera_info', '~/camera_info_invalid'),
                ],
            )
        )
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('config_file'),
        OpaqueFunction(function=launch_devices),
    ])
