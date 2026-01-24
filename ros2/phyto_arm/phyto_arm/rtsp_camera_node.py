#!/usr/bin/env python3
"""
RTSP camera node

Actually, this is just a wrapper around the gscam_node, which publishes
frames from arbitrary GStreamer pipelines.

We parse the ROS arguments, build the GStreamer pipeline, and then hand off
execution to gscam_node. The goal is backwards compatibility with the node
formerly used.
"""

import os
import sys


# Ref: https://design.ros2.org/articles/ros_command_line_arguments.html
def parse_ros_args(argv):
    video_stream_url = None
    node_name = 'camera'
    ros_args = []

    # Find --ros-args section
    try:
        ros_args_idx = argv.index('--ros-args')
    except ValueError:
        return video_stream_url, node_name, namespace, ros_args

    # Collect args from --ros-args until -- or end
    i = ros_args_idx + 1
    while i < len(argv) and argv[i] != '--':
        # Check for -p/--param video_stream_url:=<value>
        if argv[i] in ('-p', '--param') and i + 1 < len(argv):
            if argv[i+1].startswith('video_stream_url:='):
                _, _, video_stream_url = argv[i+1].partition(':=')
                i += 2
                continue

        # Check for -r/--remap __node:=<name> or __ns:=<namespace>
        if argv[i] in ('-r', '--remap') and i + 1 < len(argv):
            if argv[i+1].startswith('__node:='):
                _, _, node_name = argv[i+1].partition(':=')
                i += 2
                continue

        ros_args.append(argv[i])
        i += 1

    return video_stream_url, node_name, ros_args


def main():
    video_stream_url, node_name, ros_args = parse_ros_args(sys.argv)
    assert video_stream_url, 'video_stream_url parameter is required'

    # Build GStreamer pipeline
    pipeline = f'uridecodebin uri={video_stream_url} caps=video/x-raw ! videoconvert'

    # Build combined args
    new_args = ['ros2', 'run', 'gscam', 'gscam_node', '--ros-args']
    new_args.extend(ros_args)
    new_args.extend([
        '-p', f'gscam_config:={pipeline}',
        '-p', f'camera_name:={node_name}',
        '-p', f'frame_id:={node_name}',

        # Set the node name
        '-r', f'__node:={node_name}',

        # Prior to image_transport 6.0.0 (ROS Kilted), topic remapping does not
        # really work. So unfortunately we have to enumerate the topics to remap.
        '-r', f'camera/image_raw:={node_name}/image_raw',
        '-r', f'camera/image_raw/compressed:={node_name}/image_raw/compressed',

        # Foxglove breaks from uninitialized CameraInfo messages, so rename
        # the topic so it isn't auto-detected.
        # Ref: https://github.com/orgs/foxglove/discussions/1210
        '-r', f'camera/camera_info:={node_name}/camera_info_invalid',
    ])

    # Execute gscam_node
    os.execvp(new_args[0], new_args)


if __name__ == '__main__':
    main()
