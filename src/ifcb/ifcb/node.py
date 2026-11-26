#!/usr/bin/env python3
import functools
import json
import os
import random
import struct
import traceback

import rclpy

import ifcb_msgs.srv as srv

from foxglove_msgs.msg import Color, ImageAnnotations, Point2, PointsAnnotation
from ifcbclient import IFCBClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool
from wr2_msgs.msg import RawData

from .instrumentation import instrument_routine


connection = None

# Apologies for the horrible naming. This is the ROS service handler that
# invokes send_command() and returns an empty response.
def do_command(node, pub, req, response):
    response.success = send_command(node, pub, req.command)
    return response


def load_routine(node, name):
    path = os.path.join(node.get_parameter('routines_dir').value, f'{name}.json')
    with open(os.path.expanduser(path), 'rb') as f:
        return json.load(f)


# The ROS service handler that runs a routine, optionally with instrumentation
def do_runroutine(node, pub, req, response, load_routine):
    # XXX
    # We always use the 'interactive:start' command instead of 'routine' because
    # the latter does not (as of Apr 21, 2022) provide any feedback when the
    # routine completes.
    #
    # This way, the IFCB will send 'valuechanged:interactive:stopped' when the
    # routine completes.

    # Deny path traversal
    if '/' in req.routine:
        return response

    # Check for beads routines
    if req.routine == 'beads':
        sent = send_command(node, pub, f'daq:setdatafolder:{node.get_parameter("data_dir").value}/beads')
    else:
        sent = send_command(node, pub, f'daq:setdatafolder:{node.get_parameter("data_dir").value}')
    if not sent:
        return response

    # Look up the routine before adding instrumentation
    try:
        routine = load_routine(req.routine)
    except (KeyError, OSError, ValueError):
        node.get_logger().error(f'Could not load routine {req.routine}')
        return response

    # Optionally instrument the routine
    if req.instrument:
        routine = instrument_routine(routine, name=req.routine)

    # Encode and send the routine
    encoded = json.dumps(routine, separators=(',', ':'))  # less whitespace
    sent = send_command(node, pub, f'interactive:load:{encoded}')
    sent = sent and send_command(node, pub, 'interactive:start')

    response.success = sent
    return response



# Send a command to the IFCB host and publish it to ROS
def send_command(node, pub, command):
    try:
        connection.relay_message_to_host(command)
    except Exception:
        node.get_logger().error(f'Failed to send IFCB command "{command}"\n{traceback.format_exc()}')
        return False

    # Construct the message we are going to publish
    msg = RawData()
    msg.io_stamp = node.get_clock().now().to_msg()
    msg.data_direction = RawData.DIRECTION_DATA_OUT
    msg.data = command.encode()
    pub.publish(msg)
    return True


# Publish incoming "raw" messages from the IFCB as ROS messages
def on_any_message(node, publishers, data):
    sender_id, smsgsrc, seqno, data = data

    # Publish a copy of the incoming message
    msg = RawData()
    msg.io_stamp = node.get_clock().now().to_msg()
    msg.data_direction = RawData.DIRECTION_DATA_IN
    msg.data = data.encode()
    publishers['rx'].publish(msg)

    # Retain the last reported routine state for runners that subscribe later
    if data == 'valuechanged:interactive:started':
        publishers['run'].publish(Bool(data=True))
    elif data == 'valuechanged:interactive:stopped':
        publishers['run'].publish(Bool(data=False))


# Reset the data folder to the configured data directory
def on_interactive_stopped(node, pub, *_):
    if not send_command(node, pub, f'daq:setdatafolder:{node.get_parameter("data_dir").value}'):
        raise ConnectionError("Unable to reset IFCB data dir.")

def on_triggerimage(node, pub, _, image):
    # The image data should be in PNG format, which is an allowed ROS image type
    assert image.startswith(b'\x89PNG\x0D\x0A\x1A\x0A')  # magic bytes

    msg = CompressedImage()
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.format = 'png'
    msg.data = image
    pub.publish(msg)


def on_triggercontent(node, roi_pub, mkr_pub, _, daq, rois):
    timestamp = node.get_clock().now().to_msg()

    # TODO: Publish DAQ messages too
    pass

    # Delegate publishing of ROI images and markers to on_triggerrois
    on_triggerrois(node, roi_pub, mkr_pub, _, rois, timestamp=timestamp)


def on_triggerrois(node, roi_pub, mkr_pub, _, rois, *, timestamp=None):
    timestamp = timestamp or node.get_clock().now().to_msg()
    markers = ImageAnnotations()
    for i, (left, top, image) in enumerate(rois):
        # IFCB does not give us the width and height so we must extract from
        # the IHDR chunks.
        magic, chlen, chtype, w, h = struct.unpack_from('>8sL4sLL', image)
        assert magic == b'\x89PNG\x0D\x0A\x1A\x0A'
        assert chtype == b'IHDR'

        # Publish the ROI image itself
        roi = CompressedImage()
        roi.header.stamp = timestamp
        roi.format = 'png'
        roi.data = image
        roi_pub.publish(roi)

        # Add a polygon annotation for the ROI bounding box
        mkr = PointsAnnotation()
        mkr.timestamp = timestamp
        mkr.type = PointsAnnotation.LINE_LOOP
        mkr.thickness = 1.0
        mkr.outline_color = Color(r=0.0, g=1.0, b=1.0, a=1.0)
        mkr.points = [
            Point2(x=float(left),     y=float(top)),
            Point2(x=float(left + w), y=float(top)),
            Point2(x=float(left + w), y=float(top + h)),
            Point2(x=float(left),     y=float(top + h)),
        ]
        markers.points.append(mkr)

    # Publish all the markers together
    mkr_pub.publish(markers)


# Create a client and wire its lifecycle and ROS message callbacks
def create_client(node, publishers):
    port = node.get_parameter('port').value
    client = IFCBClient(
        f'ws://{node.get_parameter("address").value}:{port}/ifcbHub',
        node.get_parameter('serial').value)

    # Publish registration status and log connection progress
    def on_ready(ready):
        # Stopping the client can report disconnection after ROS shutdown.
        if node.context.ok():
            publishers['conn'].publish(Bool(data=ready))

    client.on_ready(on_ready)
    client.on_connecting(lambda: node.get_logger().info('Connecting to IFCB...'))
    client.on_started(lambda _: node.get_logger().info('Established connection to IFCB'))
    client.on_close(lambda *_: node.get_logger().info('IFCB connection closed'))
    client.on_reconnect(lambda: node.get_logger().info('IFCB connection restored'))
    client.on_retry(lambda delay: node.get_logger().warning(
        f'IFCB connection unavailable; retrying in {delay:g} seconds...'
    ))
    client.on_error(lambda error: node.get_logger().warning(f'IFCB error: {error}'))

    # Forward IFCB messages to their ROS publishers
    client.on_any_message(functools.partial(on_any_message, node, publishers))
    for pattern, callback in [
        (('triggerimage',), functools.partial(on_triggerimage, node, publishers['img'])),
        (('triggercontent',), functools.partial(on_triggercontent, node, publishers['roi'], publishers['mkr'])),
        (('triggerrois',), functools.partial(on_triggerrois, node, publishers['roi'], publishers['mkr'])),
        (('valuechanged', 'interactive', 'stopped'), functools.partial(on_interactive_stopped, node, publishers['tx'])),
    ]:
        client.on(pattern, callback)
    return client


def configure_node(node: Node, client_factory, routine_loader) -> None:
    global connection

    # Publishers for raw messages coming in and out of the IFCB
    rx_pub = node.create_publisher(RawData, '~/in', 32)
    tx_pub = node.create_publisher(RawData, '~/out', 5)

    # Publishers for images, ROI images, and ROI markers
    img_pub = node.create_publisher(CompressedImage, '~/image/compressed', 5)
    roi_pub = node.create_publisher(CompressedImage, '~/roi/image/compressed', 5)
    mkr_pub = node.create_publisher(ImageAnnotations, '~/roi/markers', 5)

    # Publisher for connection status (latched so new subscribers get current state)
    latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    conn_pub = node.create_publisher(Bool, '~/connected', latched)

    # Bundle publishers for the client callbacks
    publishers = {
        'run': node.create_publisher(Bool, '~/is_routine_running', latched),
        'rx': rx_pub,
        'tx': tx_pub,
        'img': img_pub,
        'roi': roi_pub,
        'mkr': mkr_pub,
        'conn': conn_pub
    }


    # Let the client manage connection attempts and readiness
    connection = client_factory(node, publishers)

    # Keep a second request from replacing a routine between load and start
    requests = MutuallyExclusiveCallbackGroup()
    if routine_loader is None:
        routine_loader = functools.partial(load_routine, node)

    # Create a ROS service for sending commands
    node.create_service(
        srv.Command, '~/command', functools.partial(do_command, node, tx_pub),
        callback_group=requests)

    # Create a ROS service for running routines
    node.create_service(
        srv.RunRoutine, '~/routine',
        functools.partial(do_runroutine, node, tx_pub, load_routine=routine_loader),
        callback_group=requests)

    connection.connect(wait_until_ready=False)
    node.get_logger().info('IFCB node started, attempting connection to IFCB...')


def main(client_factory=create_client, routine_loader=None, args=None) -> None:
    rclpy.init(args=args)
    node = rclpy.create_node(f'ifcb_{os.getpid()}_{random.getrandbits(32):08x}')
    executor = MultiThreadedExecutor(num_threads=2)
    try:
        node.declare_parameters('', [
            ('address', Parameter.Type.STRING),
            ('data_dir', Parameter.Type.STRING),
            ('port', 8092),
            ('routines_dir', Parameter.Type.STRING),
            ('serial', Parameter.Type.STRING),
        ])

        configure_node(node, client_factory, routine_loader)
        # Keep the program alive while other threads work
        rclpy.spin(node, executor=executor)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if connection is not None:
            connection.stop()
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
