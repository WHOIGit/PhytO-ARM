#!/usr/bin/env python3
import functools
import json
import os
import struct

import rospy

import ifcb.srv as srv

from ifcbclient import IFCBClient

from ds_core_msgs.msg import RawData
from foxglove_msgs.msg import ImageMarkerArray
from geometry_msgs.msg import Point
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool, ColorRGBA
from visualization_msgs.msg import ImageMarker

from .instrumentation import instrument_routine


connection = None

# Apologies for the horrible naming. This is the ROS service handler that
# invokes send_command() and returns an empty response.
def do_command(pub, req):
    sent = send_command(pub, req.command)
    return srv.CommandResponse(success=sent)


# The ROS service handler that runs a routine, optionally with instrumentation
def do_runroutine(pub, req):
    # XXX
    # We always use the 'interactive:start' command instead of 'routine' because
    # the latter does not (as of Apr 21, 2022) provide any feedback when the
    # routine completes.
    #
    # This way, the IFCB will send 'valuechanged:interactive:stopped' when the
    # routine completes.
    #
    # This means that the routines *must* be written to disk at the expected
    # location.

    # Deny path traversal
    if '/' in req.routine:
        return srv.RunRoutineResponse(success=False)

    # Check for beads routines
    if req.routine == 'beads':
        sent = send_command(pub, f'daq:setdatafolder:{rospy.get_param("~data_dir")}/beads')
    else:
        sent = send_command(pub, f'daq:setdatafolder:{rospy.get_param("~data_dir")}')
    if not sent:
        return srv.RunRoutineResponse(success=False)

    # Attempt to load the file
    path = os.path.join(rospy.get_param('~routines_dir'), f'{req.routine}.json')
    try:
        with open(os.path.expanduser(path), 'rb') as f:
            routine = json.load(f)
    except:
        rospy.logerr(f'Could not load {req.routine}.json')
        return srv.RunRoutineResponse(success=False)

    # Optionally instrument the routine
    if req.instrument:
        routine = instrument_routine(routine, name=req.routine)

    # Encode and send the routine
    encoded = json.dumps(routine, separators=(',', ':'))  # less whitespace
    sent = send_command(pub, f'interactive:load:{encoded}')
    sent = sent and send_command(pub, 'interactive:start')

    return srv.RunRoutineResponse(success=sent)
        


# Send a command to the IFCB host and publish it to ROS
def send_command(pub, command):
    try:
        connection.relay_message_to_host(command)
    except Exception:
        rospy.logerr('Failed to send IFCB command "%s"', command, exc_info=True)
        return False

    # Construct the message we are going to publish
    msg = RawData()
    msg.header.stamp = msg.ds_header.io_time = rospy.Time.now()
    msg.data_direction = RawData.DATA_OUT
    msg.data = command.encode()
    pub.publish(msg)
    return True


# Publish incoming "raw" messages from the IFCB as ROS messages
def on_any_message(pub, data):
    sender_id, smsgsrc, seqno, data = data

    # Publish a copy of the incoming message
    msg = RawData()
    msg.header.seq = seqno
    msg.header.stamp = msg.ds_header.io_time = rospy.Time.now()
    msg.data_direction = RawData.DATA_IN
    msg.data = data.encode()
    pub.publish(msg)


# Reset the data folder to the configured data directory
def on_interactive_stopped(pub, *_):
    if not send_command(pub, f'daq:setdatafolder:{rospy.get_param("~data_dir")}'):
        raise ConnectionError("Unable to reset IFCB data dir.")

def on_triggerimage(pub, _, image):
    # The image data should be in PNG format, which is an allowed ROS image type
    assert image.startswith(b'\x89PNG\x0D\x0A\x1A\x0A')  # magic bytes

    msg = CompressedImage()
    msg.header.stamp = rospy.Time.now()
    msg.format = 'png'
    msg.data = image
    pub.publish(msg)


def on_triggercontent(roi_pub, mkr_pub, _, daq, rois):
    timestamp = rospy.Time.now()

    # TODO: Publish DAQ messages too
    pass

    # Delegate publishing of ROI images and markers to on_triggerrois
    on_triggerrois(roi_pub, mkr_pub, _, rois, timestamp=timestamp)


def on_triggerrois(roi_pub, mkr_pub, _, rois, *, timestamp=None):
    timestamp = timestamp or rospy.Time.now()
    markers = ImageMarkerArray()
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

        # Add a marker to the array
        mkr = ImageMarker()
        mkr.header.stamp = timestamp
        mkr.type = ImageMarker.POLYGON
        mkr.scale = 1.0
        mkr.outline_color = ColorRGBA(0, 1, 1, 1)
        mkr.points = [
            Point(left,     top,     0),
            Point(left + w, top,     0),
            Point(left + w, top + h, 0),
            Point(left,     top + h, 0),
        ]
        markers.markers.append(mkr)

    # Publish all the markers together
    mkr_pub.publish(markers)


# Create a client and wire its lifecycle and ROS message callbacks
def create_client(publishers):
    client = IFCBClient(
        f'ws://{rospy.get_param("~address")}:{rospy.get_param("~port", 8092)}/ifcbHub',
        rospy.get_param('~serial'))

    # Publish registration status and log connection progress
    client.on_ready(
        lambda ready: publishers['status'].publish(Bool(data=ready))
    )
    client.on_connecting(lambda: rospy.loginfo('Connecting to IFCB...'))
    client.on_started(lambda _: rospy.loginfo('Established connection to IFCB'))
    client.on_close(lambda *_: rospy.loginfo('IFCB connection closed'))
    client.on_reconnect(lambda: rospy.loginfo('IFCB connection restored'))
    client.on_retry(lambda delay: rospy.logwarn(
        'IFCB connection unavailable; retrying in %g seconds...', delay
    ))
    client.on_error(lambda error: rospy.logwarn('IFCB error: %s', error))

    # Forward IFCB messages to their ROS publishers
    client.on_any_message(functools.partial(on_any_message, publishers['rx']))
    for pattern, callback in [
        (('triggerimage',), functools.partial(on_triggerimage, publishers['img'])),
        (('triggercontent',), functools.partial(on_triggercontent, publishers['roi'], publishers['mkr'])),
        (('triggerrois',), functools.partial(on_triggerrois, publishers['roi'], publishers['mkr'])),
        (('valuechanged', 'interactive', 'stopped'), functools.partial(on_interactive_stopped, publishers['tx'])),
    ]:
        client.on(pattern, callback)
    return client


def main():
    global connection
    rospy.init_node('ifcb', anonymous=True)

    # Publishers for raw messages coming in and out of the IFCB
    rx_pub = rospy.Publisher('~in',  RawData, queue_size=5)
    tx_pub = rospy.Publisher('~out', RawData, queue_size=5)

    # Publishers for images, ROI images, and ROI markers
    img_pub = rospy.Publisher('~image/compressed', CompressedImage,
                              queue_size=5)
    roi_pub = rospy.Publisher('~roi/image/compressed', CompressedImage,
                              queue_size=5)
    mkr_pub = rospy.Publisher('~roi/markers', ImageMarkerArray, queue_size=5)

    # Publisher for connection status (latched so new subscribers get current state)
    status_pub = rospy.Publisher('~connected', Bool, queue_size=1, latch=True)

    # Bundle publishers for the client callbacks
    publishers = {
        'rx': rx_pub,
        'tx': tx_pub,
        'img': img_pub,
        'roi': roi_pub,
        'mkr': mkr_pub,
        'status': status_pub
    }


    # Let the client manage connection attempts and readiness
    connection = create_client(publishers)
    rospy.on_shutdown(connection.stop)

    # Create a ROS service for sending commands
    rospy.Service(
        '~command',
        srv.Command,
        functools.partial(do_command, tx_pub),
    )

    # Create a ROS service for running routines
    rospy.Service(
        '~routine',
        srv.RunRoutine,
        functools.partial(do_runroutine, tx_pub),
    )

    connection.connect(wait_until_ready=False)

    rospy.loginfo('IFCB node started, attempting connection to IFCB...')

    # Keep the program alive while other threads work
    try:
        rospy.spin()
    finally:
        connection.stop()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
