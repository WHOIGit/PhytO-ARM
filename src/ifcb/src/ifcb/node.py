#!/usr/bin/env python3
import functools
import json
import os
import struct
import sys
import threading

import rospy

import ifcb.srv as srv

from ifcbclient import IFCBClient

from ds_core_msgs.msg import RawData
from foxglove_msgs.msg import ImageMarkerArray
from geometry_msgs.msg import Point
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import ImageMarker

from .instrumentation import instrument


ifcb_ready = threading.Event()
initial_datafolder = None

# Apologies for the horrible naming. This is the ROS service handler that
# invokes send_command() and returns an empty response.
def do_command(client, pub, req):
    send_command(client, pub, req.command)
    return srv.CommandResponse()


# The ROS service handler that runs a routine, optionally with instrumentation
def do_runroutine(client, pub, req):
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
    assert initial_datafolder is not None
    if req.routine == 'beads':
        send_command(client, pub, f'daq:setdatafolder:{initial_datafolder}/beads')
    else:
        send_command(client, pub, f'daq:setdatafolder:{initial_datafolder}')

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
        routine = instrument(routine, routine=req.routine)

    # Encode and send the routine
    encoded = json.dumps(routine, separators=(',', ':'))  # less whitespace
    send_command(client, pub, f'interactive:load:{encoded}')
    send_command(client, pub, 'interactive:start')

    return srv.RunRoutineResponse(success=True)


# Send a command to the IFCB host and publish it to ROS
def send_command(client, pub, command):
    # Wait for the IFCB to be ready; this prevents sending a command too early
    ifcb_ready.wait()

    # Construct the message we are going to publish
    msg = RawData()
    msg.header.stamp = msg.ds_header.io_time = rospy.Time.now()
    msg.data_direction = RawData.DATA_OUT
    msg.data = command.encode()

    # Send the command
    client.relay_message_to_host(command)

    # Publish the message after sending it, so that the timestamp is more
    # accurate.
    pub.publish(msg)


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


# Callback for the "startedAsClient" message from the IFCB
def on_started(client, pub, *args, **kwargs):
    rospy.loginfo('Established connection to the IFCB')

    # Allow any queued commands to proceed now that the IFCB is available
    ifcb_ready.set()

# Reset the data folder to the initial value in case bead run
# was the last command set
def on_interactive_stopped(client, pub, *_):
    send_command(client, pub, f'daq:setdatafolder:{initial_datafolder}')

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
    for i, (top, left, image) in enumerate(rois):
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

# Backup the initial value of the data folder so we can restore it after bead runs
def on_datafolder(_event, _type, value):
    global initial_datafolder
    # Reject updates once set
    if initial_datafolder is None:
        initial_datafolder = value


def main():
    rospy.init_node('ifcb', anonymous=True)

    # Publishers for raw messages coming in and out of the IFCB
    rx_pub = rospy.Publisher('~in',  RawData, queue_size=5)
    tx_pub = rospy.Publisher('~out', RawData, queue_size=5)

    # Publishers for images, ROI images, and ROI markers
    img_pub = rospy.Publisher('~image', CompressedImage, queue_size=5)
    roi_pub = rospy.Publisher('~roi/image', CompressedImage, queue_size=5)
    mkr_pub = rospy.Publisher('~roi/markers', ImageMarkerArray, queue_size=5)

    # Create an IFCB websocket API client
    client = IFCBClient(
        f'ws://{rospy.get_param("~address")}'\
            f':{rospy.get_param("~port", 8092)}/ifcbHub',
        rospy.get_param('~serial'),
    )

    # Publish all messages that come in, prior to being parsed.
    # Note: This relies on internal implementation details of pyifcbclient.
    client.hub_connection.on('messageRelayed',
        functools.partial(on_any_message, rx_pub))

    # Set up a callback for when the connection starts.
    # Note: This too relies on internal implementation details of pyifcbclient.
    client.hub_connection.on('startedAsClient',
        functools.partial(on_started, client, tx_pub))

    # Set up callbacks for trigger images and ROIs
    client.on(('triggerimage',),
              functools.partial(on_triggerimage, img_pub))
    client.on(('triggercontent',),
              functools.partial(on_triggercontent, roi_pub, mkr_pub))
    client.on(('triggerrois',),
              functools.partial(on_triggerrois, roi_pub, mkr_pub))

    # Set up callbacks for datafolder switching
    client.on(('valuechanged','setdatafolder',), on_datafolder)
    client.on(('valuechanged','interactive','stopped',),
              functools.partial(on_interactive_stopped, client, tx_pub))

    # Create a ROS service for sending commands
    rospy.Service(
        '~command',
        srv.Command,
        functools.partial(do_command, client, tx_pub),
    )

    # Create a ROS service for running routines
    rospy.Service(
        '~routine',
        srv.RunRoutine,
        functools.partial(do_runroutine, client, tx_pub),
    )

    # Connect the client
    client.connect()

    # Keep the program alive while other threads work
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
