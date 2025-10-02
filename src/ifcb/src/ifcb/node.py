#!/usr/bin/env python3
import functools
import json
import os
import struct
import threading
import time

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


ifcb_ready = threading.Event()
ifcb_client = None  # Global client reference for command sending

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
    # Wait for the IFCB to be ready; this prevents sending a command too early
    if not ifcb_ready.wait(5.0):
        rospy.logwarn('IFCB not ready within 5s of attempting to send a command; aborting.')
        return False


    # Construct the message we are going to publish
    msg = RawData()
    msg.header.stamp = msg.ds_header.io_time = rospy.Time.now()
    msg.data_direction = RawData.DATA_OUT
    msg.data = command.encode()

    # Send the command with error handling
    try:
        ifcb_client.relay_message_to_host(command)
        # Publish the message after sending it, so that the timestamp is more accurate
        pub.publish(msg)
        return True
    except Exception:
        rospy.logerr(f'Failed to send command "{command}"', exc_info=True)
        # Mark connection as lost to trigger reconnection
        ifcb_ready.clear()
        return False


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
def on_started(*args, **kwargs):
    rospy.logwarn('Established connection to the IFCB')
    ifcb_ready.set()

# Callback for when connection is lost
def on_connection_lost(status_pub):
    rospy.logwarn('IFCB connection lost, attempting to reconnect...')
    ifcb_ready.clear()

    # Publish connection status
    status_pub.publish(Bool(data=False))

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


# Manages IFCB client connection with automatic retry on failure.
def connection_manager(publishers, retry_interval=5, max_retry_interval=60):
    global ifcb_client
    current_retry_interval = retry_interval

    while not rospy.is_shutdown():
        if ifcb_ready.is_set():
            rospy.sleep(5)
            continue
        try:
            # Create a new IFCB client instance for each connection attempt
            rospy.loginfo('Creating new IFCB client...')
            try:
                ifcb_client = IFCBClient(
                    f'ws://{rospy.get_param("~address")}'\
                        f':{rospy.get_param("~port", 8092)}/ifcbHub',
                    rospy.get_param('~serial'),
                )
            except Exception as client_error:
                raise ConnectionError(f"Failed to create IFCB client: {client_error}")

            # Set up callbacks for the new client
            ifcb_client.on_started(on_started)
            ifcb_client.on_reconnect(
                        functools.partial(on_connection_lost, publishers['status']))
            ifcb_client.on_any_message(
                        functools.partial(on_any_message, publishers['rx']))
            ifcb_client.on(('triggerimage',),
                        functools.partial(on_triggerimage, publishers['img']))
            ifcb_client.on(('triggercontent',),
                        functools.partial(on_triggercontent, publishers['roi'], publishers['mkr']))
            ifcb_client.on(('triggerrois',),
                        functools.partial(on_triggerrois, publishers['roi'], publishers['mkr']))
            ifcb_client.on(('valuechanged','interactive','stopped',),
                        functools.partial(on_interactive_stopped, publishers['tx']))

            try:
                ifcb_client.connect()
            except Exception as connect_error:
                raise ConnectionError(f"Client connection error: {connect_error}")

            # Wait a bit to see if connection establishes
            time.sleep(2)

            # Check if we got the startedAsClient callback
            if ifcb_ready.is_set():
                rospy.loginfo('IFCB connection successful')
                # Publish connection status
                publishers['status'].publish(Bool(data=True))
                current_retry_interval = retry_interval  # Reset retry interval on success
            else:
                raise ConnectionError("No ready message received")

        except ConnectionError as connect_error:
            rospy.logwarn(f"Unable to establish connection to IFCB: {connect_error}")

            # Clean up the failed client
            ifcb_client = None

            # Exponential backoff with jitter
            rospy.loginfo(f'Retrying IFCB connection in {current_retry_interval} seconds...')
            time.sleep(current_retry_interval)

            # Increase retry interval, but cap it at max_retry_interval
            current_retry_interval = min(current_retry_interval * 1.5, max_retry_interval)


def main():
    rospy.init_node('ifcb', anonymous=True)

    # Publishers for raw messages coming in and out of the IFCB
    rx_pub = rospy.Publisher('~in',  RawData, queue_size=5)
    tx_pub = rospy.Publisher('~out', RawData, queue_size=5)

    # Publishers for images, ROI images, and ROI markers
    img_pub = rospy.Publisher('~image', CompressedImage, queue_size=5)
    roi_pub = rospy.Publisher('~roi/image', CompressedImage, queue_size=5)
    mkr_pub = rospy.Publisher('~roi/markers', ImageMarkerArray, queue_size=5)

    # Publisher for connection status (latched so new subscribers get current state)
    status_pub = rospy.Publisher('~connected', Bool, queue_size=1, latch=True)

    # Bundle publishers for the connection manager
    publishers = {
        'rx': rx_pub,
        'tx': tx_pub,
        'img': img_pub,
        'roi': roi_pub,
        'mkr': mkr_pub,
        'status': status_pub
    }


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

    # Publish initial disconnected status (will be updated when connection succeeds)
    status_pub.publish(Bool(data=False))

    # Start connection manager in a separate thread
    # It will handle connection retry logic and client creation
    connection_thread = threading.Thread(
        target=connection_manager,
        args=(publishers,),
        daemon=True
    )
    connection_thread.start()

    rospy.loginfo('IFCB node started, attempting connection to IFCB...')

    # Keep the program alive while other threads work
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
