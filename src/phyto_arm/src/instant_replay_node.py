#!/usr/bin/env python3
'''
This node captures a rolling buffer of video frames from an RTSP camera
in memory and can publish them on request.

Historical frames from the instant replay may be interleaved with incoming
frames.

Consider the memory requirements of this node. A 24-bit 1080p frame takes about
6 MB. Frames are stored uncompressed.
'''
import collections
import functools
import threading

import rospy

from sensor_msgs.msg import Image

import phyto_arm.srv as srv


SECONDS_BEFORE = SECONDS_AFTER = 5
MAX_FRAMERATE = 30


history_lock = threading.Lock()
history = collections.deque(maxlen=MAX_FRAMERATE*SECONDS_BEFORE)


shutter_time = None


def cmd_capture(publisher, request):
    global shutter_time
    shutter_time = rospy.Time.now()

    rospy.loginfo('Publishing instant replay')

    with history_lock:
        for f in history:
            if shutter_time - f.header.stamp < rospy.Duration(SECONDS_BEFORE):
                publisher.publish(f)

    return srv.InstantReplayCaptureCmdResponse()


def on_frame(publisher, frame):
    if shutter_time:
        if frame.header.stamp - shutter_time < rospy.Duration(SECONDS_AFTER):
            publisher.publish(frame)

    with history_lock:
        history.append(frame)


def main():
    # Initialize this node
    rospy.init_node('instant_replay', anonymous=True)

    # Publisher for instant replays
    publisher = rospy.Publisher(
        '~image',
        Image,
        queue_size=2*history.maxlen
    )

    # Subscribe to frames from the camear
    rospy.Subscriber(
        '/rtsp_camera_relay/image',
        Image,
        functools.partial(on_frame, publisher)
    )

    # Declare our service for capturing an instant replay
    service = rospy.Service(
        f'{rospy.get_name()}/capture',
        srv.InstantReplayCaptureCmd,
        functools.partial(cmd_capture, publisher)
    )

    # Other activities should continue in 
    service.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
