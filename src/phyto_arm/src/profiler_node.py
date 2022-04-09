#!/usr/bin/env python3
import functools
import importlib
import io
import itertools
import math

import numpy as np
import rospy
import scipy.interpolate, scipy.signal

from ds_sensor_msgs.msg import DepthPressure

from phyto_arm.msg import MoveToDepthActionGoal, MoveToDepthActionResult, \
                          DepthProfile


# Flag indicating whether a move_to_depth action is being performed and hence
# depth messages should be recorded.
is_recording = False

# Lists of accumulated depth and data messages while recording.
data_msgs, depth_msgs = [], []

# Data topic Subscriber and associated message field
data_field, data_sub = None, None


def on_action_start(action_msg):
    global is_recording, data_msgs, data_field, data_sub, depth_msgs

    # Subscribe to the desired topic. We do this here so that the operator can
    # update the params between casts.
    data_field = rospy.get_param('~data_field')
    data_topic = rospy.get_param('~data_topic')
    if data_sub is None or data_sub.name != data_topic:
        if data_sub is not None:
            rospy.loginfo(f'Unsubscribing from {data_sub.name}')
            data_sub.unregister()

        rospy.loginfo(f'Subscribing to {data_topic}')
        data_sub = rospy.Subscriber(data_topic, rospy.AnyMsg,
            lambda m: is_recording and data_msgs.append(m))

    # Clear buffers and start recording
    data_msgs, depth_msgs = [], []
    is_recording = True


def on_action_stop(pub, action_msg):
    global is_recording, data_field, data_msgs, data_sub, depth_msgs

    # Stop recording new data points
    is_recording = False

    if not data_msgs or not depth_msgs:
        rospy.logerr('Did not receive any data')
        return

    # Since the data topic is dynamic, we need to re-interpret the data as the
    # correct type. This is a bit hacky, but described here:
    # http://schulz-m.github.io/2016/07/18/rospy-subscribe-to-any-msg-type/
    pkg, _, typename = data_msgs[0]._connection_header['type'].partition('/')
    msg_class = getattr(importlib.import_module(f'{pkg}.msg'), typename)

    parsed_data_msgs = []
    for raw_msg in data_msgs:
        buf = io.BytesIO()
        raw_msg.serialize(buf)
        parsed_data_msgs.append(msg_class().deserialize(buf.getvalue()))

    # Group messages based on header time to correlate them. Assumptions:
    #   1. The configured message topic has a std_msg/Header with timestamp.
    #   2. The timestamps match up exactly (true of AML and RBR messages).
    #   3. The data message is not a DepthPressure message.
    groups = {}
    for m in itertools.chain(depth_msgs, parsed_data_msgs):
        groups.setdefault(m.header.stamp, {})[type(m)] = m

    # Create an array of depth x data, discarding any mismatched pairs
    data = np.array([
        (v[DepthPressure].depth, getattr(v[msg_class], data_field))
        for v in groups.values()
        if len(v) == 2 and \
            v[DepthPressure].depth != DepthPressure.DEPTH_PRESSURE_NO_DATA
    ])

    if len(groups) != data.shape[0]:
        rospy.logwarn(f'Discarded {len(groups) - data.shape[0]} data points')

    # Create a function that linearly interpolates between points
    f = scipy.interpolate.interp1d(data[:,0], data[:,1])

    # Sample the interpolated function at a regular interval
    res = rospy.get_param('~resolution')
    x = np.arange(data[:,0].min(), data[:,0].max(), res)
    y = f(x)

    # TODO: Noise elimination, smoothing, etc.

    # Publish the resampled profile data
    profile = DepthProfile()
    profile.header.stamp = action_msg.header.stamp
    profile.data_topic = data_sub.name
    profile.data_field = data_field
    profile.depths = x
    profile.values = y
    pub.publish(profile)


def main():
    rospy.init_node('profiler', anonymous=True)

    # Publish DepthProfile messages
    pub = rospy.Publisher('~', DepthProfile, queue_size=1)

    # Accumulate depth messages while recording
    rospy.Subscriber('/ctd/depth', DepthPressure,
        lambda m: is_recording and depth_msgs.append(m))

    # Subscribe to the action start/stop messages
    rospy.Subscriber('/winch/move_to_depth/goal', MoveToDepthActionGoal,
        on_action_start)
    rospy.Subscriber('/winch/move_to_depth/result', MoveToDepthActionResult,
        functools.partial(on_action_stop, pub))

    rospy.spin()


if __name__ == '__main__':
    main()
