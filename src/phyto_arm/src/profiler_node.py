#!/usr/bin/env python3
import functools
import importlib
import io
import itertools
import math
import threading

import numpy as np
import rospy
import scipy.interpolate, scipy.signal

from ds_sensor_msgs.msg import DepthPressure

from phyto_arm.msg import MoveToDepthActionGoal, MoveToDepthActionResult, \
                          DepthProfile


# This semaphore protects us from a data race when we get new data/depth
# messages.
NUM_COLLECTORS = 2
S = threading.Semaphore(value=NUM_COLLECTORS)

#
action_stopped_ev = threading.Event()


# Flag indicating whether a move_to_depth action is being performed and hence
# depth messages should be recorded.
is_recording = False

# Lists of accumulated depth and data messages while recording
data_msgs, depth_msgs = [], []

# Data topic Subscriber and associated message field
data_field, data_sub = None, None


# Non-racy way of recording incoming messages to a buffer; this acquires the
# (global) semaphore S which allows multiple threads to record, but the
# processing thread can acquire exclusive access to the buffers.
def record_data(dest, msg):
    S.acquire()
    if is_recording:
        dest.append(msg)
    S.release()


def on_action_start(action_msg):
    global is_recording, data_field, data_sub

    # Wait a short while to make sure that the 'result' message from the
    # previous goal is processed first.
    #
    # The wait() call will block until the Event object's internal flag is set,
    # or until the timeout occurs. The internal flag is set in the 'result'
    # message handler, so this should block until we get a result.
    action_stopped_ev.wait(timeout=1.0)

    # Now clear the Event's internal flag so that the next time through it
    # blocks again.
    action_stopped_ev.clear()

    # Acquire exclusive access to the buffers
    for _ in range(NUM_COLLECTORS):
        S.acquire()

    # Clear them
    del data_msgs[:], depth_msgs[:]

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
            functools.partial(record_data, data_msgs))

    # Start recording
    is_recording = True

    # Release exclusive access
    for _ in range(NUM_COLLECTORS):
        S.release()


def on_action_stop(pub, action_msg):
    global is_recording

    # Acquire exclusive access to the buffers
    for _ in range(NUM_COLLECTORS):
        S.acquire()
    is_recording = False

    # Since the data topic is dynamic, we need to re-interpret the data as the
    # correct type. This is a bit hacky, but described here:
    # http://schulz-m.github.io/2016/07/18/rospy-subscribe-to-any-msg-type/
    if data_msgs:
        pkg, _, clsname = data_msgs[0]._connection_header['type'].partition('/')
        msg_class = getattr(importlib.import_module(f'{pkg}.msg'), clsname)

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

    if data.shape[0] < 2:
        rospy.logerr('Did not receive enough data')
        for _ in range(NUM_COLLECTORS):
            S.release()
        return

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
    profile.goal_uuid = action_msg.result.uuid
    profile.data_topic = data_sub.name
    profile.data_field = data_field
    profile.depths = x
    profile.values = y
    pub.publish(profile)

    # Set the Event's internal flag so that the waiting 'goal' message handler
    # can process the next goal.
    action_stopped_ev.set()

    # Release the waiting collector threads
    for _ in range(NUM_COLLECTORS):
        S.release()


def main():
    rospy.init_node('profiler', anonymous=True)

    # Publish DepthProfile messages
    pub = rospy.Publisher('~', DepthProfile, queue_size=1)

    # Accumulate depth messages while recording
    rospy.Subscriber('/ctd/depth', DepthPressure,
        functools.partial(record_data, depth_msgs))

    # Subscribe to the action start/stop messages
    rospy.Subscriber('/winch/move_to_depth/goal', MoveToDepthActionGoal,
        on_action_start)
    rospy.Subscriber('/winch/move_to_depth/result', MoveToDepthActionResult,
        functools.partial(on_action_stop, pub))

    rospy.spin()


if __name__ == '__main__':
    main()
