#!/usr/bin/env python3
import functools
import importlib
import io
import itertools
import math

import rospy
import numpy as np

from ds_sensor_msgs.msg import DepthPressure

from phyto_arm.msg import MoveToDepthActionGoal, MoveToDepthActionResult


# Flag indicating whether a move_to_depth action is being performed and hence
# depth messages should be recorded.
is_recording = False

# Lists of accumulated depth and data messages while recording
data_msgs, depth_msgs = [], [] 


def on_action_start(action_msg):
    global is_recording, data_msgs, depth_msgs

    # Clear buffers and start recording
    data_msgs, depth_msgs = [], []
    is_recording = True


def on_action_stop(data_field, action_msg):
    global is_recording, data_msgs, depth_msgs

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
    
    # Create regularly-spaced bins for the values
    resolution = rospy.get_param('~bin_resolution')
    range_min = resolution * math.floor(data[:,0].min() / resolution)
    range_max = resolution * math.ceil(data[:,0].max() / resolution)
    bins = np.linspace(  # numerically stable, whereas np.arange() is not
        range_min, range_max,
        int(round((range_max - range_min) / resolution)) + 1
    )

    # Compute the average measurement in each bin.
    # See https://stackoverflow.com/a/6163403. Note that in our case the binning
    # is done by depth but the weight is the data value.
    bin_avgs = (np.histogram(data[:,0], bins, weights=data[:,1])[0] /
        np.histogram(data[:,0], bins)[0])
    
    print(bins)
    print(bin_avgs)
    print()


def main():
    rospy.init_node('observer', anonymous=True)

    # Accumulate depth and data messages while recording
    rospy.Subscriber('/ctd/depth', DepthPressure,
        lambda m: is_recording and depth_msgs.append(m))
    rospy.Subscriber(rospy.get_param('~data_topic'), rospy.AnyMsg,
        lambda m: is_recording and data_msgs.append(m))

    # Subscribe to the action start/stop messages
    rospy.Subscriber('/winch/move_to_depth/goal', MoveToDepthActionGoal,
        on_action_start)
    rospy.Subscriber('/winch/move_to_depth/result', MoveToDepthActionResult,
        functools.partial(on_action_stop,
            rospy.get_param('~data_field')))

    rospy.spin()


if __name__ == '__main__':
    main()
