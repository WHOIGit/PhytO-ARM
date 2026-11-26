#!/usr/bin/env python3
import functools
import itertools
import os
import random
import threading

import numpy as np
import rclpy
import scipy.interpolate
import scipy.signal

from ds_sensor_msgs.msg import DepthPressure
from phyto_arm.sensor_subscription import subscribe_sensor
from phyto_arm_msgs.msg import DepthProfile, MoveToDepthFinished, MoveToDepthStarted
from phyto_sim_utils.parameters import declare_number
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter


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


def on_action_start(node, subscribe_data, action_msg):
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
    data_field = node.get_parameter('data_field').value
    data_topic = node.get_parameter('data_topic').value
    if data_sub is None or data_sub.topic_name != data_topic:
        if data_sub is not None:
            node.get_logger().info(f'Unsubscribing from {data_sub.topic_name}')
            node.destroy_subscription(data_sub)
            data_sub = None

        node.get_logger().info(f'Subscribing to {data_topic}')
        subscribe_data(data_topic)

    # Start recording
    is_recording = True

    # Release exclusive access
    for _ in range(NUM_COLLECTORS):
        S.release()


def on_action_stop(node, pub, action_msg):
    global is_recording

    # Acquire exclusive access to the buffers
    for _ in range(NUM_COLLECTORS):
        S.acquire()
    is_recording = False

    msg_class = type(data_msgs[0]) if data_msgs else None

    # Group messages based on header time to correlate them. Assumptions:
    #   1. The configured message topic has a std_msg/Header with timestamp.
    #   2. The timestamps match up exactly (true of AML and RBR messages).
    #   3. The data message is not a DepthPressure message.
    groups = {}
    for m in itertools.chain(depth_msgs, data_msgs):
        groups.setdefault((m.header.stamp.sec, m.header.stamp.nanosec), {})[type(m)] = m

    # Create an array of depth x data, discarding any mismatched pairs
    data = np.array([
        (v[DepthPressure].depth, getattr(v[msg_class], data_field))
        for v in groups.values()
        if len(v) == 2 and \
            v[DepthPressure].depth != DepthPressure.DEPTH_PRESSURE_NO_DATA
    ])

    if len(groups) != data.shape[0]:
        node.get_logger().warning(f'Discarded {len(groups) - data.shape[0]} data points')

    if data.shape[0] < 2:
        node.get_logger().error('Did not receive enough data')
        for _ in range(NUM_COLLECTORS):
            S.release()
        return

    # Get min and max depth params
    min_depth = node.get_parameter('peak_min_depth').value
    max_depth = node.get_parameter('peak_max_depth').value

    # Filter out data points outside the specified depth range
    if min_depth is not None and max_depth is not None:
        len_before = data.shape[0]
        data = data[(data[:,0] >= min_depth) & (data[:,0] <= max_depth)]
        node.get_logger().info(f'Profile depths filtered: {min_depth} to {max_depth} m. '
                      f'{data.shape[0]}/{len_before} points remain')

    # Create a function that linearly interpolates between points
    f = scipy.interpolate.interp1d(data[:,0], data[:,1])

    # Sample the interpolated function at a regular interval
    res = node.get_parameter('resolution').value
    x = np.arange(data[:,0].min(), data[:,0].max(), res)
    y = f(x)

    # TODO: Noise elimination, smoothing, etc.

    # Publish the resampled profile data
    profile = DepthProfile()
    profile.header.stamp = action_msg.stamp
    profile.goal_uuid = action_msg.uuid
    profile.data_topic = data_sub.topic_name
    profile.data_field = data_field
    profile.depths = x.tolist()
    profile.values = y.tolist()

    pub['all'].publish(profile)
    pub['down' if data[0,0] < data[-1,0] else 'up'].publish(profile)

    # Set the Event's internal flag so that the waiting 'goal' message handler
    # can process the next goal.
    action_stopped_ev.set()

    # Release the waiting collector threads
    for _ in range(NUM_COLLECTORS):
        S.release()


def configure_node(node: Node) -> None:
    node.declare_parameters('', [
        ('ctd_topic', Parameter.Type.STRING),
        ('data_field', Parameter.Type.STRING),
        ('data_topic', Parameter.Type.STRING),
    ])

    for name in ('peak_max_depth', 'peak_min_depth', 'resolution'):
        declare_number(node, name)

    # Publish DepthProfile messages
    pub = {
        'all':  node.create_publisher(DepthProfile, '~', 1),
        'down': node.create_publisher(DepthProfile, '~/downcast', 1),
        'up':   node.create_publisher(DepthProfile, '~/upcast', 1),
    }

    # Discover sensor publishers as they appear and match their offered QoS.
    def subscribe(topic, dest, msg_type=None):
        return subscribe_sensor(node, topic, functools.partial(record_data, dest), msg_type,
                                callback_group=MutuallyExclusiveCallbackGroup())

    depth_topic = node.get_parameter('ctd_topic').value
    depth_sub = data_topic = None

    def discover():
        nonlocal depth_sub
        global data_sub
        if depth_sub is None:
            depth_sub = subscribe(depth_topic, depth_msgs, DepthPressure)
        if data_topic is not None and data_sub is None:
            data_sub = subscribe(data_topic, data_msgs)
        if depth_sub is not None and (data_topic is None or data_sub is not None):
            timer.cancel()

    def subscribe_data(topic):
        nonlocal data_topic
        data_topic = topic
        timer.reset()
        discover()

    starts = MutuallyExclusiveCallbackGroup()
    timer = node.create_timer(0.1, discover, callback_group=starts)
    discover()

    # A result must be able to wake the next goal's waiting callback.
    node.create_subscription(MoveToDepthStarted, 'winch/move_to_depth/goal',
        functools.partial(on_action_start, node, subscribe_data), 10, callback_group=starts)
    node.create_subscription(MoveToDepthFinished, 'winch/move_to_depth/result',
        functools.partial(on_action_stop, node, pub), 10, callback_group=MutuallyExclusiveCallbackGroup())


def main():
    rclpy.init()
    node = rclpy.create_node(f'profiler_{os.getpid()}_{random.getrandbits(32):08x}')
    executor = MultiThreadedExecutor()
    try:
        configure_node(node)
        rclpy.spin(node, executor=executor)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
