#!/usr/bin/env python3
import functools
import os
import random

import rclpy

from rclpy.executors import ExternalShutdownException
from wr2_msgs.msg import RawData


# Republish incoming "raw" messages, dropping data blobs
def on_message(pub, msg):
    # Omit the binary blobs while keeping all the other fields
    data = bytes(msg.data)
    if data.startswith(b'triggerimage:'):
        msg.data = b'triggerimage:<omitted>'
    elif data.startswith(b'triggerrois:'):
        parts = data.split(b':')
        for i in range(4, len(parts), 3):
            parts[i] = b'<omitted>'
        msg.data = b':'.join(parts)
    elif data.startswith(b'triggercontent:'):
        parts = data.split(b':')
        try:
            rois_idx = parts.index(b'rois')
            count = int(parts[rois_idx + 1])
            for j in range(count):
                parts[rois_idx + 1 + 3 * (j + 1)] = b'<omitted>'
        except (ValueError, IndexError):
            pass
        msg.data = b':'.join(parts)
    elif data.startswith(b'file:chunk:'):
        parts = data.split(b':', maxsplit=4)
        parts[-1] = b'<omitted>'
        msg.data = b':'.join(parts)

    # Republish the message, which has potentially been mutated
    pub.publish(msg)


def main():
    rclpy.init()
    node = rclpy.create_node(f'ifcb_logfilter_{os.getpid()}_{random.getrandbits(32):08x}')
    try:
        # Publishers for raw messages coming in and out of the IFCB
        pub = node.create_publisher(RawData, '/ifcb/in/filtered', 5)
        node.create_subscription(RawData, '/ifcb/in', functools.partial(on_message, pub), 10)
        # Keep the program alive while other threads work
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
