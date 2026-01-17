#!/usr/bin/env python3
import functools

import rospy

from wr2_msgs.msg import RawData


# Republish incoming "raw" messages, dropping data blobs
def on_message(pub, msg):
    # Omit the binary blobs while keeping all the other fields
    if msg.data.startswith(b'triggerimage:'):
        msg.data = b'triggerimage:<omitted>'
    elif msg.data.startswith(b'triggerrois:'):
        parts = msg.data.split(b':')
        for i in range(4, len(parts), 3):
            parts[i] = b'<omitted>'
        msg.data = b':'.join(parts)
    elif msg.data.startswith(b'triggercontent:'):
        parts = msg.data.split(b':')
        try:
            rois_idx = parts.index(b'rois')
            count = int(parts[rois_idx + 1])
            for j in range(count):
                parts[rois_idx + 1 + 3 * (j + 1)] = b'<omitted>'
        except (ValueError, IndexError):
            pass
        msg.data = b':'.join(parts)
    elif msg.data.startswith(b'file:chunk:'):
        parts = msg.data.split(b':', maxsplit=4)
        parts[-1] = b'<omitted>'
        msg.data = b':'.join(parts)

    # Republish the message, which has potentially been mutated
    pub.publish(msg)


def main():
    rospy.init_node('ifcb_logfilter', anonymous=True)

    # Publishers for raw messages coming in and out of the IFCB
    pub = rospy.Publisher('/ifcb/in/filtered', RawData, queue_size=5)
    rospy.Subscriber('/ifcb/in', RawData, functools.partial(on_message, pub))

    # Keep the program alive while other threads work
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
