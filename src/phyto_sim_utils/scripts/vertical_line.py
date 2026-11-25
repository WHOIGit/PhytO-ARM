#!/usr/bin/env python3
"""Supply the CTD simulator with probe positions for casts without Gazebo.

The line hangs straight down from the surface, so its length gives the
probe depth. With the winch disabled, the probe stays at its initial depth.
"""
import rospy

from geometry_msgs.msg import PointStamped
from std_msgs.msg import Float64


def main():
    rospy.init_node('vertical_line')
    length = rospy.get_param('~initial_depth')
    pub = rospy.Publisher('sim/ctd_position', PointStamped, queue_size=1)

    # A taut line hangs straight down from the water surface
    def on_length(msg):
        nonlocal length
        length = msg.data

    rospy.Subscriber('sim/line_length', Float64, on_length, queue_size=1)

    # The CTD continues sampling when the drum is stationary or absent
    rate = rospy.Rate(10, reset=True)
    while not rospy.is_shutdown():
        position = PointStamped()
        position.header.frame_id = 'world'
        position.header.stamp = rospy.Time.now()
        position.point.z = -length
        pub.publish(position)
        rate.sleep()


if __name__ == '__main__':
    main()
