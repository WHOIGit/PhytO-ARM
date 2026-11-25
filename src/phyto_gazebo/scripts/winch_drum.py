#!/usr/bin/env python3
import rospy

from std_msgs.msg import Float64


# The MAC400 encoder has 8192 counts per motor turn
def line_length(position, initial_length, gear_ratio, spool_circumference):
    spool_turns = position / (8192 * gear_ratio)
    return max(0.0, initial_length + spool_turns * spool_circumference)


def main():
    rospy.init_node('winch_drum')
    gear_ratio = rospy.get_param('~gear_ratio', 60.0)
    initial_length = rospy.get_param('~initial_line_length', 1.32)
    spool = rospy.get_param('~spool_circumference', 0.6604)
    line_pub = rospy.Publisher('sim/line_length', Float64, queue_size=1,
                               latch=True)

    # The physical shaft position is continuous across encoder zero commands
    def on_shaft(msg):
        length = line_length(msg.data, initial_length, gear_ratio, spool)
        line_pub.publish(Float64(data=length))

    rospy.Subscriber('mac400_sim/shaft_position', Float64, on_shaft,
                     queue_size=1)
    rospy.spin()


if __name__ == '__main__':
    main()
