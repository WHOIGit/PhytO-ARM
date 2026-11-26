#!/usr/bin/env python3
import rclpy

from phyto_sim_utils.parameters import declare_number
from rclpy.executors import ExternalShutdownException
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Float64


# The MAC400 encoder has 8192 counts per motor turn
def line_length(position, initial_length, gear_ratio, spool_circumference):
    spool_turns = position / (8192 * gear_ratio)
    return max(0.0, initial_length + spool_turns * spool_circumference)


def main():
    rclpy.init()
    node = rclpy.create_node('winch_drum')

    # The physical shaft position is continuous across encoder zero commands
    def on_shaft(msg):
        length = line_length(msg.data, initial_length, gear_ratio, spool)
        line_pub.publish(Float64(data=length))

    try:
        gear_ratio = declare_number(node, 'gear_ratio', 60.0)
        initial_length = declare_number(node, 'initial_line_length', 1.32)
        spool = declare_number(node, 'spool_circumference', 0.6604)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        line_pub = node.create_publisher(Float64, 'sim/line_length', latched)

        node.create_subscription(Float64, 'mac400_sim/shaft_position', on_shaft, 1)
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
