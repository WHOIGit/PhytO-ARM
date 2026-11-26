#!/usr/bin/env python3
"""Supply the CTD simulator with probe positions for casts without Gazebo.

The line hangs straight down from the surface, so its length gives the
probe depth. With the winch disabled, the probe stays at its initial depth.
"""
import rclpy

from geometry_msgs.msg import PointStamped
from phyto_sim_utils.parameters import declare_number
from rclpy.executors import ExternalShutdownException
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Float64


def main():
    rclpy.init()
    node = rclpy.create_node('vertical_line')

    # A taut line hangs straight down from the water surface
    def on_length(msg):
        nonlocal length
        length = msg.data

    # The CTD continues sampling when the drum is stationary or absent
    def publish_position():
        position = PointStamped()
        position.header.frame_id = 'world'
        position.header.stamp = node.get_clock().now().to_msg()
        position.point.z = -length
        pub.publish(position)

    try:
        length = float(declare_number(node, 'initial_depth'))
        pub = node.create_publisher(PointStamped, 'sim/ctd_position', 1)

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        node.create_subscription(Float64, 'sim/line_length', on_length, latched)

        publish_position()
        node.create_timer(0.1, publish_position)
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
