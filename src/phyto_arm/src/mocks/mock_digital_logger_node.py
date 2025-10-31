#!/usr/bin/env python3
"""
Mock functionality for Digital Loggers Web Power Switch Pro model.
Simulates the same ROS interface without requiring actual hardware.
"""
import functools
import rospy
from std_msgs.msg import Bool
from dli_power_switch.msg import OutletStatus


class MockDigitalLogger:
    def __init__(self):
        """Initialize the mock digital logger with simulated outlets."""
        rospy.init_node('mock_digital_logger')
        rospy.logwarn(f'Starting mock digital logger node {rospy.get_name()}')

        # Get configuration parameters (same as real node)
        self.outlets = rospy.get_param('~outlets', [])

        # Initialize outlet states (all start as inactive)
        self.outlet_states = {}
        self.outlet_publishers = {}

        # Set up publisher and subscriber for each outlet (matching real node)
        for outlet in self.outlets:
            name = outlet['name']
            self.outlet_states[name] = False

            # Status publisher
            self.outlet_publishers[name] = rospy.Publisher(
                f'~outlet/{name}/status',
                OutletStatus,
                queue_size=10
            )

            # Control subscriber
            rospy.Subscriber(
                f'~outlet/{name}/control',
                Bool,
                functools.partial(self.control_outlet, name)
            )
            rospy.loginfo(f"Mock digital logger configured outlet '{name}'")

        rospy.loginfo(f"Mock digital logger initialized with {len(self.outlets)} outlets")

    def control_outlet(self, outlet_name, msg):
        """Handle outlet control messages by updating internal state."""
        if outlet_name in self.outlet_states:
            self.outlet_states[outlet_name] = msg.data
            rospy.loginfo(
                f'Mock digital logger: Switched {"ON" if msg.data else "OFF"} '
                f'outlet "{outlet_name}"'
            )
        else:
            rospy.logwarn(f'Mock digital logger: Unknown outlet name "{outlet_name}"')

    def publish_outlet_statuses(self):
        """Publish current status of all outlets."""
        for outlet in self.outlets:
            name = outlet['name']
            outlet_status = OutletStatus()
            outlet_status.is_active = self.outlet_states[name]
            outlet_status.header.stamp = outlet_status.ds_header.io_time = \
                rospy.Time.now()
            self.outlet_publishers[name].publish(outlet_status)

    def run(self):
        """Run the mock digital logger node."""
        # Monitor outlets at 1Hz (same as real node)
        rate = rospy.Rate(1)

        while not rospy.is_shutdown():
            self.publish_outlet_statuses()
            rate.sleep()


def main():
    """Main entry point for the mock digital logger node."""
    try:
        mock_logger = MockDigitalLogger()
        mock_logger.run()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
