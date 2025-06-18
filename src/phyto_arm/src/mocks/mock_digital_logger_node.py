#!/usr/bin/env python3
"""
Mock functionality for Digital Loggers Web Power Switch Pro model.
Simulates the same ROS interface without requiring actual hardware.
"""
import rospy

from phyto_arm.msg import OutletStatus


class MockDigitalLogger:
    def __init__(self):
        """Initialize the mock digital logger with simulated outlets."""
        rospy.init_node('mock_digital_logger')
        rospy.logwarn(f'Starting mock digital logger node {rospy.get_name()}')
        
        # Get configuration parameters (same as real node)
        self.outlets = rospy.get_param('~outlets', [])
        self.outlet_names = {outlet['name']: int(outlet['outlet']) for outlet in self.outlets}
        
        # Initialize outlet states (all start as inactive)
        self.outlet_states = {}
        for outlet in self.outlets:
            self.outlet_states[outlet['name']] = False
        
        # Subscribe to control topic
        rospy.Subscriber('/digital_logger/control', OutletStatus, self.control_outlet)
        
        # Create publishers for each outlet status
        self.outlet_publishers = []
        for outlet_num, _ in enumerate(self.outlets):
            publisher = rospy.Publisher(f'/digital_logger/outlet/{outlet_num}/status/', OutletStatus, queue_size=10)
            self.outlet_publishers.append(publisher)
        
        rospy.loginfo(f"Mock digital logger initialized with {len(self.outlets)} outlets")

    def control_outlet(self, msg):
        """Handle outlet control messages by updating internal state."""
        if msg.name in self.outlet_states:
            self.outlet_states[msg.name] = msg.is_active
            rospy.loginfo(f'Mock digital logger: Set outlet "{msg.name}" to {"ON" if msg.is_active else "OFF"}')
        else:
            rospy.logwarn(f'Mock digital logger: Unknown outlet name "{msg.name}"')

    def publish_outlet_statuses(self):
        """Publish current status of all outlets."""
        for outlet_index, outlet in enumerate(self.outlets):
            outlet_status = OutletStatus()
            outlet_status.name = outlet['name']
            outlet_status.is_active = self.outlet_states[outlet['name']]
            
            self.outlet_publishers[outlet_index].publish(outlet_status)

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