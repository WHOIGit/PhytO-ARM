"""Common ROS graph waits for the Gazebo integration tests."""
import time
import unittest

import rclpy

from rclpy.qos import qos_profile_sensor_data


class RosTest(unittest.TestCase):
    def setUp(self):
        rclpy.init()
        self.addCleanup(rclpy.shutdown)
        self.node = rclpy.create_node('integration_test')
        self.addCleanup(self.node.destroy_node)

    def wait_until(self, condition, timeout):
        deadline = time.monotonic() + timeout
        while not condition():
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return False
            rclpy.spin_once(self.node, timeout_sec=min(remaining, 0.1))
        return True

    def wait_for_message(self, topic, msg_type, timeout, qos=qos_profile_sensor_data):
        messages = []
        sub = self.node.create_subscription(msg_type, topic, messages.append, qos)
        try:
            self.assertTrue(self.wait_until(lambda: messages, timeout), topic)
            return messages[0]
        finally:
            self.node.destroy_subscription(sub)

    def result(self, future, timeout):
        self.assertTrue(self.wait_until(future.done, timeout), 'Timed out waiting for response')
        return future.result()

    def call(self, client, request, timeout=5):
        self.assertTrue(client.wait_for_service(timeout_sec=timeout), client.srv_name)
        return self.result(client.call_async(request), timeout)
