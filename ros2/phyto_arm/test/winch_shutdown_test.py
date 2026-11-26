import time
import unittest

import rclpy

from ament_index_python.packages import get_package_share_path
from jvl_motor_msgs.msg import Motion
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_testing import post_shutdown_test
from launch_testing.actions import ReadyToTest
from launch_testing.asserts import assertExitCodes


def generate_test_description():
    launch = get_package_share_path('phyto_arm') / 'launch' / 'simulator.launch.py'
    return LaunchDescription([
        IncludeLaunchDescription(
            str(launch), launch_arguments={'motor_modbus_port': '16543'}.items()),
        ReadyToTest(),
    ])


class ActiveCastTest(unittest.TestCase):
    def test_interrupt_during_motion(self):
        rclpy.init()
        self.addCleanup(rclpy.shutdown)
        node = rclpy.create_node('winch_shutdown_test')
        self.addCleanup(node.destroy_node)

        # Returning from the active test shuts down the entire launch
        moving = False

        def motion(msg):
            nonlocal moving
            moving = msg.mode == Motion.MODE_VELOCITY

        node.create_subscription(Motion, '/arm_ifcb/motor/motion', motion, 1)
        deadline = time.monotonic() + 20
        while not moving and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        self.assertTrue(moving, 'The simulated winch never entered velocity mode')


@post_shutdown_test()
class WinchExitTest(unittest.TestCase):
    def test_winch_exits_cleanly(self, proc_info):
        assertExitCodes(proc_info, process='winch_node.py')
