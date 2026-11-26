#!/usr/bin/env python3
import unittest

import rclpy

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from gps_msgs.msg import GPSFix
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import SetParameter
from launch_testing.actions import ReadyToTest
from rclpy.task import Future
from rclpy.time import Time
from sensor_msgs.msg import NavSatFix


def generate_test_description():
    share = Path(get_package_share_directory('phyto_sim_utils'))
    return LaunchDescription([
        SetParameter(name='latitude', value=42.1),
        SetParameter(name='longitude', value=-69.9),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(share / 'launch/gps.launch.py'))),
        ReadyToTest(),
    ])


class TestGPSMock(unittest.TestCase):
    def test_stationary_fix(self):
        rclpy.init()
        self.addCleanup(rclpy.shutdown)
        node = rclpy.create_node('gpsd_client_mock_test')
        self.addCleanup(node.destroy_node)
        for topic, cls in [('fix', NavSatFix), ('extended_fix', GPSFix)]:
            future = Future()
            sub = node.create_subscription(cls, '/gps/' + topic, future.set_result, 1)
            rclpy.spin_until_future_complete(node, future, timeout_sec=5)
            node.destroy_subscription(sub)
            self.assertTrue(future.done(), topic)
            msg = future.result()
            self.assertEqual((msg.latitude, msg.longitude), (42.1, -69.9))
            self.assertGreater(Time.from_msg(msg.header.stamp).nanoseconds, 0)
