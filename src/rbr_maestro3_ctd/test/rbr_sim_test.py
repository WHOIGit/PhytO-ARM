#!/usr/bin/env python3
import math
import os
import time
import unittest

import rclpy

from functools import partial

from ament_index_python.packages import get_package_share_directory
from ds_sensor_msgs.msg import Ctd, DepthPressure
from launch import LaunchDescription
from launch.actions import GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import PushRosNamespace
from launch_testing.actions import ReadyToTest
from rbr_maestro3_ctd_msgs.msg import RbrMeasurement
from rclpy.duration import Duration
from rclpy.time import Time
from wr2_msgs.msg import RawData


def generate_test_description():
    launch_file = os.path.join(get_package_share_directory('rbr_maestro3_ctd'),
                               'launch', 'sim_ctd.launch.py')
    return LaunchDescription([
        GroupAction([
            PushRosNamespace('rbr_sim_test'),
            IncludeLaunchDescription(PythonLaunchDescriptionSource(launch_file)),
        ]),
        ReadyToTest(),
    ])


class RbrSimulator(unittest.TestCase):
    def setUp(self):
        rclpy.init()
        self.addCleanup(rclpy.try_shutdown)
        self.node = rclpy.create_node('rbr_sim_test', namespace='/rbr_sim_test')
        self.addCleanup(self.node.destroy_node)

    def round_trip(self, ctd, depth, stamp):
        readings = {}

        # The driver reconstructs a ROS stamp from the wire time in microseconds.
        def collect(msg, field):
            if field == 'raw':
                if msg.io_stamp == ctd.ds_header.io_time:
                    readings[field] = msg
            elif abs(Time.from_msg(msg.header.stamp).nanoseconds - stamp.nanoseconds) < 1000:
                readings[field] = msg

        for topic, msg_type, field in (
            ('sim/rbr', RawData, 'raw'),
            ('ctd/ctd', Ctd, 'ctd'),
            ('ctd/depth', DepthPressure, 'depth'),
            ('ctd/rbr/conductivity', RbrMeasurement, 'conductivity'),
        ):
            self.node.create_subscription(msg_type, topic, partial(collect, field=field), 5)
        ctd_pub = self.node.create_publisher(Ctd, 'sim/ctd', 5)
        depth_pub = self.node.create_publisher(DepthPressure, 'sim/depth', 5)
        ctd.header.stamp = depth.header.stamp = stamp.to_msg()
        ctd.ds_header.io_time = (stamp + Duration(seconds=0.25)).to_msg()

        def publish():
            ctd_pub.publish(ctd)
            depth_pub.publish(depth)

        self.node.create_timer(0.05, publish)
        deadline = time.monotonic() + 10
        while time.monotonic() < deadline and len(readings) < 4:
            rclpy.spin_once(self.node, timeout_sec=0.05)
        self.assertEqual(set(readings), {'raw', 'ctd', 'depth', 'conductivity'})
        self.assertEqual(readings['ctd'].ds_header.io_time, ctd.ds_header.io_time)
        return readings

    def test_driver_decodes_simulated_water(self):
        readings = self.round_trip(Ctd(conductivity=3.125, temperature=13.75),
                                   DepthPressure(pressure=0.75, depth=0.72),
                                   Time(seconds=1700000000, nanoseconds=123456000))
        self.assertEqual(readings['raw'].data_direction, RawData.DIRECTION_DATA_IN)
        self.assertEqual(bytes(readings['raw'].data),
                         b'2023-11-14 22:13:20.123456, 31.250000, 13.750000, 0.750000, 0.720000\r\n')
        self.assertEqual(readings['ctd'].temperature, 13.75)
        self.assertEqual(readings['ctd'].conductivity, 3.125)
        self.assertEqual(readings['depth'].depth, 0.72)
        self.assertEqual(readings['depth'].pressure, 0.75)
        self.assertEqual(readings['conductivity'].unit, 'mS/cm')
        self.assertEqual(readings['conductivity'].value, 31.25)

    def test_probe_above_water_reports_zero_depth_and_pressure(self):
        readings = self.round_trip(Ctd(conductivity=Ctd.CTD_NO_DATA, temperature=Ctd.CTD_NO_DATA),
                                   DepthPressure(pressure=0.0, depth=0.0), Time(seconds=1700000001))
        self.assertTrue(math.isnan(readings['ctd'].temperature))
        self.assertTrue(math.isnan(readings['ctd'].conductivity))
        self.assertTrue(math.isnan(readings['conductivity'].value))
        self.assertEqual(readings['depth'].depth, 0)
        self.assertEqual(readings['depth'].pressure, 0)
