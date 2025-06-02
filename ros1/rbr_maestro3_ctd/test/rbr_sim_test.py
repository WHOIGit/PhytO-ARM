#!/usr/bin/env python3
import math
import time
import unittest

import rospy
import rostest
from ds_core_msgs.msg import RawData
from ds_sensor_msgs.msg import Ctd, DepthPressure
from rbr_maestro3_ctd.msg import RbrMeasurement


class RbrSimulator(unittest.TestCase):
    def round_trip(self, ctd, depth, stamp):
        readings = {}

        # The driver reconstructs a ROS stamp from the wire time in microseconds.
        def collect(msg, field):
            if abs((msg.header.stamp - stamp).to_nsec()) < 1000:
                readings[field] = msg

        for topic, msg_type, field in (
            ('sim/rbr', RawData, 'raw'),
            ('ctd/ctd', Ctd, 'ctd'),
            ('ctd/depth', DepthPressure, 'depth'),
            ('ctd/rbr/conductivity', RbrMeasurement, 'conductivity'),
        ):
            sub = rospy.Subscriber(topic, msg_type, collect, callback_args=field)
            self.addCleanup(sub.unregister)
        ctd_pub = rospy.Publisher('sim/ctd', Ctd, queue_size=5)
        depth_pub = rospy.Publisher('sim/depth', DepthPressure, queue_size=5)
        self.addCleanup(ctd_pub.unregister)
        self.addCleanup(depth_pub.unregister)
        ctd.header.stamp = depth.header.stamp = stamp
        ctd.ds_header.io_time = stamp + rospy.Duration.from_sec(0.25)
        deadline = time.monotonic() + 10
        while time.monotonic() < deadline and len(readings) < 4:
            ctd_pub.publish(ctd)
            depth_pub.publish(depth)
            rospy.sleep(0.05)
        self.assertEqual(set(readings), {'raw', 'ctd', 'depth', 'conductivity'})
        self.assertEqual(readings['ctd'].ds_header.io_time, ctd.ds_header.io_time)
        return readings

    def test_driver_decodes_simulated_water(self):
        readings = self.round_trip(Ctd(conductivity=3.125, temperature=13.75),
                                   DepthPressure(pressure=0.75, depth=0.72),
                                   rospy.Time(1700000000, 123456000))
        self.assertEqual(readings['raw'].data_direction, RawData.DATA_IN)
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
                                   DepthPressure(pressure=0, depth=0), rospy.Time(1700000001))
        self.assertTrue(math.isnan(readings['ctd'].temperature))
        self.assertTrue(math.isnan(readings['ctd'].conductivity))
        self.assertTrue(math.isnan(readings['conductivity'].value))
        self.assertEqual(readings['depth'].depth, 0)
        self.assertEqual(readings['depth'].pressure, 0)


if __name__ == '__main__':
    rospy.init_node('rbr_sim_test')
    rostest.rosrun('rbr_maestro3_ctd', 'rbr_sim', RbrSimulator)
