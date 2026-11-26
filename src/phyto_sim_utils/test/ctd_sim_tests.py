import os
import sys
import unittest

from builtin_interfaces.msg import Time
from ds_sensor_msgs.msg import Ctd, DepthPressure
from geometry_msgs.msg import PointStamped

sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'scripts'))

from ctd_sim import sample_ctd


class TestCTDSim(unittest.TestCase):
    def sample(self, z, surface_z=0.0, warm_layer_depth=None):
        position = PointStamped()
        position.header.frame_id = 'world'
        position.header.stamp = Time(sec=12, nanosec=345)
        position.point.z = float(z)
        return sample_ctd(position, surface_z, warm_layer_depth)

    def test_synchronized_pressure_and_depth(self):
        ctd, depth = self.sample(-1.75, surface_z=0.25)
        self.assertEqual(depth.depth, 2.0)
        self.assertAlmostEqual(ctd.pressure, 2.01036325)
        self.assertEqual(ctd.pressure, depth.pressure)
        self.assertEqual(ctd.pressure, depth.pressure_raw)
        self.assertEqual(depth.pressure_raw_unit,
                         DepthPressure.UNIT_PRESSURE_DBAR)
        for msg in (ctd, depth):
            self.assertEqual(msg.header.stamp, Time(sec=12, nanosec=345))
            self.assertEqual(msg.ds_header.io_time, msg.header.stamp)
            self.assertEqual(msg.header.frame_id, 'world')

    def test_water_column(self):
        surface, _ = self.sample(0)
        deeper, _ = self.sample(-2)
        self.assertEqual(surface.temperature, 15)
        self.assertAlmostEqual(surface.conductivity, 4.2914, places=3)
        self.assertAlmostEqual(surface.sound_speed, 1506.7, delta=0.2)
        self.assertEqual(deeper.salinity, 35)
        self.assertLess(deeper.temperature, surface.temperature)
        self.assertLess(deeper.conductivity, surface.conductivity)

    def test_warm_layer_at_configured_depth(self):
        peak, _ = self.sample(-0.7, warm_layer_depth=0.7)
        shallow, _ = self.sample(-0.5, warm_layer_depth=0.7)
        deep, _ = self.sample(-0.9, warm_layer_depth=0.7)
        self.assertAlmostEqual(peak.temperature, 16.0)
        self.assertGreater(peak.temperature, shallow.temperature)
        self.assertGreater(peak.temperature, deep.temperature)

    def test_dry_sample(self):
        ctd, depth = self.sample(1)
        self.assertEqual(depth.depth, 0)
        self.assertEqual(ctd.pressure, 0)
        self.assertEqual(ctd.conductivity, Ctd.CTD_NO_DATA)
        self.assertEqual(ctd.salinity, Ctd.CTD_NO_DATA)
