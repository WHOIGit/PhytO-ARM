#!/usr/bin/env python3
import math
import os
import sys
import tempfile
import unittest

sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from thermal import ThermalMonitor, lockout_active, read_lockout, write_lockout


def make_monitor(**kwargs):
    args = dict(max_temperature=40.0, trip_duration=300, min_readings=3,
                stale_timeout=900, now=0.0)
    args.update(kwargs)
    m = ThermalMonitor(**args)
    m.set_connected(True, args['now'])
    return m


class TestThermalMonitor(unittest.TestCase):
    def test_normal_readings_do_not_trip(self):
        m = make_monitor()
        for t in range(0, 1000, 60):
            m.reading(25.0, 50.0, t)
            self.assertIsNone(m.check(t))

    def test_sustained_over_temperature_trips(self):
        m = make_monitor()
        for t in range(0, 300, 60):
            m.reading(45.0, 50.0, t)
            self.assertIsNone(m.check(t))
        m.reading(45.0, 50.0, 300)
        self.assertIn('temperature', m.check(300))

    def test_blip_does_not_trip(self):
        m = make_monitor()
        m.reading(45.0, 50.0, 0)
        m.reading(25.0, 50.0, 60)
        for t in range(120, 400, 60):
            m.reading(45.0, 50.0, t)
        # Streak restarted at t=120, so 300 s have not elapsed by t=360
        self.assertIsNone(m.check(360))
        m.reading(45.0, 50.0, 420)
        self.assertIsNotNone(m.check(420))

    def test_min_readings_required(self):
        m = make_monitor(min_readings=3)
        m.reading(45.0, 50.0, 0)
        m.reading(45.0, 50.0, 400)
        self.assertIsNone(m.check(400))
        m.reading(45.0, 50.0, 460)
        self.assertIsNotNone(m.check(460))

    def test_invalid_readings_ignored(self):
        m = make_monitor(stale_timeout=0)
        for t in range(0, 1000, 60):
            m.reading(math.nan, 50.0, t)
            m.reading(500.0, 50.0, t)
            self.assertIsNone(m.check(t))

    def test_invalid_reading_does_not_reset_streak(self):
        m = make_monitor()
        m.reading(45.0, 50.0, 0)
        m.reading(math.nan, 50.0, 60)
        m.reading(45.0, 50.0, 120)
        m.reading(45.0, 50.0, 300)
        self.assertIsNotNone(m.check(300))

    def test_stale_trips_when_powered(self):
        m = make_monitor(stale_timeout=900)
        m.reading(25.0, 50.0, 100)
        self.assertIsNone(m.check(999))
        self.assertIn('no valid', m.check(1000))

    def test_stale_counts_invalid_readings_as_missing(self):
        m = make_monitor(stale_timeout=900)
        for t in range(0, 1000, 60):
            m.reading(math.nan, 50.0, t)
        self.assertIsNotNone(m.check(960))

    def test_stale_ignored_when_unpowered(self):
        m = make_monitor(stale_timeout=900)
        m.set_powered(False, 10)
        self.assertIsNone(m.check(5000))

    def test_stale_clock_restarts_at_power_on(self):
        m = make_monitor(stale_timeout=900)
        m.reading(25.0, 50.0, 0)
        m.set_powered(False, 10)
        m.set_powered(True, 5000)
        self.assertIsNone(m.check(5899))
        self.assertIsNotNone(m.check(5900))

    def test_stale_ignored_when_disconnected(self):
        m = make_monitor(stale_timeout=900)
        m.reading(25.0, 50.0, 0)
        m.set_connected(False, 10)
        self.assertIsNone(m.check(5000))

    def test_stale_clock_restarts_at_reconnect(self):
        m = make_monitor(stale_timeout=900)
        m.reading(25.0, 50.0, 0)
        m.set_connected(False, 10)
        m.set_connected(True, 5000)
        self.assertIsNone(m.check(5899))
        self.assertIsNotNone(m.check(5900))

    def test_stale_waits_for_first_connection(self):
        m = ThermalMonitor(40.0, 300, 3, 900, now=0.0)
        self.assertIsNone(m.check(5000))

    def test_stale_disabled(self):
        m = make_monitor(stale_timeout=0)
        self.assertIsNone(m.check(1e6))

    def test_power_off_resets_streak(self):
        m = make_monitor()
        m.reading(45.0, 50.0, 0)
        m.set_powered(False, 60)
        m.set_powered(True, 120)
        m.reading(45.0, 50.0, 180)
        m.reading(45.0, 50.0, 240)
        m.reading(45.0, 50.0, 300)
        self.assertIsNone(m.check(300))

    def test_humidity_ignored_by_default(self):
        m = make_monitor()
        for t in range(0, 1000, 60):
            m.reading(25.0, 99.0, t)
        self.assertIsNone(m.check(960))

    def test_humidity_trips_when_enabled(self):
        m = make_monitor(max_humidity=90.0)
        for t in range(0, 400, 60):
            m.reading(25.0, 95.0, t)
        self.assertIn('humidity', m.check(360))


class TestLockoutFile(unittest.TestCase):
    def test_roundtrip(self):
        with tempfile.TemporaryDirectory() as d:
            path = os.path.join(d, 'sub', 'thermal_lockout')
            self.assertFalse(lockout_active(path))
            write_lockout(path, 'too hot', temperature=45.0)
            self.assertTrue(lockout_active(path))
            record = read_lockout(path)
            self.assertEqual(record['reason'], 'too hot')
            self.assertFalse(record['completed'])
            self.assertEqual(record['temperature'], 45.0)

    def test_empty_path_is_inactive(self):
        self.assertFalse(lockout_active(''))
        self.assertFalse(lockout_active(None))

    def test_corrupt_file_still_locks_out(self):
        with tempfile.TemporaryDirectory() as d:
            path = os.path.join(d, 'thermal_lockout')
            with open(path, 'w') as f:
                f.write('not json')
            self.assertTrue(lockout_active(path))
            self.assertEqual(read_lockout(path), {})


if __name__ == '__main__':
    unittest.main()
