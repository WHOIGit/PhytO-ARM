#!/usr/bin/env python3
"""
Simple unit tests for the scheduling logic without any ROS dependencies.
"""
import unittest
import sys
import os
from datetime import time

# Add the src directory to the path so we can import the module
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))

from scheduling import parse_time_string, should_outlet_be_on


class TestSchedulingFunctions(unittest.TestCase):
    """Test cases for scheduled switching logic"""

    def test_parse_time_string_valid_formats(self):
        """Test parsing valid time strings"""
        self.assertEqual(parse_time_string("08:30"), time(8, 30))
        self.assertEqual(parse_time_string("00:00"), time(0, 0))
        self.assertEqual(parse_time_string("23:59"), time(23, 59))
        self.assertEqual(parse_time_string("09:05"), time(9, 5))

    def test_parse_time_string_invalid_formats(self):
        """Test parsing invalid time strings"""
        with self.assertRaises(ValueError):
            parse_time_string("25:00")  # Invalid hour

        with self.assertRaises(ValueError):
            parse_time_string("12:60")  # Invalid minute

        with self.assertRaises(ValueError):
            parse_time_string("12-30")  # Wrong separator

        with self.assertRaises(ValueError):
            parse_time_string("noon")   # Text format

    def test_should_outlet_be_on_no_schedule(self):
        """Test behavior when no schedule is defined"""
        # No schedule defined
        result = should_outlet_be_on([], [], time(10, 0))
        self.assertIsNone(result)

    def test_should_outlet_be_on_simple_schedule(self):
        """Test simple on/off schedule within same day"""
        on_times = ["08:00"]
        off_times = ["12:00"]

        # At 10:00, should be ON (most recent action was ON at 08:00)
        result = should_outlet_be_on(on_times, off_times, time(10, 0))
        self.assertTrue(result)

        # At 14:00, should be OFF (most recent action was OFF at 12:00)
        result = should_outlet_be_on(on_times, off_times, time(14, 0))
        self.assertFalse(result)

        # At 06:00 (before any schedule), should use yesterday's last action (OFF at 12:00)
        result = should_outlet_be_on(on_times, off_times, time(6, 0))
        self.assertFalse(result)

    def test_should_outlet_be_on_multiple_switches(self):
        """Test schedule with multiple on/off times per day"""
        on_times = ["08:00", "14:00"]
        off_times = ["12:00", "18:00"]

        # At 15:00, most recent was ON at 14:00
        result = should_outlet_be_on(on_times, off_times, time(15, 0))
        self.assertTrue(result)

        # At 07:00, most recent would be from previous day (OFF at 18:00)
        result = should_outlet_be_on(on_times, off_times, time(7, 0))
        self.assertFalse(result)

        # At 10:00, most recent was ON at 08:00
        result = should_outlet_be_on(on_times, off_times, time(10, 0))
        self.assertTrue(result)

        # At 13:00, most recent was OFF at 12:00
        result = should_outlet_be_on(on_times, off_times, time(13, 0))
        self.assertFalse(result)

    def test_should_outlet_be_on_only_on_times(self):
        """Test schedule with only ON times (no OFF times)"""
        on_times = ["08:00"]
        off_times = []

        # Should be ON after the on time
        result = should_outlet_be_on(on_times, off_times, time(10, 0))
        self.assertTrue(result)

        # Test before the on time - should use yesterday's ON
        result = should_outlet_be_on(on_times, off_times, time(6, 0))
        self.assertTrue(result)

    def test_should_outlet_be_on_only_off_times(self):
        """Test schedule with only OFF times (no ON times)"""
        on_times = []
        off_times = ["12:00"]

        # Before off time - should use yesterday's OFF
        result = should_outlet_be_on(on_times, off_times, time(10, 0))
        self.assertFalse(result)

        # After off time
        result = should_outlet_be_on(on_times, off_times, time(14, 0))
        self.assertFalse(result)

    def test_should_outlet_be_on_exact_schedule_times(self):
        """Test behavior exactly at scheduled times"""
        on_times = ["08:00"]
        off_times = ["12:00"]

        # Exactly at ON time
        result = should_outlet_be_on(on_times, off_times, time(8, 0))
        self.assertTrue(result)

        # Exactly at OFF time
        result = should_outlet_be_on(on_times, off_times, time(12, 0))
        self.assertFalse(result)

    def test_should_outlet_be_on_complex_schedule(self):
        """Test complex schedule with multiple overlapping times"""
        on_times = ["06:00", "12:00", "18:00"]
        off_times = ["09:00", "15:00", "21:00"]

        test_cases = [
            (time(7, 0), True),   # After 06:00 ON
            (time(10, 0), False), # After 09:00 OFF
            (time(13, 0), True),  # After 12:00 ON
            (time(16, 0), False), # After 15:00 OFF
            (time(19, 0), True),  # After 18:00 ON
            (time(22, 0), False), # After 21:00 OFF
            (time(5, 0), False),  # Before any schedule (yesterday's 21:00 OFF)
        ]

        for test_time, expected_state in test_cases:
            with self.subTest(time=test_time):
                result = should_outlet_be_on(on_times, off_times, test_time)
                self.assertEqual(result, expected_state,
                               f"At {test_time}, expected {expected_state}, got {result}")

    def test_edge_case_midnight_schedule(self):
        """Test schedules that span midnight"""
        # Night shift: ON at 22:00, OFF at 06:00
        on_times = ["22:00"]
        off_times = ["06:00"]

        # Test times during the night (should be ON)
        result = should_outlet_be_on(on_times, off_times, time(23, 0))
        self.assertTrue(result)  # After 22:00 ON

        result = should_outlet_be_on(on_times, off_times, time(1, 0))
        self.assertTrue(result)  # Before 06:00 OFF, after yesterday's 22:00 ON

        # Test times during the day (should be OFF)
        result = should_outlet_be_on(on_times, off_times, time(8, 0))
        self.assertFalse(result)  # After 06:00 OFF

        result = should_outlet_be_on(on_times, off_times, time(20, 0))
        self.assertFalse(result)  # Before 22:00 ON, after 06:00 OFF


if __name__ == '__main__':
    unittest.main()
