#!/usr/bin/env python3
PKG="phyto_arm"
from datetime import datetime
import os
import sys
import unittest
from unittest.mock import patch, Mock, MagicMock

import numpy as np
import rospy


# Hack for getting this to recognize local modules for import
# Have not found a ROS-recommended way to import modules being tested
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

import arm_ifcb
from lock_manager import NamedLockManager


class TestNamedLockManager(unittest.TestCase):
    def setUp(self):
        # Initialize NamedLockManager with a limit of 2 concurrent locks for testing
        self.lock_manager = NamedLockManager(max_concurrent_locks=2)

    def test_acquire_lock_success(self):
        # Test that a lock can be successfully acquired
        self.assertTrue(self.lock_manager.acquire_lock("arm1"))

    def test_acquire_lock_fail_due_to_limit(self):
        # Acquire locks up to the limit
        self.lock_manager.acquire_lock("arm1")
        self.lock_manager.acquire_lock("arm2")
        # Test that acquiring another lock fails due to the limit
        self.assertFalse(self.lock_manager.acquire_lock("arm3"))

    def test_release_lock_success(self):
        # Acquire a lock and then release it
        self.lock_manager.acquire_lock("arm1")
        self.assertTrue(self.lock_manager.release_lock("arm1"))

    def test_release_lock_fail_not_owned(self):
        # Test releasing a lock that was not acquired
        self.assertFalse(self.lock_manager.release_lock("arm1"))

    def test_check_lock_owned(self):
        # Acquire a lock and check it
        self.lock_manager.acquire_lock("arm1")
        self.assertTrue(self.lock_manager.check_lock("arm1"))

    def test_check_lock_not_owned(self):
        # Check a lock without acquiring it
        self.assertFalse(self.lock_manager.check_lock("arm1"))

    def test_reentrant_acquire_lock(self):
        # Test acquiring a lock twice by the same arm
        self.assertTrue(self.lock_manager.acquire_lock("arm1"))
        self.assertFalse(self.lock_manager.acquire_lock("arm1"))  # Should fail due to re-entrance

    def test_release_lock_not_affecting_others(self):
        # Test that releasing one lock does not affect others
        self.lock_manager.acquire_lock("arm1")
        self.lock_manager.acquire_lock("arm2")
        self.lock_manager.release_lock("arm1")
        self.assertTrue(self.lock_manager.check_lock("arm2"))

class TestWizToRosTime(unittest.TestCase):
    @patch('arm_ifcb.datetime', wraps=datetime)
    def test_wiz_to_rostime_next_day(self, mock_datetime):
        # Mock datetime.now() to return a specific point in time in the evening
        mock_now = datetime(2024, 2, 16, 23, 0)  # Feb 16, 2024, 11:00 PM
        mock_datetime.now.return_value = mock_now
        
        # Calculate the expected ROS time for the next day
        target_time = datetime(2024, 2, 17, 9, 0)  # Target time: Feb 17, 2024, 9:00 AM
        expected_secs_since_epoch = target_time.timestamp()
        expected_ros_time = rospy.Time.from_sec(expected_secs_since_epoch)
        
        # Call the function under test and get the return value
        result_ros_time = arm_ifcb.wiz_to_rostime("09:00")
        
        # Verify that the result matches the expected ROS time for the next day
        self.assertEqual(result_ros_time.secs, expected_ros_time.secs, "The returned ROS time does not match the expected time for the next day.")


    @patch('arm_ifcb.datetime', wraps=datetime)
    def test_wiz_to_rostime_before_current_time(self, mock_datetime):
        # Mock datetime.now() to return a specific point in time
        mock_now = datetime(2024, 2, 16, 10, 30)  # Feb 16, 2024, 10:30 AM
        mock_datetime.now.return_value = mock_now
        
        # Calculate the expected ROS time for the same day but later
        target_time = datetime(2024, 2, 16, 10, 0)  # Target time: 10:00 AM same day
        expected_secs_since_epoch = target_time.timestamp() + 86400  # Adding one day since 10:00 AM has already passed
        expected_ros_time = rospy.Time.from_sec(expected_secs_since_epoch)
        
        # Call the function under test with a time before the current mock time to ensure it rolls over to the next day
        result_ros_time = arm_ifcb.wiz_to_rostime("10:00")
        
        # Verify that the result matches the expected ROS time for the next day due to time rollover
        self.assertEqual(result_ros_time.secs, expected_ros_time.secs, "The returned ROS time should roll over to the next day when the target time is before the current time.")

# TODO: Finish this test case. Mocking datetime.now() causes this particular
# function to break as it tries to compare instances of MagicMock.
# class TestFindNextWizTime(unittest.TestCase):
#     @patch('arm_ifcb.rospy')
#     def test_find_next_wiz_time(self, mock_rospy):
#         mock_rospy.get_param.return_value = ["12:00", "15:00", "18:00"]
#         # Now is 14:00
#         mock_now = datetime(2024, 2, 16, 14, 0)
#         # Next wiz time should be 15:00
#         expected = datetime(2024, 2, 16, 15, 0).timestamp()
#         with patch('arm_ifcb.datetime', wraps=datetime) as mock_datetime:
#             mock_datetime.now.return_value = mock_now
#             next_time = arm_ifcb.find_next_wiz_time()
#             self.assertEqual(next_time.secs, expected, "The next wiz time is not correct.")


class TestItsWizTime(unittest.TestCase):
    @patch('arm_ifcb.rospy')
    def test_its_wiz_time_true(self, mock_rospy):
        mock_rospy.get_param.return_value = 10  # Preparation window in minutes
        mock_rospy.Duration.return_value = rospy.Duration(600)  # 10 minutes in seconds
        with patch('arm_ifcb.find_next_wiz_time') as mock_find_next_wiz_time, \
             patch('arm_ifcb.rospy.Time.now') as mock_rospy_time_now:
            mock_find_next_wiz_time.return_value = rospy.Time.from_sec(datetime(2024, 2, 16, 12, 10).timestamp())
            mock_rospy_time_now.return_value = rospy.Time.from_sec(datetime(2024, 2, 16, 12, 0).timestamp())
            self.assertTrue(arm_ifcb.its_wiz_time(), "It should be wiz time.")


    @patch('arm_ifcb.rospy')
    def test_its_wiz_time_false(self, mock_rospy):
        mock_rospy.get_param.return_value = 5  # Preparation window in minutes
        mock_rospy.Duration.return_value = rospy.Duration(300)  # 5 minutes in seconds
        with patch('arm_ifcb.find_next_wiz_time') as mock_find_next_wiz_time, \
             patch('arm_ifcb.rospy.Time.now') as mock_rospy_time_now:
            mock_find_next_wiz_time.return_value = rospy.Time.from_sec(datetime(2024, 2, 16, 12, 10).timestamp())
            mock_rospy_time_now.return_value = rospy.Time.from_sec(datetime(2024, 2, 16, 12, 0).timestamp())
            self.assertFalse(arm_ifcb.its_wiz_time(), "It should not be wiz time yet.")

class TestComputeWizDepth(unittest.TestCase):
    @patch('arm_ifcb.rospy')
    def test_use_phy_peak_disabled(self, mock_rospy):
        mock_rospy.get_param.return_value = False  # Disable use_phy_peak
        default_depth = 100  # Set a default depth for when phy_peak usage is disabled
        mock_rospy.get_param.return_value = default_depth
        result = arm_ifcb.compute_wiz_depth(50)  # The input value shouldn't matter in this case
        self.assertEqual(result, default_depth, "Should return default depth when use_phy_peak is disabled.")

    @patch('arm_ifcb.rospy')
    def test_no_phy_peak_depth_available(self, mock_rospy):
        mock_rospy.get_param.side_effect = [True, 200, None]  # Enable use_phy_peak, set default depth to 200
        result = arm_ifcb.compute_wiz_depth(None)  # No peak depth available
        self.assertEqual(result, 200, "Should return default depth when no phy peak depth is available.")

    @patch('arm_ifcb.rospy')
    def test_preferred_depth_within_winch_range(self, mock_rospy):
        mock_rospy.get_param.side_effect = lambda param: {
            'tasks/wiz_probe/use_phy_peak': True,
            'tasks/wiz_probe/offset': 10,
            'winch/range/max': 300,
            'winch/range/min': 100,
            'tasks/wiz_probe/default_depth': 200
        }[param]
        peak_depth = 150
        expected_depth = 160  # peak_depth + offset
        result = arm_ifcb.compute_wiz_depth(peak_depth)
        self.assertEqual(result, expected_depth, "Should return preferred depth within winch range.")

    @patch('arm_ifcb.rospy')
    def test_preferred_depth_exceeds_max_depth(self, mock_rospy):
        mock_rospy.get_param.side_effect = lambda param: {
            'tasks/wiz_probe/use_phy_peak': True,
            'tasks/wiz_probe/offset': 50,
            'winch/range/max': 200,
            'winch/range/min': 50,
            'tasks/wiz_probe/default_depth': 150
        }[param]
        peak_depth = 160
        expected_depth = 200  # Max depth
        result = arm_ifcb.compute_wiz_depth(peak_depth)
        self.assertEqual(result, expected_depth, "Should return max depth if preferred depth exceeds max depth.")

    @patch('arm_ifcb.rospy')
    def test_preferred_depth_below_min_depth(self, mock_rospy):
        mock_rospy.get_param.side_effect = lambda param: {
            'tasks/wiz_probe/use_phy_peak': True,
            'tasks/wiz_probe/offset': -20,
            'winch/range/max': 250,
            'winch/range/min': 100,
            'tasks/wiz_probe/default_depth': 150
        }[param]
        peak_depth = 110
        expected_depth = 100  # Min depth
        result = arm_ifcb.compute_wiz_depth(peak_depth)
        self.assertEqual(result, expected_depth, "Should return min depth if preferred depth is below min depth.")


class TestAwaitWizProbe(unittest.TestCase):
    @patch('arm_ifcb.rospy')
    def test_await_wiz_probe(self, mock_rospy):
        mock_rospy.get_param.side_effect = [10, 20]  # Preparation window in minutes, duration in minutes
        mock_rospy.Duration.return_value = rospy.Duration(600)  # 10 minutes in seconds
        with patch('arm_ifcb.find_next_wiz_time') as mock_find_next_wiz_time, \
             patch('arm_ifcb.rospy.Time.now') as mock_rospy_time_now, \
             patch('arm_ifcb.rospy.sleep') as mock_sleep:

            mock_callback = Mock()
            # Next wiz time is 12:10
            mock_find_next_wiz_time.return_value = rospy.Time.from_sec(datetime(2024, 2, 16, 12, 10).timestamp())
            # Now is 12:05
            mock_rospy_time_now.return_value = rospy.Time.from_sec(datetime(2024, 2, 16, 12, 5).timestamp())
            arm_ifcb.await_wiz_probe(mock_callback)
            # Sleep for 5 minute remaining window + 20 minute duration
            mock_sleep.assert_called_once_with(5 * 60 + 20 * 60)  # 2 minutes for wiz duration
            mock_callback.assert_called_once()


class TestItsScheduledDepthTime(unittest.TestCase):
    @patch('arm_ifcb.rospy')
    @patch('arm_ifcb.arm')
    def test_its_scheduled_depth_time_disabled(self, mock_arm, mock_rospy):
        mock_rospy.get_param.return_value = 0  # Scheduled interval set to 0 to disable
        mock_rospy.Duration.return_value = rospy.Duration(0)
        self.assertFalse(arm_ifcb.its_scheduled_depth_time(None), "Scheduled depth should not run when disabled.")


    @patch('arm_ifcb.rospy')
    @patch('arm_ifcb.arm')
    def test_its_scheduled_depth_time_first_run(self, mock_arm, mock_rospy):
        mock_rospy.get_param.return_value = 10  # Interval in minutes
        mock_rospy.Duration.return_value = rospy.Duration(600)
        mock_arm.last_scheduled_time = None
        self.assertTrue(arm_ifcb.its_scheduled_depth_time(None), "Scheduled depth should run on the first attempt.")


    @patch('arm_ifcb.rospy')
    @patch('arm_ifcb.arm')
    def test_its_scheduled_depth_time_interval_passed(self, mock_arm, mock_rospy):
        mock_rospy.get_param.return_value = 10  # Interval in minutes
        mock_rospy.Duration.return_value = rospy.Duration(600)
        mock_arm.last_scheduled_time = rospy.Time.from_sec(0)  # Simulate an old timestamp
        with patch('arm_ifcb.rospy.Time.now', return_value=rospy.Time.from_sec(601)):
            self.assertTrue(arm_ifcb.its_scheduled_depth_time(None), "Scheduled depth should run when the interval has passed.")


    @patch('arm_ifcb.rospy')
    @patch('arm_ifcb.arm')
    def test_its_scheduled_depth_time_peak_below_threshold(self, mock_arm, mock_rospy):
        mock_rospy.get_param.side_effect = [10, 0.5]  # Interval in minutes and peak value threshold
        mock_rospy.Duration.return_value = rospy.Duration(600)
        mock_arm.last_scheduled_time = rospy.Time.from_sec(100)
        with patch('arm_ifcb.rospy.Time.now', return_value=rospy.Time.from_sec(200)):
            self.assertTrue(arm_ifcb.its_scheduled_depth_time(0.1), "Scheduled depth should run when peak value is below threshold.")


# TODO: Figure out why get_param mocking seems to break here
# class TestScheduledDepth(unittest.TestCase):
#     @patch('arm_ifcb.rospy')
#     @patch('arm_ifcb.arm')
#     def test_scheduled_depth(self, mock_arm, mock_rospy):
#         mock_rospy.get_param.side_effect = [10, 20, 5]  # Range first, last, count
#         mock_arm.next_scheduled_index = 0
#         target_depth = arm_ifcb.scheduled_depth()
#         self.assertEqual(target_depth, 10.0, "The first scheduled depth should be the first element in the range.")
#         self.assertEqual(mock_arm.next_scheduled_index, 1, "The next scheduled index should increment.")

#         mock_arm.next_scheduled_index = 4
#         target_depth = arm_ifcb.scheduled_depth()
#         self.assertEqual(target_depth, 20.0, "The scheduled depth should wrap around to the start of the range.")

#         mock_arm.next_scheduled_index = 5
#         target_depth = arm_ifcb.scheduled_depth()
#         self.assertEqual(target_depth, 10.0, "The scheduled depth should loop back to the first element after reaching the end of the range.")

#         # Ensure last_scheduled_time is updated correctly
#         with patch('arm_ifcb.rospy.Time.now', return_value=rospy.Time.from_sec(1000)):
#             arm_ifcb.scheduled_depth()
#             self.assertEqual(mock_arm.last_scheduled_time.secs, 1000, "The last scheduled time should be updated to the current time.")


