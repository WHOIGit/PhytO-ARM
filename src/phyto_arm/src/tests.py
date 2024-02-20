#!/usr/bin/env python3
PKG="phyto_arm"
import os
import sys

from datetime import datetime
import numpy as np
import rospy
from unittest.mock import patch, Mock
import unittest

# Hack for getting this to recognize local modules for import
# Have not found a ROS-recommended way to import modules being tested
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
import arm_ifcb

from datetime import timedelta
from unittest.mock import patch, MagicMock

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


