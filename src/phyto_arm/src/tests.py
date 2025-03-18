#!/usr/bin/env python3
PKG="phyto_arm"

from datetime import datetime, timedelta
import os
import rospy
import sys
import unittest
from unittest.mock import patch, Mock
import socket
import time


# Hack for getting this to recognize local modules for import
# Have not found a ROS-recommended way to import modules being tested
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from arm_chanos import profiler_peak_ready, compute_profiler_steps, clamp_steps
import arm_ifcb
from lock_manager import NamedLockManager
import network_data_capture
from network_data_capture import (
    validate_config, validate_topic_config, validate_subtopic_config,
    convert_to_ros_msg, parse_delimited_message, parse_json_dict_message,
    parse_json_array_message, extract_messages, append_to_buffer
)


class TestProfilerPeakReady(unittest.TestCase):
    @patch('arm_chanos.arm')
    @patch('arm_chanos.rospy')
    def test_no_profiler_peak_available(self, mock_rospy, mock_arm):
        mock_arm.latest_profile = None
        mock_rospy.logwarn = Mock()
        result = profiler_peak_ready()
        self.assertFalse(result)
        mock_rospy.logwarn.assert_called_with('No profiler peak available')

    @patch('arm_chanos.arm')
    @patch('arm_chanos.rospy')
    def test_profiler_peak_below_threshold(self, mock_rospy, mock_arm):
        mock_arm.latest_profile = Mock()
        mock_arm.profiler_peak_value = 0.3
        mock_rospy.logwarn = Mock()
        mock_rospy.get_param.return_value = 0.5
        result = profiler_peak_ready()
        self.assertFalse(result)
        mock_rospy.logwarn.assert_called_with('Profiler peak value 0.3 is below threshold')

    @patch('arm_chanos.arm')
    @patch('arm_chanos.rospy')
    def test_profiler_peak_expired(self, mock_rospy, mock_arm):
        mock_arm.latest_profile = Mock()
        mock_arm.latest_profile.header.stamp = datetime(2023, 1, 1, 11, 59, 55)
        mock_arm.profiler_peak_value = 0.6
        mock_rospy.logwarn = Mock()
        mock_rospy.get_param.side_effect = [0.5, 5]
        mock_rospy.Duration.return_value = timedelta(seconds=5)
        mock_rospy.Time.now.return_value = datetime(2023, 1, 1, 12, 0, 1)
        result = profiler_peak_ready()
        self.assertFalse(result)
        mock_rospy.logwarn.assert_called_with('Profiler peak expired at 2023-01-01 12:00:00')

    @patch('arm_chanos.arm')
    @patch('arm_chanos.rospy')
    def test_profiler_peak_ready(self, mock_rospy, mock_arm):
        mock_arm.latest_profile = Mock()
        mock_arm.latest_profile.header.stamp = datetime(2023, 1, 1, 11, 59, 55)
        mock_arm.profiler_peak_value = 0.6
        mock_rospy.get_param.side_effect = [0.5, 5]
        mock_rospy.Duration.return_value = timedelta(seconds=5)
        mock_rospy.Time.now.return_value = datetime(2023, 1, 1, 12, 0, 0)
        result = profiler_peak_ready()
        self.assertTrue(result)


class TestClampStemps(unittest.TestCase):
    @patch('arm_chanos.arm')
    @patch('arm_chanos.rospy')
    def test_exceed_max(self, mock_rospy, mock_arm):
        mock_arm.latest_profile = Mock()
        mock_rospy.get_param.side_effect = [10.0, 0.0]
        result = clamp_steps([4.0, 6.0, 11.0])
        self.assertEqual(result, [4.0, 6.0, 10.0])

    @patch('arm_chanos.arm')
    @patch('arm_chanos.rospy')
    def test_exceed_min(self, mock_rospy, mock_arm):
        mock_arm.latest_profile = Mock()
        mock_rospy.get_param.side_effect = [10.0, 5.0]
        result = clamp_steps([4.0, 6.0, 10.0])
        self.assertEqual(result, [5.0, 6.0, 10.0])


class TestComputeProfilerSteps(unittest.TestCase):
    @patch('arm_chanos.arm')
    @patch('arm_chanos.rospy')
    def test_no_profiler_peak_available(self, mock_rospy, mock_arm):
        mock_arm.latest_profile = None
        with self.assertRaises(ValueError) as exc:
            compute_profiler_steps(True)
        self.assertEqual(str(exc.exception), 'No profiler peak available')

    @patch('arm_chanos.arm')
    @patch('arm_chanos.rospy')
    def test_compute_profiler_steps(self, mock_rospy, mock_arm):
        mock_arm.latest_profile = Mock()
        mock_arm.profiler_peak_depth = 5.0
        mock_rospy.get_param.side_effect = [[-1.0, 1.0, 2.0], 10.0, 0.0]
        result = compute_profiler_steps(True)
        self.assertEqual(result, [4.0, 6.0, 7.0])

    @patch('arm_chanos.arm')
    @patch('arm_chanos.rospy')
    def test_compute_profiler_steps_upcast(self, mock_rospy, mock_arm):
        mock_arm.latest_profile = Mock()
        mock_arm.profiler_peak_depth = 5.0
        mock_rospy.get_param.side_effect = [[-1.0, 1.0, 2.0], 10.0, 0.0]
        result = compute_profiler_steps(False)
        self.assertEqual(result, [7.0, 6.0, 4.0])

    @patch('arm_chanos.arm')
    @patch('arm_chanos.rospy')
    def test_compute_profiler_steps_exceeds_max(self, mock_rospy, mock_arm):
        mock_arm.latest_profile = Mock()
        mock_arm.profiler_peak_depth = 9.0
        mock_rospy.get_param.side_effect = [[1.0, 2.0], 10.0, 0.0]
        result = compute_profiler_steps(True)
        self.assertEqual(result, [10.0, 10.0])

    @patch('arm_chanos.arm')
    @patch('arm_chanos.rospy')
    def test_compute_profiler_steps_less_than_min(self, mock_rospy, mock_arm):
        mock_arm.latest_profile = Mock()
        mock_arm.profiler_peak_depth = 1.0
        mock_rospy.get_param.side_effect = [[-2.0, -1.0], 10.0, 0.0]
        result = compute_profiler_steps(True)
        self.assertEqual(result, [0.0, 0.0])

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
    def test_use_profiler_peak_disabled(self, mock_rospy):
        mock_rospy.get_param.return_value = False  # Disable use_profiler_peak
        default_depth = 100  # Set a default depth for when profiler_peak usage is disabled
        mock_rospy.get_param.return_value = default_depth
        result = arm_ifcb.compute_wiz_depth(50)  # The input value shouldn't matter in this case
        self.assertEqual(result, default_depth, "Should return default depth when use_profiler_peak is disabled.")

    @patch('arm_ifcb.rospy')
    def test_no_profiler_peak_depth_available(self, mock_rospy):
        mock_rospy.get_param.side_effect = [200, True, 200, 250, 10]  # Enable use_profiler_peak, set default depth to 200
        result = arm_ifcb.compute_wiz_depth(None)  # No peak depth available
        self.assertEqual(result, 200, "Should return default depth when no profiler peak depth is available.")

    @patch('arm_ifcb.rospy')
    def test_preferred_depth_within_winch_range(self, mock_rospy):
        mock_rospy.get_param.side_effect = lambda param: {
            'tasks/wiz_probe/use_profiler_peak': True,
            'tasks/wiz_probe/peak_offset': 10,
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
            'tasks/wiz_probe/use_profiler_peak': True,
            'tasks/wiz_probe/peak_offset': 50,
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
            'tasks/wiz_probe/use_profiler_peak': True,
            'tasks/wiz_probe/peak_offset': -20,
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


class TestNetworkDataCaptureValidation(unittest.TestCase):
    def test_validate_config_valid(self):
        config = {
            "topics": {
                "test_topic": {
                    "connection_type": "udp",
                    "port": 8080,
                    "parsing_strategy": "json_dict",
                    "subtopics": {
                        "value": {
                            "field_id": "data",
                            "type": "float"
                        }
                    }
                }
            }
        }
        # No exception should be raised
        validate_config(config)

    def test_validate_config_missing_topics(self):
        config = {}
        with self.assertRaises(network_data_capture.ConfigurationError) as context:
            validate_config(config)
        self.assertIn("must contain 'topics'", str(context.exception))

    def test_validate_topic_config_valid(self):
        topic_config = {
            "connection_type": "udp",
            "port": 8080,
            "parsing_strategy": "json_dict",
            "subtopics": {
                "value": {
                    "field_id": "data",
                    "type": "float"
                }
            }
        }
        # No exception should be raised
        validate_topic_config("test_topic", topic_config)

    def test_validate_topic_config_missing_field(self):
        topic_config = {
            "connection_type": "udp",
            "parsing_strategy": "json_dict",
            "subtopics": {}
        }
        with self.assertRaises(network_data_capture.ConfigurationError) as context:
            validate_topic_config("test_topic", topic_config)
        self.assertIn("Missing required field 'port'", str(context.exception))

    def test_validate_topic_config_invalid_connection_type(self):
        topic_config = {
            "connection_type": "invalid",
            "port": 8080,
            "parsing_strategy": "json_dict",
            "subtopics": {}
        }
        with self.assertRaises(network_data_capture.ConfigurationError) as context:
            validate_topic_config("test_topic", topic_config)
        self.assertIn("Invalid connection_type", str(context.exception))

    def test_validate_topic_config_missing_delimiter(self):
        topic_config = {
            "connection_type": "udp",
            "port": 8080,
            "parsing_strategy": "delimited",
            "subtopics": {}
        }
        with self.assertRaises(network_data_capture.ConfigurationError) as context:
            validate_topic_config("test_topic", topic_config)
        self.assertIn("Delimiter required", str(context.exception))

    def test_validate_topic_config_delimited_with_array_type(self):
        topic_config = {
            "connection_type": "udp",
            "port": 8080,
            "parsing_strategy": "delimited",
            "delimiter": ",",
            "subtopics": {
                "values": {
                    "field_id": 0,
                    "type": "float[]"
                }
            }
        }
        with self.assertRaises(network_data_capture.ConfigurationError) as context:
            validate_topic_config("test_topic", topic_config)
        self.assertIn("Arrays are not a supported field type for delimited string parsing", str(context.exception))

    def test_validate_subtopic_config_valid(self):
        topic_config = {
            "parsing_strategy": "json_dict",
            "subtopics": {
                "test_subtopic": {
                    "field_id": "data",
                    "type": "float"
                }
            }
        }
        subtopic_config = topic_config["subtopics"]["test_subtopic"]
        # No exception should be raised
        validate_subtopic_config(subtopic_config, topic_config["parsing_strategy"])

    def test_validate_subtopic_config_missing_field(self):
        topic_config = {
            "parsing_strategy": "json_dict",
            "subtopics": {
                "test_subtopic": {
                    "field_id": "data"
                }
            }
        }
        subtopic_config = topic_config["subtopics"]["test_subtopic"]
        with self.assertRaises(network_data_capture.ConfigurationError) as context:
            validate_subtopic_config(subtopic_config, topic_config["parsing_strategy"])
        self.assertIn("Missing required field 'type'", str(context.exception))

    def test_validate_subtopic_config_invalid_type(self):
        topic_config = {
            "parsing_strategy": "json_dict",
            "subtopics": {
                "test_subtopic": {
                    "field_id": "data",
                    "type": "invalid_type"
                }
            }
        }
        subtopic_config = topic_config["subtopics"]["test_subtopic"]
        with self.assertRaises(network_data_capture.ConfigurationError) as context:
            validate_subtopic_config(subtopic_config, topic_config["parsing_strategy"])
        self.assertIn("Invalid type", str(context.exception))


class TestNetworkDataCaptureMessageProcessing(unittest.TestCase):
    def test_convert_to_ros_msg_float(self):
        msg = convert_to_ros_msg(42.5, "float")
        self.assertEqual(msg.data, 42.5)

    def test_convert_to_ros_msg_string(self):
        msg = convert_to_ros_msg("test", "str")
        self.assertEqual(msg.data, "test")

    def test_convert_to_ros_msg_int(self):
        msg = convert_to_ros_msg(42, "int")
        self.assertEqual(msg.data, 42)

    def test_convert_to_ros_msg_bool(self):
        msg = convert_to_ros_msg(True, "bool")
        self.assertEqual(msg.data, True)

    def test_convert_to_ros_msg_float_array(self):
        msg = convert_to_ros_msg([1.0, 2.0, 3.0], "float[]")
        self.assertEqual(list(msg.data), [1.0, 2.0, 3.0])

    def test_convert_to_ros_msg_int_array(self):
        msg = convert_to_ros_msg([1, 2, 3], "int[]")
        self.assertEqual(list(msg.data), [1, 2, 3])

    def test_convert_to_ros_msg_bool_array(self):
        msg = convert_to_ros_msg([True, False, True], "bool[]")
        self.assertEqual(list(msg.data), [1, 0, 1])

    def test_convert_to_ros_msg_type_mismatch(self):
        with self.assertRaises(network_data_capture.ConversionError):
            convert_to_ros_msg("not_a_number", "float")

    def test_append_to_buffer(self):
        buffer = append_to_buffer(b"hello", b" world")
        self.assertEqual(buffer, b"hello world")

    def test_append_to_buffer_overflow(self):
        with self.assertRaises(BufferError):
            append_to_buffer(b"hello", b" world", max_size=10)

    def test_extract_messages_raw(self):
        buffer = b"line1\nline2\nline3"
        messages, remaining = extract_messages(buffer, "raw", {})
        self.assertEqual(messages, [b"line1", b"line2"])

        # Remaining because raw only splits on newlines
        self.assertEqual(remaining, b"line3")

    def test_extract_messages_delimited(self):
        buffer = b"a,b,c\nd,e,f\ng,h"
        config = {"delimiter": ",", "use_regex_delimiter": False}
        messages, remaining = extract_messages(buffer, "delimited", config)
        self.assertEqual(messages, [b"a,b,c", b"d,e,f"])
        self.assertEqual(remaining, b"g,h")

    def test_extract_messages_delimited_with_regex(self):
        buffer = b"a, b c\nd, e,  f\ng, h"
        config = {"delimiter": ",?\\s+", "use_regex_delimiter": True}
        messages, remaining = extract_messages(buffer, "delimited", config)
        self.assertEqual(messages, [b"a, b c", b"d, e,  f"])
        self.assertEqual(remaining, b"g, h")

    def test_extract_messages_json_dict(self):
        buffer = b'{"a":1}\n{"b":2}\n{"c":'
        messages, remaining = extract_messages(buffer, "json_dict", {})
        self.assertEqual(len(messages), 2)
        self.assertEqual(remaining, b'{"c":')

    def test_extract_messages_invalid_strategy(self):
        buffer = b"some data"
        with self.assertRaises(network_data_capture.ParsingError):
            extract_messages(buffer, "invalid_strategy", {})

    def test_parse_delimited_message(self):
        message = b"42.5,test,1"
        config = {
            "delimiter": ",",
            "use_regex_delimiter": False,
            "subtopics": {
                "value": {"field_id": 0, "type": "float"},
                "name": {"field_id": 1, "type": "str"},
                "flag": {"field_id": 2, "type": "bool"}
            }
        }
        result = parse_delimited_message(message, config)
        self.assertEqual(result["value"], 42.5)
        self.assertEqual(result["name"], "test")
        self.assertEqual(result["flag"], True)

    def test_parse_delimited_message_with_regex(self):
        message = b"42.5, test  1"
        config = {
            "delimiter": ",?\\s+",
            "use_regex_delimiter": True,
            "subtopics": {
                "value": {"field_id": 0, "type": "float"},
                "name": {"field_id": 1, "type": "str"},
                "flag": {"field_id": 2, "type": "bool"}
            }
        }
        result = parse_delimited_message(message, config)
        self.assertEqual(result["value"], 42.5)
        self.assertEqual(result["name"], "test")
        self.assertEqual(result["flag"], True)

    def test_parse_delimited_message_field_out_of_range(self):
        message = b"42.5,test"
        config = {
            "delimiter": ",",
            "subtopics": {
                "value": {"field_id": 0, "type": "float"},
                "name": {"field_id": 1, "type": "str"},
                "flag": {"field_id": 2, "type": "bool"}
            }
        }

        with self.assertRaises(network_data_capture.ParsingError):
            parse_delimited_message(message, config)

    def test_parse_json_dict_message(self):
        message = b'{"value": 42.5, "name": "test", "flag": true}'
        config = {
            "subtopics": {
                "value": {"field_id": "value", "type": "float"},
                "name": {"field_id": "name", "type": "str"},
                "flag": {"field_id": "flag", "type": "bool"}
            }
        }
        result = parse_json_dict_message(message, config)
        self.assertEqual(result["value"], 42.5)
        self.assertEqual(result["name"], "test")
        self.assertEqual(result["flag"], True)

    def test_parse_json_dict_message_missing_field(self):
        message = b'{"value": 42.5, "flag": true}'
        config = {
            "subtopics": {
                "value": {"field_id": "value", "type": "float"},
                "name": {"field_id": "name", "type": "str"},
                "flag": {"field_id": "flag", "type": "bool"}
            }
        }
        with self.assertRaises(network_data_capture.ParsingError):
            parse_json_dict_message(message, config)

    def test_parse_json_array_message(self):
        message = b'[42.5, "test", true]'
        config = {
            "subtopics": {
                "value": {"field_id": 0, "type": "float"},
                "name": {"field_id": 1, "type": "str"},
                "flag": {"field_id": 2, "type": "bool"}
            }
        }
        result = parse_json_array_message(message, config)
        self.assertEqual(result["value"], 42.5)
        self.assertEqual(result["name"], "test")
        self.assertEqual(result["flag"], True)

    def test_parse_json_array_message_index_out_of_range(self):
        message = b'[42.5, "test"]'
        config = {
            "subtopics": {
                "value": {"field_id": 0, "type": "float"},
                "name": {"field_id": 1, "type": "str"},
                "flag": {"field_id": 2, "type": "bool"}
            }
        }
        with self.assertRaises(network_data_capture.ParsingError):
            parse_json_array_message(message, config)

    @patch('socket.socket')
    def test_create_socket_udp(self, mock_socket):
        mock_socket_instance = Mock()
        mock_socket.return_value = mock_socket_instance

        network_data_capture.create_socket("udp", 8080)

        mock_socket.assert_called_with(socket.AF_INET, socket.SOCK_DGRAM)
        mock_socket_instance.bind.assert_called_with(("0.0.0.0", 8080))

    @patch('socket.socket')
    def test_create_socket_tcp(self, mock_socket):
        mock_socket_instance = Mock()
        mock_socket.return_value = mock_socket_instance

        network_data_capture.create_socket("tcp", 8080)

        mock_socket.assert_called_with(socket.AF_INET, socket.SOCK_STREAM)
        mock_socket_instance.bind.assert_called_with(("0.0.0.0", 8080))
        mock_socket_instance.listen.assert_called_with(1)

    @patch('socket.socket')
    def test_create_socket_error(self, mock_socket):
        mock_socket.side_effect = socket.error("Test socket error")

        with self.assertRaises(network_data_capture.SocketError):
            network_data_capture.create_socket("udp", 8080)


class TestTopicStats(unittest.TestCase):
    def test_update_rates(self):
        stats = network_data_capture.TopicStats()

        # Add some message times
        current_time = time.time()
        stats.message_times = [
            current_time - 9,
            current_time - 7,
            current_time - 5,
            current_time - 3,
            current_time - 1
        ]

        # Add some error times
        stats.error_times = [
            current_time - 8,
            current_time - 4
        ]

        # Update rates
        stats.update_rates(current_time)

        # Check rates
        self.assertEqual(stats.message_rate, 0.5)  # 5 messages / 10 seconds
        self.assertEqual(stats.error_rate, 0.2)    # 2 errors / 10 seconds

        # Add an old message that should be removed
        stats.message_times.append(current_time - 15)
        stats.update_rates(current_time)

        # The old message should be removed
        self.assertEqual(stats.message_rate, 0.5)  # Still 5 messages / 10 seconds
