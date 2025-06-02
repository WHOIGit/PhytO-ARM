from datetime import datetime, timedelta
import os
import sys
import unittest
from unittest.mock import Mock, patch

sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'src'))

from arm_chanos import clamp_steps, compute_profiler_steps, profiler_peak_ready


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
