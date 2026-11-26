import unittest

from unittest.mock import Mock, patch

from phyto_arm.arm_chanos import clamp_steps, compute_profiler_steps, profiler_peak_ready
from rclpy.clock import ClockType
from rclpy.parameter import Parameter
from rclpy.time import Time


class TestProfilerPeakReady(unittest.TestCase):
    @patch('phyto_arm.arm_chanos.arm')
    def test_no_profiler_peak_available(self, mock_arm):
        mock_arm.latest_profile = None
        result = profiler_peak_ready()
        self.assertFalse(result)
        mock_arm.node.get_logger.return_value.warning.assert_called_with('No profiler peak available')

    @patch('phyto_arm.arm_chanos.arm')
    def test_profiler_peak_below_threshold(self, mock_arm):
        mock_arm.latest_profile = Mock()
        mock_arm.profiler_peak_value = 0.3
        mock_arm.node.get_parameter.return_value = Parameter('threshold', value=0.5)
        result = profiler_peak_ready()
        self.assertFalse(result)
        mock_arm.node.get_logger.return_value.warning.assert_called_with('Profiler peak value 0.3 is below threshold')

    @patch('phyto_arm.arm_chanos.arm')
    def test_profiler_peak_expired(self, mock_arm):
        mock_arm.latest_profile = Mock()
        mock_arm.latest_profile.header.stamp = Time(seconds=55).to_msg()
        mock_arm.profiler_peak_value = 0.6
        mock_arm.node.get_parameter.side_effect = [Parameter('value', value=v) for v in [0.5, 5]]
        mock_arm.node.get_clock.return_value.now.return_value = Time(seconds=61, clock_type=ClockType.ROS_TIME)
        result = profiler_peak_ready()
        self.assertFalse(result)
        mock_arm.node.get_logger.return_value.warning.assert_called_with(f'Profiler peak expired at {Time(seconds=60, clock_type=ClockType.ROS_TIME)}')

    @patch('phyto_arm.arm_chanos.arm')
    def test_profiler_peak_ready(self, mock_arm):
        mock_arm.latest_profile = Mock()
        mock_arm.latest_profile.header.stamp = Time(seconds=55).to_msg()
        mock_arm.profiler_peak_value = 0.6
        mock_arm.node.get_parameter.side_effect = [Parameter('value', value=v) for v in [0.5, 5]]
        mock_arm.node.get_clock.return_value.now.return_value = Time(seconds=60, clock_type=ClockType.ROS_TIME)
        result = profiler_peak_ready()
        self.assertTrue(result)


class TestClampStemps(unittest.TestCase):
    @patch('phyto_arm.arm_chanos.arm')
    def test_exceed_max(self, mock_arm):
        mock_arm.latest_profile = Mock()
        mock_arm.node.get_parameter.side_effect = [Parameter('value', value=v) for v in [10.0, 0.0]]
        result = clamp_steps([4.0, 6.0, 11.0])
        self.assertEqual(result, [4.0, 6.0, 10.0])

    @patch('phyto_arm.arm_chanos.arm')
    def test_exceed_min(self, mock_arm):
        mock_arm.latest_profile = Mock()
        mock_arm.node.get_parameter.side_effect = [Parameter('value', value=v) for v in [10.0, 5.0]]
        result = clamp_steps([4.0, 6.0, 10.0])
        self.assertEqual(result, [5.0, 6.0, 10.0])


class TestComputeProfilerSteps(unittest.TestCase):
    @patch('phyto_arm.arm_chanos.arm')
    def test_no_profiler_peak_available(self, mock_arm):
        mock_arm.latest_profile = None
        with self.assertRaises(ValueError) as exc:
            compute_profiler_steps(True)
        self.assertEqual(str(exc.exception), 'No profiler peak available')

    @patch('phyto_arm.arm_chanos.arm')
    def test_compute_profiler_steps(self, mock_arm):
        mock_arm.latest_profile = Mock()
        mock_arm.profiler_peak_depth = 5.0
        mock_arm.node.get_parameter.side_effect = [Parameter('value', value=v) for v in [[-1.0, 1.0, 2.0], 10.0, 0.0]]
        result = compute_profiler_steps(True)
        self.assertEqual(result, [4.0, 6.0, 7.0])

    @patch('phyto_arm.arm_chanos.arm')
    def test_compute_profiler_steps_upcast(self, mock_arm):
        mock_arm.latest_profile = Mock()
        mock_arm.profiler_peak_depth = 5.0
        mock_arm.node.get_parameter.side_effect = [Parameter('value', value=v) for v in [[-1.0, 1.0, 2.0], 10.0, 0.0]]
        result = compute_profiler_steps(False)
        self.assertEqual(result, [7.0, 6.0, 4.0])

    @patch('phyto_arm.arm_chanos.arm')
    def test_compute_profiler_steps_exceeds_max(self, mock_arm):
        mock_arm.latest_profile = Mock()
        mock_arm.profiler_peak_depth = 9.0
        mock_arm.node.get_parameter.side_effect = [Parameter('value', value=v) for v in [[1.0, 2.0], 10.0, 0.0]]
        result = compute_profiler_steps(True)
        self.assertEqual(result, [10.0, 10.0])

    @patch('phyto_arm.arm_chanos.arm')
    def test_compute_profiler_steps_less_than_min(self, mock_arm):
        mock_arm.latest_profile = Mock()
        mock_arm.profiler_peak_depth = 1.0
        mock_arm.node.get_parameter.side_effect = [Parameter('value', value=v) for v in [[-2.0, -1.0], 10.0, 0.0]]
        result = compute_profiler_steps(True)
        self.assertEqual(result, [0.0, 0.0])
