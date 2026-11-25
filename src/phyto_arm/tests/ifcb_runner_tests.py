import json
import os
import sys
import unittest
from threading import Event, Thread
from unittest.mock import patch

from ds_core_msgs.msg import RawData
from ifcb.instrumentation import marker
from ifcb.srv import RunRoutineResponse

sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'src'))

import ifcb_runner


class TestSampleHold(unittest.TestCase):
    def setUp(self):
        report = marker('before', 'runsample', [0, 0],
                        ifcb_runner.SAMPLING_SPEED)
        self.release = 'reportevent:' + report['Arguments'][0]
        self.stop = 'valuechanged:interactive:stopped'

    def run_with_events(self, *events):
        def send(routine, instrument_before):
            selector, = instrument_before
            self.assertEqual(json.loads(selector), ifcb_runner.SAMPLING_SPEED)
            for event in events:
                ifcb_runner.on_ifcb_msg(RawData(data=event.encode()))
            return RunRoutineResponse(success=True)

        # All callbacks have run by the time the conductor checks the hold
        def check_ready(predicate):
            self.assertTrue(predicate())

        condition = ifcb_runner.state.position_hold_condition
        with patch.object(ifcb_runner.state, 'ifcb_is_idle', Event()), \
                patch.object(condition, 'wait_for', check_ready), \
                patch.object(ifcb_runner, 'ifcb_run_routine', send):
            ifcb_runner.run_routine('runsample')

    def test_marker_before_service_response(self):
        self.run_with_events(self.release)

    def test_stop_after_marker(self):
        self.run_with_events(self.release, self.stop)

    def test_stop_wakes_hold_waiter(self):
        condition = ifcb_runner.state.position_hold_condition
        idle = Event()
        stop = RawData(data=self.stop.encode())
        callback = Thread(target=ifcb_runner.on_ifcb_msg, args=(stop,))
        with patch.object(ifcb_runner.state, 'ifcb_is_idle', idle):
            # Let the callback take the lock once the waiter is listening
            with condition:
                callback.start()
                notified = condition.wait(timeout=5)
            callback.join()
        self.assertTrue(notified)
        self.assertTrue(idle.is_set())

    def test_late_marker_cannot_release_a_stopped_sample(self):
        with self.assertRaisesRegex(ValueError, 'stopped before'):
            self.run_with_events(self.stop, self.release)
