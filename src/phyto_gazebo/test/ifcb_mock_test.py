#!/usr/bin/env python3
import json
import threading
import time
import unittest

import actionlib
import rospy
import rostest

from actionlib_msgs.msg import GoalStatus
from ds_core_msgs.msg import RawData
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool

from ifcb.instrumentation import parse_marker
from ifcb.srv import Command, RunRoutine
from phyto_arm.msg import RunIFCBAction, RunIFCBGoal


class TestIFCBMock(unittest.TestCase):
    def test_late_subscriber(self):
        topic = '/ifcb/is_routine_running'
        self.assertFalse(rospy.wait_for_message(topic, Bool, timeout=5).data)
        command = rospy.ServiceProxy('/ifcb/command', Command)
        command.wait_for_service(timeout=5)
        report = threading.Event()

        def on_input(msg):
            if (msg.data == b'reportevent:waiting' and
                    msg._connection_header['latching'] == '0'):
                report.set()

        rx = rospy.Subscriber('/ifcb/in', RawData, on_input)
        self.addCleanup(rx.unregister)
        deadline = time.monotonic() + 3
        while not rx.get_num_connections() and time.monotonic() < deadline:
            rospy.sleep(0.01)
        self.assertTrue(rx.get_num_connections())
        routine = [{'Type': 'Sequence', 'Steps': [
            {'StepType': 'Report', 'Arguments': ['waiting']},
            {'StepType': 'Wait', 'Arguments': [60000]},
        ]}]
        loaded = command('interactive:load:' + json.dumps(routine))
        self.assertTrue(loaded.success)
        try:
            self.assertTrue(command('interactive:start').success)
            self.assertTrue(report.wait(3))
            self.assertTrue(rospy.wait_for_message(topic, Bool, timeout=3).data)
            self.assertFalse(command('interactive:load:[]').success)
            self.assertFalse(command('interactive:start').success)
            self.assertTrue(command('daq:setdatafolder:/sim').success)
        finally:
            self.assertTrue(command('interactive:stop').success)

    def test_sampling(self):
        ready = rospy.wait_for_message('/ifcb/connected', Bool, timeout=5)
        self.assertTrue(ready.data)
        routine = rospy.ServiceProxy('/ifcb/routine', RunRoutine)
        self.assertFalse(routine(routine='missing').success)
        command = rospy.ServiceProxy('/ifcb/command', Command)
        self.assertTrue(command('daq:setdatafolder:/sim').success)

        # Track the release marker and the later end of the same sample
        release = threading.Event()
        stopped = threading.Event()
        image = threading.Event()

        def on_input(msg):
            text = msg.data.decode()
            if text.startswith('reportevent:'):
                marker = parse_marker(text.split(':', 1)[1])
                if (marker and marker['routine'] == 'runsample' and
                        marker['kind'] == 'before' and
                        marker['value'].get('StepType') == 'ChangeSpeed'):
                    release.set()
            if release.is_set() and text == 'valuechanged:interactive:stopped':
                stopped.set()

        rx = rospy.Subscriber('/ifcb/in', RawData, on_input)
        roi = rospy.Subscriber('/ifcb/roi/image/compressed', CompressedImage,
                               lambda msg: image.set() if msg.format == 'png'
                               else None)
        self.addCleanup(rx.unregister)
        self.addCleanup(roi.unregister)
        client = actionlib.SimpleActionClient('/arm_ifcb/ifcb_runner/sample',
                                              RunIFCBAction)
        self.assertTrue(client.wait_for_server(rospy.Duration(8)))
        client.send_goal(RunIFCBGoal())
        self.assertTrue(client.wait_for_result(rospy.Duration(12)))
        self.assertEqual(client.get_state(), GoalStatus.SUCCEEDED)
        self.assertTrue(release.wait(1))
        self.assertFalse(stopped.is_set())
        self.assertTrue(image.wait(4))
        self.assertTrue(stopped.wait(4))


if __name__ == '__main__':
    rospy.init_node('ifcb_mock_test')
    rostest.rosrun('phyto_gazebo', 'ifcb_mock', TestIFCBMock)
