#!/usr/bin/env python3
import json
import threading

from pathlib import Path

from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from ifcb.instrumentation import parse_marker
from ifcb_msgs.srv import Command, RunRoutine
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_testing.actions import ReadyToTest
from phyto_arm_msgs.action import RunIFCB
from rclpy.action import ActionClient
from rclpy.qos import DurabilityPolicy, QoSProfile
from ros_test import RosTest
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool
from wr2_msgs.msg import RawData


LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)


def generate_test_description():
    launch = Path(get_package_share_directory('ifcb')) / 'launch' / 'ifcb_mock.launch.py'
    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(launch))),
        Node(package='phyto_arm', executable='ifcb_runner.py', name='ifcb_runner',
             namespace='arm_ifcb', parameters=[{
                 'ifcb_maintenance.bead_interval': 0,
                 'ifcb_maintenance.cartridge_debubble_interval': 0,
                 'ifcb_maintenance.clean_interval': 0,
                 'ifcb_maintenance.flush_after_sample': False,
             }]),
        ReadyToTest(),
    ])


class TestIFCBMock(RosTest):
    def setUp(self):
        super().setUp()
        self.command_client = self.node.create_client(Command, '/ifcb/command')

    def command(self, command):
        return self.call(self.command_client, Command.Request(command=command))

    def test_late_subscriber(self):
        topic = '/ifcb/is_routine_running'
        self.assertFalse(self.wait_for_message(topic, Bool, timeout=5, qos=LATCHED).data)
        report = threading.Event()

        def on_input(msg):
            if bytes(msg.data) == b'reportevent:waiting':
                report.set()

        self.node.create_subscription(RawData, '/ifcb/in', on_input, 10)
        self.assertTrue(self.wait_until(lambda: self.node.count_publishers('/ifcb/in'), 3))
        publishers = self.node.get_publishers_info_by_topic('/ifcb/in')
        self.assertTrue(publishers)
        self.assertTrue(all(p.qos_profile.durability == DurabilityPolicy.VOLATILE
                            for p in publishers))
        routine = [{'Type': 'Sequence', 'Steps': [
            {'StepType': 'Report', 'Arguments': ['waiting']},
            {'StepType': 'Wait', 'Arguments': [60000]},
        ]}]
        loaded = self.command('interactive:load:' + json.dumps(routine))
        self.assertTrue(loaded.success)
        try:
            self.assertTrue(self.command('interactive:start').success)
            self.assertTrue(self.wait_until(report.is_set, 3))
            self.assertTrue(self.wait_for_message(topic, Bool, timeout=3, qos=LATCHED).data)
            self.assertFalse(self.command('interactive:load:[]').success)
            self.assertFalse(self.command('interactive:start').success)
            self.assertTrue(self.command('daq:setdatafolder:/sim').success)
        finally:
            self.assertTrue(self.command('interactive:stop').success)

    def test_sampling(self):
        ready = self.wait_for_message('/ifcb/connected', Bool, timeout=5, qos=LATCHED)
        self.assertTrue(ready.data)
        routine = self.node.create_client(RunRoutine, '/ifcb/routine')
        self.assertFalse(self.call(routine, RunRoutine.Request(routine='missing')).success)
        self.assertTrue(self.command('daq:setdatafolder:/sim').success)

        # Track the release marker and the later end of the same sample
        release = threading.Event()
        stopped = threading.Event()
        image = threading.Event()

        def on_input(msg):
            text = bytes(msg.data).decode()
            if text.startswith('reportevent:'):
                marker = parse_marker(text.split(':', 1)[1])
                if (marker and marker['routine'] == 'runsample' and
                        marker['kind'] == 'before' and
                        marker['value'].get('StepType') == 'ChangeSpeed'):
                    release.set()
            if release.is_set() and text == 'valuechanged:interactive:stopped':
                stopped.set()

        self.node.create_subscription(RawData, '/ifcb/in', on_input, 10)
        self.node.create_subscription(CompressedImage, '/ifcb/roi/image/compressed',
                                      lambda msg: image.set() if msg.format == 'png' else None, 10)
        client = ActionClient(self.node, RunIFCB, '/arm_ifcb/ifcb_runner/sample')
        self.addCleanup(client.destroy)
        self.assertTrue(client.wait_for_server(timeout_sec=8))
        goal = self.result(client.send_goal_async(RunIFCB.Goal()), 8)
        self.assertTrue(goal.accepted)
        result = self.result(goal.get_result_async(), 12)
        self.assertEqual(result.status, GoalStatus.STATUS_SUCCEEDED)
        self.assertTrue(self.wait_until(release.is_set, 1))
        self.assertFalse(stopped.is_set())
        self.assertTrue(self.wait_until(image.is_set, 4))
        self.assertTrue(self.wait_until(stopped.is_set, 4))
