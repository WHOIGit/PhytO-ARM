#!/usr/bin/env python3
import queue
import time
import unittest

import rclpy

from threading import Thread

from launch import LaunchDescription
from launch_ros.actions import Node
from launch_testing.actions import ReadyToTest
from phyto_arm_msgs.action import MoveToDepth
from phyto_arm_msgs.msg import DepthProfile
from rclpy.action import ActionServer
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.task import Future
from std_msgs.msg import String


DWELL = 0.3


def generate_test_description():
    return LaunchDescription([
        Node(package='phyto_arm', executable='lock_manager.py', name='lock_manager',
             parameters=[{'max_moving_winches': 1}]),
        Node(package='phyto_arm', executable='arm_chanos.py', name='arm', namespace='arm_chanos',
             parameters=[{'winch_enabled': True, 'winch.range.min': 0.5, 'winch.range.max': 0.9,
                          'winch.max_speed': 0.04, 'tasks.continuous_speed': 0.04,
                          'tasks.downcast_type': 'stepped', 'tasks.upcast_type': 'continuous',
                          'tasks.dwell_time': DWELL, 'tasks.default_steps': [0.60],
                          'tasks.profiler_peak.enabled': True, 'tasks.profiler_peak.threshold': 1.0,
                          'tasks.profiler_peak.peak_expiration': 60.0,
                          'tasks.profiler_peak.offset_steps': [-0.035, 0.035]}]),
        ReadyToTest(),
    ])


class ChanosCastTest(unittest.TestCase):
    def setUp(self):
        self.goals = queue.Queue()
        self.moves_sent = queue.Queue()
        rclpy.init()
        self.addCleanup(rclpy.shutdown)
        self.node = rclpy.create_node('chanos_cast_test', namespace='/arm_chanos')
        self.addCleanup(self.node.destroy_node)
        self.node.create_subscription(String, 'task', self.receive_task, 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.profile = self.node.create_publisher(DepthProfile, '/arm_ifcb/profiler', latched)
        self.winch = ActionServer(self.node, MoveToDepth, 'winch/move_to_depth', self.receive_goal)
        self.addCleanup(self.winch.destroy)
        executor = SingleThreadedExecutor()
        executor.add_node(self.node)
        thread = Thread(target=executor.spin)
        thread.start()
        self.addCleanup(thread.join)
        self.addCleanup(executor.shutdown)

    async def receive_goal(self, handle):
        result = Future()
        self.goals.put((handle, result, time.monotonic()))
        return await result

    def receive_task(self, msg):
        if msg.data == 'winch_movement':
            self.moves_sent.put(True)

    def expect_move(self, depth, dwell_after=None):
        try:
            self.goal, self.result, received = self.goals.get(timeout=8)
        except queue.Empty:
            self.fail(f'No winch goal for {depth} m')
        # The arm publishes this after send_goal returns. Wait before replying
        # so the test winch cannot finish before the action client tracks its goal.
        self.moves_sent.get(timeout=8)
        self.assertAlmostEqual(self.goal.request.depth, depth, places=5)
        if dwell_after is not None:
            self.assertGreaterEqual(received - dwell_after, DWELL)

    def finish_move(self):
        finished = time.monotonic()
        self.goal.succeed()
        self.result.set_result(MoveToDepth.Result())
        return finished

    def test_profile_sets_the_next_cast_depths(self):
        self.expect_move(0.50)
        self.finish_move()
        self.expect_move(0.60)  # Default step on the first downcast.

        # A profile received mid-cast should guide the next stepped downcast.
        profile = DepthProfile()
        profile.header.stamp = self.node.get_clock().now().to_msg()
        profile.depths = [0.60, 0.70, 0.80]
        profile.values = [1.0, 3.0, 2.0]
        self.profile.publish(profile)
        finished = self.finish_move()
        self.expect_move(0.90, dwell_after=finished)
        self.finish_move()
        self.expect_move(0.50)
        self.finish_move()

        self.expect_move(0.665)
        finished = self.finish_move()
        self.expect_move(0.735, dwell_after=finished)
        finished = self.finish_move()
        self.expect_move(0.90, dwell_after=finished)
        self.finish_move()
        self.expect_move(0.50)
