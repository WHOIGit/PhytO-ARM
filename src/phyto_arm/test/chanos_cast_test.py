#!/usr/bin/env python3
import queue
import time
import unittest

import actionlib
import rospy
import rostest

from phyto_arm.msg import DepthProfile, MoveToDepthAction, MoveToDepthResult
from std_msgs.msg import String


class ChanosCastTest(unittest.TestCase):
    def setUp(self):
        self.goals = queue.Queue()
        self.moves_sent = queue.Queue()
        self.task = rospy.Subscriber('task', String, self.receive_task)
        self.dwell = rospy.get_param('tasks/dwell_time')
        self.profile = rospy.Publisher('/arm_ifcb/profiler', DepthProfile, queue_size=1, latch=True)
        self.winch = actionlib.SimpleActionServer('winch/move_to_depth', MoveToDepthAction,
                                                 auto_start=False)
        self.winch.register_goal_callback(self.receive_goal)
        self.winch.start()

    def receive_goal(self):
        goal = self.winch.accept_new_goal()
        self.goals.put((goal.depth, time.monotonic()))

    def receive_task(self, msg):
        if msg.data == 'winch_movement':
            self.moves_sent.put(True)

    def expect_move(self, depth, dwell_after=None):
        try:
            actual, received = self.goals.get(timeout=8)
        except queue.Empty:
            self.fail(f'No winch goal for {depth} m')
        # The arm publishes this after send_goal returns. Wait before replying
        # so the test winch cannot finish before the action client tracks its goal.
        self.moves_sent.get(timeout=8)
        self.assertAlmostEqual(actual, depth, places=5)
        if dwell_after is not None:
            self.assertGreaterEqual(received - dwell_after, self.dwell)

    def finish_move(self):
        finished = time.monotonic()
        self.winch.set_succeeded(MoveToDepthResult())
        return finished

    def test_profile_sets_the_next_cast_depths(self):
        self.expect_move(0.50)
        self.finish_move()
        self.expect_move(0.60)  # Default step on the first downcast.

        # A profile received mid-cast should guide the next stepped downcast.
        profile = DepthProfile()
        profile.header.stamp = rospy.Time.now()
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


if __name__ == '__main__':
    rospy.init_node('chanos_cast_test')
    rostest.rosrun('phyto_arm', 'chanos_cast', ChanosCastTest)
