#!/usr/bin/env python3
import math
import struct
import threading
import unittest

import actionlib
import rospy
import rostest

from actionlib_msgs.msg import GoalStatus
from dli_power_switch.msg import OutletStatus
from ds_sensor_msgs.msg import Ctd, DepthPressure
from gazebo_msgs.srv import GetLinkState
from gps_common.msg import GPSFix
from jvl_motor.msg import Motion
from jvl_motor.srv import ZeroCmd
from phyto_arm.msg import MoveToDepthAction, MoveToDepthGoal
from pyModbusTCP.client import ModbusClient
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import Bool, Float64
from tf.transformations import quaternion_matrix


class TestGazeboWinch(unittest.TestCase):
    def setUp(self):
        self.client = actionlib.SimpleActionClient('winch/move_to_depth',
                                                   MoveToDepthAction)
        self.assertTrue(self.client.wait_for_server(rospy.Duration(40)))
        self.link_state = rospy.ServiceProxy('/gazebo/get_link_state',
                                             GetLinkState)
        self.link_state.wait_for_service(timeout=30)
        self.bus = ModbusClient(host='127.0.0.1', port=rospy.get_param('~port'),
                                timeout=2, auto_open=True)
        self.addCleanup(self.bus.close)

    def register(self, number):
        words = self.bus.read_holding_registers(2 * number, 2)
        self.assertIsNotNone(words, f'Failed to read MAC400 register {number}')
        return struct.unpack('<i', struct.pack('<HH', *words))[0]

    def probe_depth(self):
        response = self.link_state('ifcb::body', 'world')
        self.assertTrue(response.success, response.status_message)
        pose = response.link_state.pose
        q = pose.orientation
        rotation = quaternion_matrix((q.x, q.y, q.z, q.w))
        offset = rotation[2].dot((0.185, 0, -0.1, 0))
        return -(pose.position.z + offset)

    def cast(self, target, direction):
        seen_motion = threading.Event()

        def check_motion(msg):
            if (msg.mode == Motion.MODE_VELOCITY and
                    direction * msg.velocity > 0):
                seen_motion.set()

        subscription = rospy.Subscriber('motor/motion', Motion, check_motion)
        done = threading.Event()
        try:
            self.client.send_goal(MoveToDepthGoal(depth=target, velocity=0.18),
                                  done_cb=lambda state, result: done.set())
            self.assertTrue(done.wait(85), f'Timed out moving to {target} m')
            self.assertEqual(self.client.get_state(), GoalStatus.SUCCEEDED,
                             self.client.get_goal_status_text())
        finally:
            if not done.is_set():
                self.client.cancel_goal()
            subscription.unregister()

        self.assertTrue(seen_motion.is_set(), 'Wrong or missing motor motion')
        self.assertGreater(self.client.get_result().time_elapsed.data.to_sec(),
                           0)

        # Read the mode directly from the simulated drive after each stop
        self.assertEqual(self.register(2), Motion.MODE_PASSIVE)
        encoder = self.register(10)
        ctd = rospy.wait_for_message('ctd/ctd', Ctd, timeout=5)
        depth = rospy.wait_for_message('ctd/depth', DepthPressure, timeout=5)
        measured = self.probe_depth()
        self.assertAlmostEqual(measured, target, delta=0.08)
        self.assertAlmostEqual(depth.depth, measured, delta=0.04)
        self.assertAlmostEqual(depth.pressure, 1025 * 9.80665 * depth.depth /
                               10000, places=4)
        self.assertEqual(depth.pressure_raw_unit,
                         DepthPressure.UNIT_PRESSURE_DBAR)
        self.assertAlmostEqual(ctd.pressure, depth.pressure, delta=0.04)
        self.assertGreater(ctd.header.stamp.to_sec(), 0)
        self.assertGreater(ctd.conductivity, 0)
        self.assertTrue(-2 < ctd.temperature < 40)
        self.assertTrue(20 < ctd.salinity < 42)
        rospy.loginfo('Target %.2f m, Gazebo %.3f m, CTD %.3f m, %.3f dbar, '
                      'encoder %d', target, measured, depth.depth,
                      depth.pressure, encoder)
        return measured, encoder

    def test_gps_fix(self):
        model = rospy.wait_for_message('sim/gps', NavSatFix, timeout=15)

        # Wait for gpsd to acquire a position and altitude from the receiver
        for _ in range(8):
            fix = rospy.wait_for_message('gps/fix', NavSatFix, timeout=10)
            if (fix.status.status >= NavSatStatus.STATUS_FIX and
                    all(map(math.isfinite, (fix.latitude, fix.altitude)))):
                break
        self.assertGreaterEqual(fix.status.status, NavSatStatus.STATUS_FIX)
        self.assertAlmostEqual(fix.latitude, model.latitude, places=5)
        self.assertAlmostEqual(fix.longitude, model.longitude, places=5)
        self.assertAlmostEqual(fix.altitude, model.altitude, places=2)
        extended = rospy.wait_for_message('gps/extended_fix', GPSFix, timeout=5)
        self.assertAlmostEqual(extended.latitude, model.latitude, places=5)
        self.assertAlmostEqual(extended.longitude, model.longitude, places=5)

    def test_outlet_control(self):
        base = 'digital_logger/outlet/starlink/'
        control = rospy.Publisher(base + 'control', Bool, queue_size=1,
                                  latch=True)
        self.addCleanup(control.unregister)

        # Switch one outlet off and back on through the production driver
        for active in (False, True):
            seen = threading.Event()

            def on_status(msg):
                if msg.is_active == active:
                    seen.set()

            sub = rospy.Subscriber(base + 'status', OutletStatus,
                                   on_status)
            try:
                control.publish(active)
                self.assertTrue(seen.wait(10), 'Outlet did not switch')
            finally:
                sub.unregister()

    def test_round_trip(self):
        initial = rospy.wait_for_message('ctd/depth', DepthPressure, timeout=15)
        initial_count = self.register(10)
        deep, deep_count = self.cast(2.2, 1)

        # Changing the encoder reference must leave the drum and payload still
        paid_out = 1.32 + deep_count * 0.6604 / (8192 * 60)
        rospy.ServiceProxy('motor/zero_position', ZeroCmd)()
        self.assertEqual(self.register(10), 0)
        lengths = []
        updated = threading.Event()

        # Wait past the latched value for fresh shaft observations
        def on_length(msg):
            lengths.append(msg.data)
            if len(lengths) >= 3:
                updated.set()

        sub = rospy.Subscriber('sim/line_length', Float64, on_length)
        try:
            self.assertTrue(updated.wait(5))
            self.assertAlmostEqual(lengths[-1], paid_out, places=5)
        finally:
            sub.unregister()

        shallow, shallow_count = self.cast(0.85, -1)
        self.assertGreater(deep - initial.depth, 1.0)
        self.assertGreater(deep - shallow, 1.0)
        self.assertGreater(deep_count, initial_count)
        self.assertLess(shallow_count, 0)


if __name__ == '__main__':
    rospy.init_node('gazebo_winch_test')
    rostest.rosrun('phyto_gazebo', 'gazebo_winch', TestGazeboWinch)
