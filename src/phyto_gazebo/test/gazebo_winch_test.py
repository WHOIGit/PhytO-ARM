#!/usr/bin/env python3
import math
import struct
import threading

from pathlib import Path

from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from aml_ctd_msgs.msg import AmlMeasurement
from dli_power_switch_msgs.msg import OutletStatus
from ds_sensor_msgs.msg import Ctd, DepthPressure
from geometry_msgs.msg import Pose, Vector3Stamped
from gps_msgs.msg import GPSFix
from jvl_motor_msgs.msg import Motion
from jvl_motor_msgs.srv import ZeroCmd
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_testing.actions import ReadyToTest
from phyto_arm_msgs.action import MoveToDepth
from pyModbusTCP.client import ModbusClient
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time
from ros_test import RosTest
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import Bool, Float64


DEFAULT_MOTOR_PORT = '16542'


def generate_test_description():
    launch = Path(get_package_share_directory('phyto_arm')) / 'launch' / 'simulator.launch.py'

    return LaunchDescription([
        DeclareLaunchArgument('gpsd', default_value='true'),
        DeclareLaunchArgument('gpsd_port', default_value='16544'),
        DeclareLaunchArgument('motor_modbus_port', default_value=DEFAULT_MOTOR_PORT),
        SetEnvironmentVariable('IGN_PARTITION', ['winch_test_', LaunchConfiguration('motor_modbus_port')]),
        Node(package='ros_gz_bridge', executable='parameter_bridge',
             arguments=['/model/ifcb/pose@geometry_msgs/msg/Pose[ignition.msgs.Pose']),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(launch)), launch_arguments={
            'ctd': 'aml',
            'dli_port': '16545',
            'gazebo': 'true',
            'gpsd': LaunchConfiguration('gpsd'),
            'gpsd_port': LaunchConfiguration('gpsd_port'),
            'gui': 'false',
            'motor_modbus_port': LaunchConfiguration('motor_modbus_port'),
        }.items()),
        ReadyToTest(),
    ])


class TestGazeboWinch(RosTest):
    def setUp(self, test_args):
        super().setUp()
        self.client = ActionClient(self.node, MoveToDepth, 'winch/move_to_depth')
        self.addCleanup(self.client.destroy)
        self.assertTrue(self.client.wait_for_server(timeout_sec=40))
        port = int(test_args.get('motor_modbus_port', DEFAULT_MOTOR_PORT))
        self.bus = ModbusClient(host='127.0.0.1', port=port, timeout=2, auto_open=True)
        self.addCleanup(self.bus.close)

    def register(self, number):
        words = self.bus.read_holding_registers(2 * number, 2)
        self.assertIsNotNone(words, f'Failed to read MAC400 register {number}')
        return struct.unpack('<i', struct.pack('<HH', *words))[0]

    def probe_depth(self):
        pose = self.wait_for_message('/model/ifcb/pose', Pose, timeout=5)
        q = pose.orientation
        rotation = Rotation.from_quat((q.x, q.y, q.z, q.w))
        offset = rotation.apply((0.185, 0, -0.1))[2]
        return -(pose.position.z + offset)

    def cast(self, target, direction):
        seen_motion = threading.Event()

        def check_motion(msg):
            if (msg.mode == Motion.MODE_VELOCITY and
                    direction * msg.velocity > 0):
                seen_motion.set()

        subscription = self.node.create_subscription(Motion, 'motor/motion', check_motion, 10)
        try:
            request = MoveToDepth.Goal(depth=target, velocity=0.18)
            goal = self.result(self.client.send_goal_async(request), 10)
            self.assertTrue(goal.accepted)
            pending_result = goal.get_result_async()
            try:
                self.assertTrue(self.wait_until(pending_result.done, 85),
                                f'Timed out moving to {target} m')
                result = pending_result.result()
                self.assertEqual(result.status, GoalStatus.STATUS_SUCCEEDED)
            finally:
                if not pending_result.done():
                    self.result(goal.cancel_goal_async(), 5)
        finally:
            self.node.destroy_subscription(subscription)

        self.assertTrue(seen_motion.is_set(), 'Wrong or missing motor motion')
        self.assertGreater(Duration.from_msg(result.result.time_elapsed).nanoseconds, 0)

        # Read the mode directly from the simulated drive after each stop
        self.assertEqual(self.register(2), Motion.MODE_PASSIVE)
        encoder = self.register(10)
        ctd = self.wait_for_message('ctd/ctd', Ctd, timeout=5)
        depth = self.wait_for_message('ctd/depth', DepthPressure, timeout=5)
        measured = self.probe_depth()
        self.assertAlmostEqual(measured, target, delta=0.08)
        self.assertAlmostEqual(depth.depth, measured, delta=0.04)
        self.assertAlmostEqual(depth.pressure, 1025 * 9.80665 * depth.depth /
                               10000, places=4)
        self.assertEqual(depth.pressure_raw_unit,
                         DepthPressure.UNIT_PRESSURE_DBAR)
        self.assertAlmostEqual(ctd.pressure, depth.pressure, delta=0.04)
        self.assertGreater(Time.from_msg(ctd.header.stamp).nanoseconds, 0)
        self.assertGreater(ctd.conductivity, 0)
        self.assertTrue(-2 < ctd.temperature < 40)
        salinity = self.wait_for_message('ctd/aml/derive/sal_c',
                                          AmlMeasurement, timeout=5)
        self.assertTrue(20 < salinity.value < 42)
        self.node.get_logger().info(f'Target {target:.2f} m, Gazebo {measured:.3f} m, '
                                    f'CTD {depth.depth:.3f} m, {depth.pressure:.3f} dbar, '
                                    f'encoder {encoder}')
        return measured, encoder

    def test_gps_fix(self):
        model = self.wait_for_message('sim/gps', NavSatFix, timeout=15)

        velocity = self.wait_for_message('sim/gps_velocity', Vector3Stamped, timeout=5)
        self.assertEqual(velocity.header.frame_id, model.header.frame_id)
        self.assertEqual((velocity.vector.x, velocity.vector.y, velocity.vector.z), (0, 0, 0))

        # Wait for gpsd to acquire a position and altitude from the receiver
        for _ in range(8):
            fix = self.wait_for_message('gps/fix', NavSatFix, timeout=10)
            if (fix.status.status >= NavSatStatus.STATUS_FIX and
                    all(map(math.isfinite, (fix.latitude, fix.altitude)))):
                break
        self.assertGreaterEqual(fix.status.status, NavSatStatus.STATUS_FIX)
        self.assertAlmostEqual(fix.latitude, model.latitude, places=5)
        self.assertAlmostEqual(fix.longitude, model.longitude, places=5)
        self.assertAlmostEqual(fix.altitude, model.altitude, places=2)
        extended = self.wait_for_message('gps/extended_fix', GPSFix, timeout=5)
        self.assertAlmostEqual(extended.latitude, model.latitude, places=5)
        self.assertAlmostEqual(extended.longitude, model.longitude, places=5)

    def test_outlet_control(self):
        base = 'digital_logger/outlet/starlink/'
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        control = self.node.create_publisher(Bool, base + 'control', qos)

        # Switch one outlet off and back on through the production driver
        for active in (False, True):
            seen = threading.Event()

            def on_status(msg):
                if msg.is_active == active:
                    seen.set()

            sub = self.node.create_subscription(OutletStatus, base + 'status', on_status, 10)
            try:
                self.assertTrue(self.wait_until(control.get_subscription_count, 5))
                control.publish(Bool(data=active))
                self.assertTrue(self.wait_until(seen.is_set, 10), 'Outlet did not switch')
            finally:
                self.node.destroy_subscription(sub)

    def test_round_trip(self):
        initial = self.wait_for_message('ctd/depth', DepthPressure, timeout=15)
        initial_count = self.register(10)
        deep, deep_count = self.cast(2.2, 1)

        # Changing the encoder reference must leave the drum and payload still
        paid_out = 1.32 + deep_count * 0.6604 / (8192 * 60)
        zero = self.node.create_client(ZeroCmd, 'motor/zero_position')
        self.call(zero, ZeroCmd.Request())
        self.assertEqual(self.register(10), 0)
        lengths = []
        updated = threading.Event()

        # Wait past the latched value for fresh shaft observations
        def on_length(msg):
            lengths.append(msg.data)
            if len(lengths) >= 3:
                updated.set()

        sub = self.node.create_subscription(Float64, 'sim/line_length', on_length, 10)
        try:
            self.assertTrue(self.wait_until(updated.is_set, 5))
            self.assertAlmostEqual(lengths[-1], paid_out, places=5)
        finally:
            self.node.destroy_subscription(sub)

        shallow, shallow_count = self.cast(0.85, -1)
        self.assertGreater(deep - initial.depth, 1.0)
        self.assertGreater(deep - shallow, 1.0)
        self.assertGreater(deep_count, initial_count)
        self.assertLess(shallow_count, 0)
