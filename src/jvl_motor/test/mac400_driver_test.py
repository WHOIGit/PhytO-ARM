#!/usr/bin/env python3
import math
import struct
import time
import unittest

import rclpy

from functools import partial
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from jvl_motor_msgs.msg import Electrical, Error, Motion
from jvl_motor_msgs.srv import (
    SetPositionCmd, SetPositionEnvelopeCmd, SetVelocityCmd, StopCmd, ZeroCmd,
)
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import SetParameter
from launch_testing.actions import ReadyToTest
from pyModbusTCP.client import ModbusClient
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time
from rosgraph_msgs.msg import Clock


DEFAULT_MOTOR_PORT = '15502'


def generate_test_description():
    launch = Path(get_package_share_directory('jvl_motor')) / 'launch' / 'sim_motor.launch.py'
    return LaunchDescription([
        DeclareLaunchArgument('motor_modbus_port', default_value=DEFAULT_MOTOR_PORT),
        SetParameter(name='use_sim_time', value=True),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(launch)), launch_arguments={
            'motor_modbus_port': LaunchConfiguration('motor_modbus_port'),
            'refresh_rate': '20',
        }.items()),
        ReadyToTest(),
    ])


# Wire expectations from the JVL MACmotor Register List
ERR_STAT = 35
MODE_REG = 2
P_IST = 10
V_IST_16 = 11

VELOCITY_UNITS_PER_RPM = 2.83989


class RealDriverTest(unittest.TestCase):
    def setUp(self, test_args=None):
        rclpy.init()
        self.addCleanup(rclpy.shutdown)
        self.node = rclpy.create_node('mac400_driver_test')
        self.addCleanup(self.node.destroy_node)
        self.electrical = None
        self.error = None
        self.motion = None
        self.time_ms = 1000

        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.clock = self.node.create_publisher(Clock, '/clock', qos)
        for field, message in [('electrical', Electrical), ('error', Error), ('motion', Motion)]:
            self.node.create_subscription(message, '/motor/' + field,
                                          partial(setattr, self, field), 10)

        # Read the simulator registers directly to check the driver's feedback
        port = int((test_args or {}).get('motor_modbus_port', DEFAULT_MOTOR_PORT))
        self.client = ModbusClient(
            host='127.0.0.1', port=port,
            auto_open=True, timeout=1.0,
        )
        self.addCleanup(self.client.close)

        self.envelope = self.service(SetPositionEnvelopeCmd, '/motor/set_position_envelope')
        self.stop = self.service(StopCmd, '/motor/stop')
        self.target = self.service(SetPositionCmd, '/motor/set_position')
        self.velocity = self.service(SetVelocityCmd, '/motor/set_velocity')
        self.zero = self.service(ZeroCmd, '/motor/zero_position')

    def service(self, service_type, name):
        client = self.node.create_client(service_type, name)
        self.assertTrue(client.wait_for_service(timeout_sec=15), name)

        def call(**fields):
            future = client.call_async(service_type.Request(**fields))
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=5)
            self.assertTrue(future.done(), name)
            response = future.result()
            self.assertTrue(response.success, response.message)
            return response

        return call

    def set_clock(self, milliseconds):
        self.time_ms = milliseconds
        seconds, remainder_ms = divmod(milliseconds, 1000)
        stamp = Time(seconds=seconds, nanoseconds=remainder_ms * 10**6).to_msg()
        self.clock.publish(Clock(clock=stamp))
        return stamp

    # Step ROS time until the driver publishes matching feedback for that tick
    def advance_until_motion(self, predicate):
        deadline = time.monotonic() + 10
        while time.monotonic() < deadline:
            stamp = self.set_clock(self.time_ms + 100)
            tick_deadline = time.monotonic() + 0.25
            while time.monotonic() < tick_deadline:
                rclpy.spin_once(self.node, timeout_sec=max(0, tick_deadline - time.monotonic()))
                if (self.motion is not None and self.motion.header.stamp == stamp and
                        predicate(self.motion)):
                    return self.motion
        self.fail('No matching current motor feedback. Last: %r' % self.motion)

    def read_register(self, register):
        # Signed 32-bit value, low word first on the MAC400 bus
        words = self.client.read_holding_registers(2 * register, 2)
        self.assertIsNotNone(words, 'Failed to read register %d' % register)
        return struct.unpack('<i', struct.pack('<HH', *words))[0]

    def wait_for_register(self, register, predicate):
        # The simulator can receive /clock after the driver. Observe its own
        # state over TCP without advancing the paused clock any further.
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline:
            value = self.read_register(register)
            if predicate(value):
                return value
            time.sleep(0.01)
        self.fail('Register %d did not reach the expected value' % register)

    def check_electrical_and_velocity_feedback(self):
        self.advance_until_motion(lambda m: m.mode == Motion.MODE_PASSIVE and
                                  self.electrical is not None)

        # Nominal 24 V on the 0–55 V, 12-bit scale
        self.assertEqual(self.electrical.u_24v, 1787)
        self.assertGreater(self.electrical.u_bus, 0)

        # At 8.2 RPM a float round trip can truncate register value 23 to 22
        self.zero()
        self.velocity(velocity=8.2, acceleration=1200.0, torque=1.0)
        expected_rpm = 23 / VELOCITY_UNITS_PER_RPM
        self.advance_until_motion(
            lambda m: m.mode == Motion.MODE_VELOCITY and m.position > 0 and
            math.isclose(m.velocity, expected_rpm, rel_tol=1e-6)
        )
        self.assertEqual(self.read_register(V_IST_16), 23)

    def check_motion_in_ros_time(self):
        # A switch from velocity to passive confirms the drive saw the rewind
        self.set_clock(250)
        self.wait_for_register(
            MODE_REG, lambda mode: mode == Motion.MODE_PASSIVE)
        self.zero()
        self.assertEqual(self.read_register(P_IST), 0)

        # Give the motor one simulated second to reach the requested speed
        self.velocity(velocity=60.0, acceleration=1200.0, torque=1.0)
        self.set_clock(1250)
        start = self.wait_for_register(P_IST, lambda position: position > 0)

        # Wall time passes while the simulated encoder stays still
        time.sleep(0.15)
        self.assertEqual(self.read_register(P_IST), start)

        # Measure one second at speed. 60 RPM quantizes to 170 register units,
        # and one shaft revolution produces 8192 encoder counts.
        self.set_clock(2250)
        end = self.wait_for_register(P_IST, lambda position: position > start)
        expected_delta = (170 / VELOCITY_UNITS_PER_RPM) * 8192 / 60
        self.assertAlmostEqual(end - start, expected_delta, delta=1)

        # Rewinding stops the motor at its last encoder position
        self.set_clock(100)
        self.wait_for_register(
            MODE_REG, lambda mode: mode == Motion.MODE_PASSIVE)
        self.advance_until_motion(lambda m: m.mode == Motion.MODE_PASSIVE and
                                  m.position == end and m.velocity == 0)

    def check_position_envelope(self):
        # The motor may reach either end of the position envelope
        self.zero()
        self.advance_until_motion(lambda m: m.position == 0)
        self.envelope(min=-1000, max=1000)
        for target in (1000, -1000):
            self.target(position=target, velocity=60.0, acceleration=1200.0, torque=1.0)
            self.advance_until_motion(
                lambda m: m.mode == Motion.MODE_POSITION and
                m.position == target and m.velocity == 0
            )

        # Disabling the envelope permits travel beyond the previous limits
        self.envelope(min=0, max=0)
        self.target(position=-3000, velocity=60.0, acceleration=1200.0, torque=1.0)
        self.advance_until_motion(lambda m: m.mode == Motion.MODE_POSITION and
                                  m.position == -3000 and m.velocity == 0)
        self.stop()
        self.advance_until_motion(lambda m: m.mode == Motion.MODE_PASSIVE)

    def check_latched_fault(self):
        # Crossing the lower limit sets bit 7 (PLIM_ERR) in ERR_STAT
        self.envelope(min=-4000, max=0)
        self.velocity(velocity=-60.0, acceleration=1200.0, torque=1.0)
        limit_error = 1 << 7
        tripped = self.advance_until_motion(
            lambda m: m.mode == Motion.MODE_PASSIVE and m.position < -4000 and
            self.error is not None and self.error.err_stat & limit_error
        )

        # Loosening the envelope and requesting inward motion cannot clear
        # the latched drive fault. Require fresh telemetry after the retry.
        self.envelope(min=0, max=0)
        self.velocity(velocity=60.0, acceleration=1200.0, torque=1.0)
        self.advance_until_motion(
            lambda m: m.mode == Motion.MODE_PASSIVE and
            m.position == tripped.position and m.velocity == 0
        )
        self.assertTrue(self.read_register(ERR_STAT) & limit_error)

    def test_driver_commands_and_feedback(self):
        self.check_electrical_and_velocity_feedback()
        self.check_motion_in_ros_time()
        self.check_position_envelope()

        # A limit fault stays latched for the lifetime of this simulator
        self.check_latched_fault()


if __name__ == '__main__':
    unittest.main()
