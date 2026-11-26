import unittest

from unittest.mock import Mock

from jvl_motor import mac400
from jvl_motor.mac400_model import MotorModel
from jvl_motor.mac400_sim_node import MotorRegisters
from rclpy.time import Time
from rosgraph_msgs.msg import Clock


class TestMotorRegisters(unittest.TestCase):
    def setUp(self):
        self.node = Mock()
        self.now = self.node.get_clock.return_value.now
        self.now.return_value = Time(seconds=1)

        self.motor = MotorModel()
        self.registers = MotorRegisters(self.node, self.motor)

    def test_clock_advances_while_subscribing(self):
        # A callback can fire during subscription, before the initial snapshot
        def subscribe(kind, topic, callback, qos):
            callback(Clock(clock=Time(seconds=99).to_msg()))
            self.now.return_value = Time(seconds=100)

        self.node.create_subscription.side_effect = subscribe
        registers = MotorRegisters(self.node, self.motor, use_sim_time=True)
        on_clock = self.node.create_subscription.call_args[0][2]

        # Requests must use the last clock tick even if the node knows a later time
        self.motor.write({
            mac400.A_SOLL: 0,
            mac400.MODE_REG: mac400.MODE.VELOCITY,
            mac400.V_SOLL: 60,
        })
        self.now.return_value = Time(seconds=110)
        self.assertEqual(registers.read_words(20, 2), [0, 0])
        on_clock(Clock(clock=Time(seconds=101).to_msg()))
        self.assertEqual(registers.read_words(20, 2), [8192, 0])

    def test_wall_time_advances_at_reads_and_writes(self):
        # At 60 counts per turn the speed in RPM is also in counts per second
        registers = MotorRegisters(self.node, MotorModel(counts_per_turn=60))
        registers.write_words(10, mac400.V_SOLL.encode(60))
        registers.write_words(4, [mac400.MODE.VELOCITY, 0])
        self.now.return_value = Time(seconds=2)
        self.assertEqual(registers.read_words(20, 2), [60, 0])

        # Stopping after two more seconds must include that unobserved travel
        self.now.return_value = Time(seconds=4)
        registers.write_words(4, [mac400.MODE.PASSIVE, 0])
        self.now.return_value = Time(seconds=6)
        self.assertEqual(registers.read_words(20, 2), [180, 0])

    def test_read_windows_and_unused_words(self):
        # An unaligned read spans the supply reading and an unused word
        self.assertEqual(self.registers.read_words(41, 4), [0, 1787, 0, 0])
        for first, last in (mac400.ERR_STAT.addr, mac400.U_BUS.addr):
            with self.subTest(address=last):
                words = self.registers.read_words(first, 2)
                self.assertEqual(self.registers.read_words(last, 1), words[1:])
                with self.assertRaises(KeyError):
                    self.registers.read_words(last, 2)

    def test_setpoints_echo_and_velocity_feedback(self):
        # These units lose a count on a float decode followed by truncation
        for second, speed in enumerate((23, -23), 1):
            with self.subTest(speed=speed):
                # Write MODE, P_SOLL, P_NEW, V_SOLL, A_SOLL and T_SOLL in
                # consecutive register pairs.
                lo, hi = mac400.pack('l', speed)
                block = [1, 0, 0, 0, 0, 0, lo, hi, 4, 0, 29, 0]
                self.registers.write_words(4, block)
                self.assertEqual(self.registers.read_words(4, 12), block)
                self.now.return_value = Time(seconds=1 + second)
                self.assertEqual(self.registers.read_words(22, 2), [lo, hi])

        # Rewinding stops the motor and retains the written setpoint
        self.now.return_value = Time(seconds=1)
        self.assertEqual(self.registers.read_words(4, 2), [0, 0])
        self.assertEqual(self.registers.read_words(22, 2), [0, 0])
        self.assertEqual(self.registers.read_words(10, 2), [lo, hi])

    def test_initial_torque_matches_model(self):
        self.assertEqual(self.motor.read(mac400.T_SOLL), 3.0)
        self.assertEqual(self.registers.read_words(14, 2), [1023, 0])

    def test_setpoint_write_commits_atomically(self):
        # The second pair crosses into an unsupported register after T_SOLL
        with self.assertRaises(KeyError):
            self.registers.write_words(14, [29, 0, 1, 0])
        self.assertEqual(self.motor.read(mac400.T_SOLL), 3.0)
        self.assertEqual(self.registers.read_words(14, 2), [1023, 0])
