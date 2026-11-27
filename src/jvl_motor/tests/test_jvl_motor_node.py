import unittest

from unittest.mock import MagicMock

from jvl_motor.jvl_motor_node import (
    cmd_set_position, cmd_set_position_envelope, cmd_set_velocity, cmd_zero_position,
)
from jvl_motor_msgs.srv import SetPositionCmd, SetPositionEnvelopeCmd, SetVelocityCmd, ZeroCmd


class TestMotorWriteErrors(unittest.TestCase):
    def test_zero_position_write_fails(self):
        client = MagicMock()
        client.write_multiple_registers.return_value = False
        with self.assertRaises(RuntimeError):
            cmd_zero_position(client, ZeroCmd.Request())

    def test_position_write_fails(self):
        client = MagicMock()
        client.write_multiple_registers.return_value = False
        with self.assertRaises(RuntimeError):
            cmd_set_position(client, SetPositionCmd.Request())

    def test_velocity_write_fails(self):
        client = MagicMock()
        client.write_multiple_registers.return_value = False
        with self.assertRaises(RuntimeError):
            cmd_set_velocity(client, SetVelocityCmd.Request(velocity=10.0, acceleration=1200.0, torque=3.0))

    def test_either_envelope_write_fails(self):
        for replies in ([False], [True, False]):
            with self.subTest(replies=replies):
                client = MagicMock()
                client.write_multiple_registers.side_effect = replies
                with self.assertRaises(RuntimeError):
                    cmd_set_position_envelope(
                        client, SetPositionEnvelopeCmd.Request(min=-100, max=100))
