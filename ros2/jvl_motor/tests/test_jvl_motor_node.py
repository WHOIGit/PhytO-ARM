import unittest
from unittest.mock import MagicMock

import rospy
from jvl_motor.srv import SetPositionEnvelopeCmdRequest, \
    SetVelocityCmdRequest, ZeroCmdRequest

from jvl_motor.jvl_motor_node import cmd_set_position_envelope, cmd_set_velocity, \
    cmd_zero_position


class TestMotorWriteErrors(unittest.TestCase):
    def test_zero_position_write_fails(self):
        client = MagicMock()
        client.write_multiple_registers.return_value = False
        with self.assertRaises(rospy.ServiceException):
            cmd_zero_position(client, ZeroCmdRequest())

    def test_velocity_write_fails(self):
        client = MagicMock()
        client.write_multiple_registers.return_value = False
        with self.assertRaises(rospy.ServiceException):
            cmd_set_velocity(client, SetVelocityCmdRequest(10, 1200, 3))

    def test_either_envelope_write_fails(self):
        for replies in ([False], [True, False]):
            with self.subTest(replies=replies):
                client = MagicMock()
                client.write_multiple_registers.side_effect = replies
                with self.assertRaises(rospy.ServiceException):
                    cmd_set_position_envelope(
                        client, SetPositionEnvelopeCmdRequest(-100, 100))
