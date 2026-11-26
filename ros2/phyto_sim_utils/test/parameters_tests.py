import unittest

import rclpy

from phyto_sim_utils.parameters import declare_number
from rclpy.exceptions import InvalidParameterTypeException
from rclpy.parameter import Parameter


class TestParameterDeclarations(unittest.TestCase):
    def setUp(self):
        rclpy.init()
        self.addCleanup(rclpy.try_shutdown)

    def node_with(self, **values):
        node = rclpy.create_node('parameter_test', parameter_overrides=[
            Parameter(name, value=value) for name, value in values.items()
        ])
        self.addCleanup(node.destroy_node)
        return node

    def test_numeric_defaults_and_overrides(self):
        node = self.node_with(gear_ratio=60, depth=1.5, invalid='60')
        self.assertEqual(declare_number(node, 'gear_ratio', 60.0), 60)
        self.assertEqual(declare_number(node, 'depth'), 1.5)
        self.assertEqual(declare_number(node, 'height', 0.5), 0.5)
        self.assertIsNone(declare_number(node, 'optional'))
        with self.assertRaises(InvalidParameterTypeException):
            declare_number(node, 'invalid')
