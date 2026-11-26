import unittest

import rclpy

from phyto_sim_utils.parameters import declare_matching_parameters, declare_number
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

    def test_empty_web_default(self):
        node = self.node_with(**{'field_map.sample.default': None})
        declare_matching_parameters(node, r'field_map\.[^.]+\.default')
        self.assertEqual(node.get_parameter('field_map.sample.default').value, [])

    def test_numeric_defaults_and_overrides(self):
        node = self.node_with(gear_ratio=60, depth=1.5, invalid='60')
        self.assertEqual(declare_number(node, 'gear_ratio', 60.0), 60)
        self.assertEqual(declare_number(node, 'depth'), 1.5)
        self.assertEqual(declare_number(node, 'height', 0.5), 0.5)
        self.assertIsNone(declare_number(node, 'optional'))
        with self.assertRaises(InvalidParameterTypeException):
            declare_number(node, 'invalid')

    def test_deployment_names(self):
        node = self.node_with(**{'topics.deck.port': 4321, 'topics.deck.typo': 2,
                                'other.port': 9876})
        declare_matching_parameters(node, r'topics\.[^.]+\.port', Parameter.Type.INTEGER)
        self.assertEqual(node.get_parameter('topics.deck.port').value, 4321)
        self.assertFalse(node.has_parameter('topics.deck.typo'))
        self.assertFalse(node.has_parameter('other.port'))
