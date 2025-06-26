#!/usr/bin/env python3
import datetime
import importlib.util
import os
import re
import sys
import unittest
import unittest.mock

sys.path.append(os.path.join(os.path.dirname(__file__), os.path.pardir, 'src'))


# Hack: Allow running tests without a ROS environment by stubbing out the
# modules that would normally be available.
stub_modules = (
    'ds_core_msgs',
    'ds_core_msgs.msg',
    'rospy',
    'std_msgs',
    'std_msgs.msg',
)
for module in stub_modules:
    if importlib.util.find_spec(module) is None:
        sys.modules[module] = unittest.mock.Mock()
        sys.modules[module].__path__ = ''  # type: ignore


from io_node import (
    FieldSpec,
    RawFramer,
    JsonFramer,
    DelimitedExtractor,
    JsonExtractor,
    Extractor,
)


class TestFieldSpec(unittest.TestCase):
    def test_simple_field(self):
        spec = FieldSpec(name='f', type='int', selector='field')
        self.assertEqual(spec.path, ['field'])

    def test_bracket_and_dot_syntax(self):
        cases = {
            '[a]': ['a'],
            '.a.b': ['a', 'b'],
            'x[0]': ['x', 0],
            'y.1': ['y', 1],
            '$.z': ['z'],
        }
        for sel, expected in cases.items():
            spec = FieldSpec(name='f', type='str', selector=sel)
            self.assertEqual(spec.path, expected)

    def test_invalid_selector(self):
        with self.assertRaises(ValueError):
            FieldSpec(name='f', type='str', selector='.a.b]')


class TestRawFramer(unittest.TestCase):
    def test_empty(self):
        fr = RawFramer()
        self.assertEqual(fr.frame(b''), [])

    def test_non_empty(self):
        fr = RawFramer()
        out = fr.frame(b'data')
        self.assertEqual(len(out), 1)
        ts, payload = out[0]
        self.assertIsInstance(ts, datetime.datetime)
        self.assertEqual(payload, b'data')


class TestJsonFramer(unittest.TestCase):
    def test_complete_json(self):
        fr = JsonFramer()
        out = fr.frame(b'{"a":1}')
        self.assertEqual(len(out), 1)
        _, payload = out[0]
        self.assertEqual(payload, b'{"a":1}')

    def test_incremental_json(self):
        fr = JsonFramer()
        self.assertEqual(fr.frame(b'{"a":1'), [])
        out = fr.frame(b'}{"b":2}')
        self.assertEqual(len(out), 2)
        _, first = out[0]
        self.assertEqual(first, b'{"a":1}')
        out2 = fr.frame(b'')
        self.assertEqual(out2, [])


class TestExtractor(unittest.TestCase):
    def test_base(self):
        self.assertEqual(Extractor().extract(b'anything'), {})

    def test_delimited(self):
        pattern = re.compile(r',')
        fields = [FieldSpec(name='one', type='str', selector='[0]'),
                  FieldSpec(name='three', type='str', selector='[2]')]
        ex = DelimitedExtractor(pattern, fields)
        result = ex.extract(b'a,b,c,d')
        self.assertEqual(result, {'one': 'a', 'three': 'c'})

    def test_json_extractor(self):
        data = b'{"a":{"b":[10,20,30]}}'
        spec = FieldSpec(name='x', type='int', selector='a.b.2')
        ex = JsonExtractor([spec])
        self.assertEqual(ex.extract(data), {'x': 30})


if __name__ == '__main__':
    unittest.main()