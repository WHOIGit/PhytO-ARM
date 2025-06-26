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
    DelimitedExtractor,
    DelimitedFramer,
    FieldSpec,
    JsonExtractor,
    JsonFramer,
    RawFramer,
)


class TestFieldSpec(unittest.TestCase):
    def test_selector_syntax(self):
        cases = {
          '[field]': ['field'],  # bracket syntax
          '.field': ['field'],  # dot syntax
          '.field.subfield': ['field', 'subfield'],  # nested field
          'field[0]': ['field', 0],  # numeric index bracket syntax
          'field.0': ['field', 0],  # numeric index dot syntax
          'field': ['field'],  # leading dot is optional
          '$.field': ['field'],  # leading $ (from JSONPath) is optional
        }
        for sel, expected in cases.items():
            spec = FieldSpec(name='f', type='str', selector=sel)
            self.assertEqual(spec.path, expected)

    @unittest.expectedFailure
    def test_future_selector_syntax(self):
        cases = {
            '.escaped\\.char': ['escaped.char'],  # escaped characters
            '."field with spaces"': ['field with spaces'],  # quoted field
            '.field[\'0\']': ['field', '0'],  # numeric string
        }
        for sel, expected in cases.items():
            spec = FieldSpec(name='f', type='str', selector=sel)
            self.assertEqual(spec.path, expected)

    def test_invalid_selector(self):
        with self.assertRaises(ValueError):
            FieldSpec(name='f', type='str', selector='.a.b]')


class TestRawFramer(unittest.TestCase):
    def test_empty(self):
        f = RawFramer()
        self.assertEqual(f.frame(b''), [])

    def test_non_empty(self):
        f = RawFramer()
        frames = f.frame(b'data')
        self.assertEqual(len(frames), 1)
        ts, payload = frames[0]
        self.assertIsInstance(ts, datetime.datetime)
        self.assertEqual(payload, b'data')


class TestDelimitedFramer(unittest.TestCase):
    def test_empty(self):
        f = DelimitedFramer(pattern=re.compile(rb'\n'))
        self.assertEqual(f.frame(b''), [])

    def test_non_empty(self):
        f = DelimitedFramer(pattern=re.compile(rb'\n'))
        frames = f.frame(b'data')
        self.assertEqual(len(frames), 0)  # no newline yet
        frames = f.frame(b'\n')
        self.assertEqual(len(frames), 1)
        ts, payload = frames[0]
        self.assertIsInstance(ts, datetime.datetime)
        self.assertEqual(payload, b'data\n')
        frames = f.frame(b'data\nmore data\nyet more')
        self.assertEqual(len(frames), 2)
        (_, first), (_, second) = frames
        self.assertEqual(first, b'data\n')
        self.assertEqual(second, b'more data\n')
        self.assertEqual(f.buffer, b'yet more')


class TestJsonFramer(unittest.TestCase):
    def test_complete_frame(self):
        f = JsonFramer()
        frames = f.frame(b'{"a":1}')
        self.assertEqual(len(frames), 1)
        ts, payload = frames[0]
        self.assertIsInstance(ts, datetime.datetime)
        self.assertEqual(payload, b'{"a":1}')

    def test_partial_frames(self):
        f = JsonFramer()
        frames = f.frame(b'{"a":')
        self.assertEqual(len(frames), 0)
        frames = f.frame(b'1}{"b":2}{')
        self.assertEqual(len(frames), 2)
        (_, first), (_, second) = frames
        self.assertEqual(first, b'{"a":1}')
        self.assertEqual(second, b'{"b":2}')
        self.assertEqual(f.buffer, b'{')


class TestDelimitedExtractor(unittest.TestCase):
    def test_delimited(self):
        ex = DelimitedExtractor(
            re.compile(rb','), 
            [
                FieldSpec(name='one', type='str', selector='[0]'),
                FieldSpec(name='three', type='int', selector='[2]')
            ]
        )
        result = ex.extract(b'a,b,42,d')
        self.assertEqual(result, {'one': 'a', 'three': 42})


class TestJsonExtractor(unittest.TestCase):
    def test_json_extractor(self):
        ex = JsonExtractor([
            FieldSpec(name='x', type='int', selector='a.b.2')
        ])
        data = b'{"a":{"b":[10,20,30]}}'
        self.assertEqual(ex.extract(data), {'x': 30})


if __name__ == '__main__':
    unittest.main()