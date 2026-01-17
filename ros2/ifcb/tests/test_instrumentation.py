import json
import os
import sys
import unittest

# Allow importing from the package directory
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

from ifcb.instrumentation import (
    instrument_routine,
    instrument_stepgroup,
    instrument_step,
    marker,
    parse_marker,
    MAGIC,
)


def get_payload(report_step):
    assert report_step.get('StepType') == 'Report'
    args = report_step.get('Arguments', [])
    assert len(args) == 1
    return parse_marker(args[0])


class TestParseMarker(unittest.TestCase):
    def test_create_and_parse(self):
        kind, routine, path, value = 'enter', 'routine', [1, 2], {'foo': 'bar'}
        report_step = marker(kind, routine, path, value)
        encoded = report_step['Arguments'][0]
        self.assertTrue(encoded.startswith(MAGIC))
        self.assertEqual(
            parse_marker(encoded),
            {'kind': kind, 'routine': routine, 'path': path, 'value': value}
        )

    def test_ignore_invalid(self):
        self.assertIsNone(parse_marker('notmagic'))


class TestInstrumentStep(unittest.TestCase):
    def test_step_before_after(self):
        step = {'StepType': 'X', 'Arguments': []}
        seq = instrument_step(step, 'routine', 1, 2)
        before, core, after = seq
        before, after = get_payload(before), get_payload(after)
        self.assertEqual(before['kind'], 'before')
        self.assertEqual(before['path'], [1, 2])
        self.assertEqual(core, step)  # don't care if same object
        self.assertEqual(after['kind'], 'after')
        self.assertEqual(after['path'], [1, 2])


class TestInstrumentStepGroup(unittest.TestCase):
    def test_group_before_enter_exit_after(self):
        group = {
            'Type': 'Sequence',
            'Steps': [
                {'StepType': 'A', 'Arguments': []},
            ]
        }

        before, core, after = instrument_stepgroup(group, 'routine', 0)

        self.assertEqual(before['Type'], 'Sequence')
        self.assertEqual(len(before['Steps']), 1)
        before = get_payload(before['Steps'][0])
        self.assertEqual(before['kind'], 'before')
        self.assertEqual(before['path'], [0])

        self.assertEqual(core['Type'], 'Sequence')
        steps = core.get('Steps', [])
        enter, exit_ = get_payload(steps[0]), get_payload(steps[-1])
        self.assertEqual(enter['kind'], 'enter')
        self.assertEqual(enter['path'], [0])
        self.assertEqual(exit_['kind'], 'exit')
        self.assertEqual(exit_['path'], [0])

        self.assertEqual(after['Type'], 'Sequence')
        self.assertEqual(len(after['Steps']), 1)
        after = get_payload(after['Steps'][0])
        self.assertEqual(after['kind'], 'after')
        self.assertEqual(after['path'], [0])


class TestInstrumentRoutine(unittest.TestCase):
    def test_routine_enter_exit(self):
        routine = [
            {
                'Type': 'Sequence',
                'Steps': [
                    {'StepType': 'A', 'Arguments': []},
                    {'StepType': 'B', 'Arguments': []},
                ]
            },
        ]

        groups = instrument_routine(routine, 'routine')
        enter, exit_ = groups[0], groups[-1]
        self.assertGreater(len(groups), 3)

        self.assertEqual(enter['Type'], 'Sequence')
        self.assertEqual(len(enter['Steps']), 1)
        enter = get_payload(enter['Steps'][0])
        self.assertEqual(enter['kind'], 'enter')
        self.assertEqual(enter['path'], [])

        self.assertEqual(exit_['Type'], 'Sequence')
        self.assertEqual(len(exit_['Steps']), 1)
        exit_ = get_payload(exit_['Steps'][0])
        self.assertEqual(exit_['kind'], 'exit')
        self.assertEqual(exit_['path'], [])


if __name__ == '__main__':
    unittest.main()
