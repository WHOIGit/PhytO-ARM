#!/usr/bin/env python3
import json

from typing import Any, Dict, List, Optional


Step = Dict[str, Any]
StepGroup = Dict[str, Any]
Routine = List[StepGroup]


# This magic prefix identifies a report as being our instrumentation.
# Increment the version whenever an incompatible change is introduced.
MAGIC = '$phyto-arm$0$'


# A marker is a Report step with a JSON-encoded payload indicating which
# routine, step group, or step is starting/completing.
def marker(kind: str, routine: str, path: List[int], node: Any) -> Step:
    return {
        'StepType': 'Report',
        'Arguments': [
            MAGIC + json.dumps({
                'kind': kind,
                'routine': routine,
                'path': path,
                'value': node,
            }, separators=(',', ':')),  # minimize whitespace
        ]
    }


# Given a report string, decodes the marker or returns None if it is not a valid
# marker.
def parse_marker(marker: str) -> Optional[Dict[str, Any]]:
    if not marker.startswith(MAGIC):
        return None
    return json.loads(marker[len(MAGIC):])


# Index steps by type and arguments, preserving JSON argument types
def step_key(step: Step):
    return step['StepType'], json.dumps(step.get('Arguments', []),
                                        sort_keys=True)


# Resolve each selector to a position in the original routine
def selected_paths(selectors, paths):
    result = set()
    for selector in selectors:
        try:
            step = json.loads(selector)
            if (not isinstance(step, dict)
                    or not isinstance(step.get('StepType'), str)
                    or not isinstance(step.get('Arguments', []), list)
                    or step.keys() - {'StepType', 'Arguments'}):
                raise ValueError
        except ValueError:
            raise ValueError(f'Invalid step selector: {selector}') from None

        matches = paths.get(step_key(step), [])
        if len(matches) != 1:
            raise ValueError(f'Expected one match for {selector}, '
                             f'found {len(matches)}')
        result.add(matches[0])
    return result


def instrument_routine(routine: Routine, name: str, *, before=(), after=(),
                       trace=True) -> Routine:
    # Validate required markers even when the caller requests a full trace
    if before or after:
        paths = {}
        for i, group in enumerate(routine):
            for j, step in enumerate(group.get('Steps', [])):
                paths.setdefault(step_key(step), []).append((i, j))
        before = selected_paths(before, paths)
        after = selected_paths(after, paths)

    # Keep selected markers inside the step's own condition or iteration
    if not trace:
        if not before and not after:
            return routine
        groups = []
        for i, group in enumerate(routine):
            steps = []
            for j, step in enumerate(group.get('Steps', [])):
                if (i, j) in before:
                    steps.append(marker('before', name, [i, j], step))
                steps.append(step)
                if (i, j) in after:
                    steps.append(marker('after', name, [i, j], step))
            groups.append(dict(group, Steps=steps))
        return groups

    # Trace the routine, its groups, and each step
    result: Routine = []
    result.append({
        'Type': 'Sequence',
        'Steps': [
            marker('enter', name, [], routine),
        ],
    })
    for i, group in enumerate(routine):
        result.extend(instrument_stepgroup(group, name, i))
    result.append({
        'Type': 'Sequence',
        'Steps': [
            marker('exit', name, [], routine)
        ],
    })
    return result


def instrument_stepgroup(group: StepGroup, routine_name: str, group_i: int) -> \
List[StepGroup]:
    new_steps: List[Step] = []
    new_steps.append(marker('enter', routine_name, [group_i], group))
    for j, step in enumerate(group.get('Steps', [])):
        new_steps.extend(instrument_step(step, routine_name, group_i, j))
    new_steps.append(marker('exit', routine_name, [group_i], group))

    inner = group.copy()
    inner['Steps'] = new_steps

    return [
        {
            'Type': 'Sequence',
            'Steps': [
                marker('before', routine_name, [group_i], group),
            ],
        },
        inner,
        {
            'Type': 'Sequence',
            'Steps': [
                marker('after', routine_name, [group_i], group),
            ],
        }
    ]


def instrument_step(step: Step, routine_name: str, group_i: int, step_j: int) \
-> List[Step]:
    return [
        marker('before', routine_name, [group_i, step_j], step),
        step,
        marker('after', routine_name, [group_i, step_j], step),
    ]


if __name__ == '__main__':
    import argparse
    import os

    parser = argparse.ArgumentParser()
    parser.add_argument('input')
    args = parser.parse_args()

    with open(args.input) as f:
        routine_name, _ = os.path.splitext(os.path.basename(args.input))
        out = instrument_routine(json.load(f), routine_name)
        print(json.dumps(out, indent=True))
