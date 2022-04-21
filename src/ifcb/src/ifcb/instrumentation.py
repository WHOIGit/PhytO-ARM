#!/usr/bin/env python3
import copy
import json


# This magic prefix identifies a report as being our instrumentation.
# Increment the version whenever an incompatible change is introduced.
MAGIC = '$phyto-arm$0$'


# A marker is a routine step of type Report which includes a JSON-encoded
# structure that tells clients what part of the routine is being executed.
def marker(kind, routine, path, node):
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
def parse_marker(marker):
    if not marker.startswith(MAGIC):
        return None
    return json.loads(marker[len(MAGIC):])


# Instruments a node with markers.
def instrument(node, routine='', path=None):
    path = path or []

    if isinstance(node, list):
        children = node
    elif 'Steps' in node:
        children = node['Steps']
    else:
        return node  # return unmodified

    new_children = []

    # Helper function to append a marker, creating a new Sequence to contain
    # it if necessary.
    def append_marker(*args, **kwargs):
        if path == []:
            # At the top level we need to create a new Sequence
            new_children.append({
                'Type': 'Sequence',
                'Steps': [ marker(*args, **kwargs) ],
            })
        else:
            new_children.append(marker(*args, **kwargs))

    # Instrument this node and its children
    append_marker('enter', routine, path, node)
    for i, child in enumerate(children):
        append_marker('before', routine, path + [i], child)
        new_children.append(instrument(child, routine, path + [i]))
        append_marker('after', routine, path + [i], child)
    append_marker('exit', routine, path, node)

    # Return the instrumented object
    if isinstance(node, list):
        return new_children
    else:
        new_node = copy.deepcopy(node)
        new_node['Steps'] = new_children
        return new_node


if __name__ == '__main__':
    import argparse
    import os

    parser = argparse.ArgumentParser()
    parser.add_argument('input')
    args = parser.parse_args()

    with open(args.input) as f:
        routine, _ = os.path.splitext(os.path.basename(args.input))
        out = instrument(json.load(f), routine=routine)
        print(json.dumps(out, indent=True))
