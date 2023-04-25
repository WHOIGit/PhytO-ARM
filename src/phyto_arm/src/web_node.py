#!/usr/bin/env python3
import functools
import importlib
import io
import os

import rospy

from aiohttp import web


response_values = {}

def capture_field_value(name, field, msg):
    # Since the data topic is dynamic, we need to re-interpret the data as the
    # correct type. This is a bit hacky, but described here:
    # http://schulz-m.github.io/2016/07/18/rospy-subscribe-to-any-msg-type/
    pkg, _, clsname = msg._connection_header['type'].partition('/')
    msg_class = getattr(importlib.import_module(f'{pkg}.msg'), clsname)
    if not hasattr(msg_class, field):
        rospy.logerr(f'Field {field} not found in msg type {msg_class}. Check config {name}.')
    buf = io.BytesIO()
    msg.serialize(buf)
    parsed = msg_class().deserialize(buf.getvalue())
    field_value = getattr(parsed, field)
    response_values[name] = field_value


app = web.Application()
routes = web.RouteTableDef()


@routes.get('/')
async def index(request):
    return web.json_response(response_values)


app.add_routes(routes)


def main():
    # Note that rospy spawns threads for message dispatch, so we don't have to
    rospy.init_node('web_node')

    # Read config, set defaults, create subscribers
    field_map = rospy.get_param('~field_map')
    for name, config in field_map.items():
        if 'environment' in config:
            if 'topic' in config:
                rospy.logerr(f'Config {name} invalid. Mixing topic and environment not supported')
            var_name = config['environment']
            if var_name in os.environ:
                response_values[name] = os.environ[var_name]
            elif 'default' in config:
                rospy.logwarn(f'Env var {var_name} not found, switching to default for config {name}')
                response_values[name] = config['default']
            else:
                rospy.logerr(f'Config {name} invalid. Env var {var_name} not found and no default provided')
        elif 'default' in config:
            response_values[name] = config['default']
            # Subscriber optional; skip if no topic. Makes default permanent
            if 'topic' in config:
                if 'topic_field' not in config:
                    rospy.logerr(f'Config {name} invalid, topic_field required when topic used')
                rospy.Subscriber(config['topic'], rospy.AnyMsg,
                                functools.partial(capture_field_value, name, config['topic_field']))
        else:
            rospy.logerr(f'Config {name} invalid, must have "default" or "environment" set')
    web.run_app(app, port=8098)


if __name__ == '__main__':
    main()
