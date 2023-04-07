#!/usr/bin/env python3
import functools
import importlib
import io
import math

from aiohttp import web

from ds_sensor_msgs.msg import DepthPressure
from sensor_msgs.msg import NavSatFix
import rospy


config_validated = {}
response_values = {}


def capture_field_value(name, field, msg):
    # Since the data topic is dynamic, we need to re-interpret the data as the
    # correct type. This is a bit hacky, but described here:
    # http://schulz-m.github.io/2016/07/18/rospy-subscribe-to-any-msg-type/
    pkg, _, clsname = msg._connection_header['type'].partition('/')
    msg_class = getattr(importlib.import_module(f'{pkg}.msg'), clsname)
    if hasattr(msg_class, field):
        config_validated[name] = True
    else:
        rospy.logerr(f'Field {field} not found in msg type {msg_class}. Check config of {name}.')
    buf = io.BytesIO()
    msg.serialize(buf)
    parsed = msg_class().deserialize(buf.getvalue())
    field_value = getattr(parsed, field)
    response_values[name] = field_value


app = web.Application()
routes = web.RouteTableDef()


@routes.get('/')
async def index(request):
    for name, validated in config_validated.items():
        if not validated: 
            rospy.logwarn(f'For {name} config: No value received and no default, returning None')
    return web.json_response(response_values)


app.add_routes(routes)


def main():
    # Note that rospy spawns threads for message dispatch, so we don't have to
    rospy.init_node('web_node')

    # Read config and create subscribers accordingly
    field_map = rospy.get_param('~field_map')
    for name, config in field_map.items():
        # Keep track of which fields are known to exist or have defaults
        config_validated[name] = 'default' in config
        # Initialize with default values or None if no default provided
        response_values[name] = config.get('default')
        # Skip creating subscriber if topic not provided. Makes default permanent
        if 'topic' in config:
            rospy.Subscriber(config['topic'], rospy.AnyMsg,
                         functools.partial(capture_field_value, name, config['topic_field']))
    web.run_app(app, port=8098)


if __name__ == '__main__':
    main()
