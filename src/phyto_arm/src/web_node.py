#!/usr/bin/env python3
import math
import importlib
import io
import functools
import rospy

from aiohttp import web
from ds_sensor_msgs.msg import DepthPressure
from sensor_msgs.msg import NavSatFix


response_values = {}


def capture_field_value(name, field, default, msg):
    # Since the data topic is dynamic, we need to re-interpret the data as the
    # correct type. This is a bit hacky, but described here:
    # http://schulz-m.github.io/2016/07/18/rospy-subscribe-to-any-msg-type/
    pkg, _, clsname = msg._connection_header['type'].partition('/')
    msg_class = getattr(importlib.import_module(f'{pkg}.msg'), clsname)
    buf = io.BytesIO()
    msg.serialize(buf)
    parsed = msg_class().deserialize(buf.getvalue())
    response_values[name] = getattr(parsed, field, default or 'Field not found, no default provided')


app = web.Application()
routes = web.RouteTableDef()


@routes.get('/')
async def index(request):
    return web.json_response(response_values)


app.add_routes(routes)


def main():
    # Note that rospy spawns threads for message dispatch, so we don't have to
    rospy.init_node('web_node')

    # Read config and create subscribers accordingly
    field_map = rospy.get_param('~field_map')
    for name, config in field_map.items():
        # Initialize with default values or initial message if none provided
        default = config.get('default')
        response_values[name] = default or 'Initial value, no default provided'
        # Skip creating subscriber if topic not provided. Makes default permanent
        if 'topic' in config:
            rospy.Subscriber(config['topic'], rospy.AnyMsg,
                         functools.partial(capture_field_value, name, config['topic_field'], default))
    web.run_app(app, port=8098)


if __name__ == '__main__':
    main()
