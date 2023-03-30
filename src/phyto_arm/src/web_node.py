#!/usr/bin/env python3
import math
import importlib
import io
import functools
import rospy

from aiohttp import web
from ds_sensor_msgs.msg import DepthPressure
from sensor_msgs.msg import NavSatFix


last_depth = math.nan
last_location = {}
field_map = {}


def on_depth_message(msg):
    global last_depth
    last_depth = msg.depth
    if last_depth == DepthPressure.DEPTH_PRESSURE_NO_DATA:
        last_depth = math.nan

def on_gps_message(msg):
    # These field names are chosen to match IFCBacquire so that our values end
    # up in the same place in the HDR file.
    global last_location
    last_location = {
        'gpsLatitude':  msg.latitude,
        'gpsLongitude': msg.longitude,
    }


def capture_field_value(name, field, default, msg):
    # Since the data topic is dynamic, we need to re-interpret the data as the
    # correct type. This is a bit hacky, but described here:
    # http://schulz-m.github.io/2016/07/18/rospy-subscribe-to-any-msg-type/
    pkg, _, clsname = msg._connection_header['type'].partition('/')
    msg_class = getattr(importlib.import_module(f'{pkg}.msg'), clsname)
    buf = io.BytesIO()
    msg.serialize(buf)
    parsed = msg_class().deserialize(buf.getvalue())
    field_map[name] = parsed[field] if parsed[field] else default


app = web.Application()
routes = web.RouteTableDef()


@routes.get('/')
async def index(request):
    return web.json_response({
        'depth': last_depth,
        **last_location,
        **field_map
    })


app.add_routes(routes)


def main():
    # Note that rospy spawns threads for message dispatch, so we don't have to
    rospy.init_node('web_node')

    # Read config and create subscribers accordingly
    field_map = rospy.get_param('~field_map')
    for name, config in field_map.items():
        if config.default:
            field_map[name] = config.default
        rospy.Subscriber(config.topic, rospy.AnyMsg,
                         functools.partial(capture_field_value, name, config.topic_field, config.default))


    rospy.Subscriber('/ctd/depth', DepthPressure, on_depth_message)
    rospy.Subscriber('/gps/fix', NavSatFix, on_gps_message)

    web.run_app(app, port=8098)


if __name__ == '__main__':
    main()
