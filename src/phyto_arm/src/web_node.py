#!/usr/bin/env python3
import math

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


def capture_field_value(name, path, default, msg):
    global field_map
    if msg[path]:
        field_map[name] = msg[path]
    elif default:
        field_map[name] = default
    


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
    global field_map
    field_config = rospy.get_param('/web_node')
    for name, config in field_config.items():
        if config.default:
            field_map[name] = config.default
        rospy.Subscriber(config.topic, object, lambda msg: capture_field_value(name, config.field, config.default, msg))


    rospy.Subscriber('/ctd/depth', DepthPressure, on_depth_message)
    rospy.Subscriber('/gps/fix', NavSatFix, on_gps_message)

    web.run_app(app, port=8098)


if __name__ == '__main__':
    main()
