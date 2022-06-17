#!/usr/bin/env python3
import math

import rospy

from aiohttp import web
from ds_sensor_msgs.msg import DepthPressure
from sensor_msgs.msg import NavSatFix


last_depth = math.nan
last_location = {}


def on_depth_message(msg):
    global last_depth
    last_depth = msg.depth
    if last_depth == DepthPressure.DEPTH_PRESSURE_NO_DATA:
        last_depth = math.nan

def on_gps_message(msg):
    global last_location
    last_location = {
        'latitude':  msg.latitude,
        'longitude': msg.longitude,
    }


app = web.Application()
routes = web.RouteTableDef()


@routes.get('/')
async def index(request):
    return web.json_response({
        'depth': last_depth,
        'location': last_location,
    })


app.add_routes(routes)


def main():
    # Note that rospy spawns threads for message dispatch, so we don't have to
    rospy.init_node('web_node')
    rospy.Subscriber('/ctd/depth', DepthPressure, on_depth_message)
    rospy.Subscriber('/gps/fix', NavSatFix, on_gps_message)

    web.run_app(app, port=8098)


if __name__ == '__main__':
    main()
