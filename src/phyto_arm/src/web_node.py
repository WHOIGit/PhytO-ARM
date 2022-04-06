#!/usr/bin/env python3
import rospy

from aiohttp import web
from sensor_msgs.msg import NavSatFix


last_location = {}

def on_message(msg):
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
        'location': last_location,
    })


app.add_routes(routes)


def main():
    # Note that rospy spawns threads for message dispatch, so we don't have to
    rospy.init_node('web_node')
    rospy.Subscriber('/gps/fix', NavSatFix, on_message)

    web.run_app(app, port=8098)


if __name__ == '__main__':
    main()
