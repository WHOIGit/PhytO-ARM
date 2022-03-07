#!/usr/bin/env python3
import rospy

from aiohttp import web
from ds_nmea_msgs.msg import Gga


last_location = {}

def on_message(msg):
    global last_location
    sign = { 'N': 1, 'E': 1, 'S': -1, 'W': -1 }
    last_location = {
        'latitude':  msg.latitude * sign[chr(msg.latitude_dir)],
        'longitude': msg.longitude * sign[chr(msg.longitude_dir)],
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
    rospy.Subscriber('/nmea_listener/nmea_gga', Gga, on_message)

    web.run_app(app, port=8098)


if __name__ == '__main__':
    main()
