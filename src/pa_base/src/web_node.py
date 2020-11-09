#!/usr/bin/env python2.7
import flask
import rospy

from ds_nmea_msgs.msg import Gga


last_location = {}

def on_message(msg):
    global last_location
    sign = { 'N': 1, 'E': 1, 'S': -1, 'W': -1 }
    last_location = {
        'latitude':  msg.latitude * sign[chr(msg.latitude_dir)],
        'longitude': msg.longitude * sign[chr(msg.longitude_dir)],
    }


app = flask.Flask(__name__)

@app.route('/')
def index():
    return flask.jsonify({
        'location': last_location,
    })


def main():
    # Note that rospy spawns threads for message dispatch, so we don't have to
    rospy.init_node('web_node')
    rospy.Subscriber('/nmea_listener/nmea_gga', Gga, on_message)

    app.run(host='0.0.0.0', port=8092)


if __name__ == '__main__':
    main()
