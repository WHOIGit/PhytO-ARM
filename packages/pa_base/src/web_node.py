#!/usr/bin/env python2.7
import flask
import rospy

from ds_core_msgs.msg import RawData


last_message = ''

def on_message(msg):
    global last_message
    last_message = msg.data


app = flask.Flask(__name__)

@app.route('/')
def index():
    return last_message


def main():
    # Note that rospy spawns threads for message dispatch, so we don't have to
    rospy.init_node('web_node')
    rospy.Subscriber('/nmea_listener/in', RawData, on_message)

    app.run(host='0.0.0.0', port=8000)


if __name__ == '__main__':
    main()
