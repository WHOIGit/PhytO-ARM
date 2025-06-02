#!/usr/bin/env python3
import re

from http.server import BaseHTTPRequestHandler, HTTPServer

import rospy


class OutletHandler(BaseHTTPRequestHandler):
    def outlet(self):
        match = re.fullmatch(r'/restapi/relay/outlets/([0-7])/state/',
                             self.path)
        if match:
            return int(match[1])
        self.send_error(404)

    def do_GET(self):
        number = self.outlet()
        if number is None:
            return

        body = b'true' if self.server.outlets[number] else b'false'
        self.send_response(200)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_PUT(self):
        number = self.outlet()
        if number is None:
            return

        # Accept the two form values sent by the DLI client
        body = self.rfile.read(int(self.headers.get('Content-Length', 0)))
        if body not in (b'value=true', b'value=false'):
            self.send_error(400)
            return

        self.server.outlets[number] = body == b'value=true'
        self.send_response(204)
        self.end_headers()

    # Silence the request log because the driver polls once per ROS second
    def log_request(self, code='-', size='-'):
        pass


def main():
    rospy.init_node('dli_sim')
    address = ('127.0.0.1', rospy.get_param('~port'))
    with HTTPServer(address, OutletHandler) as server:
        server.outlets = [True] * 8
        server.timeout = 0.25
        while not rospy.is_shutdown():
            server.handle_request()


if __name__ == '__main__':
    main()
