#!/usr/bin/env python3

import socket
import rospy
import json
from ds_core_msgs.msg import RawData
from std_msgs.msg import Float32, String, Int32, Float32MultiArray, Int32MultiArray

class UdpToRosNode:
    def __init__(self):
        rospy.init_node('udp_to_ros_node', anonymous=True)

        self.parse_json = rospy.get_param('~parse_json', False)
        self.base_topic = rospy.get_param('~topic', 'udp_stream')
        self.UDP_IP = "0.0.0.0"
        self.UDP_PORT = rospy.get_param('~port')

        rospy.loginfo(f"Listening for UDP data on {self.UDP_IP}:{self.UDP_PORT}...")
        rospy.loginfo(f"Base topic: {self.base_topic}")
        rospy.loginfo(f"JSON parsing: {'enabled' if self.parse_json else 'disabled'}")

        self.publishers = {}
        self.raw_pub = None
        if not self.parse_json:
            self.raw_pub = rospy.Publisher(self.base_topic, RawData, queue_size=10)

        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.sock.bind((self.UDP_IP, self.UDP_PORT))
        except socket.error as e:
            rospy.logerr(f"Failed to create UDP socket: {str(e)}")
            raise

        self.buffer = b""
        self.decoder = json.JSONDecoder()

    def get_ros_msg_type(self, value):
        if isinstance(value, float):
            return Float32
        elif isinstance(value, int):
            return Int32
        elif isinstance(value, str):
            return String
        elif isinstance(value, list):
            if all(isinstance(x, float) for x in value):
                return Float32MultiArray
            elif all(isinstance(x, int) for x in value):
                return Int32MultiArray
        return String

    def publish_message(self, json_data):
        for key, value in json_data.items():
            if key not in self.publishers:
                msg_type = self.get_ros_msg_type(value)
                self.publishers[key] = {
                    'publisher': rospy.Publisher(f"{self.base_topic}/{key}", msg_type, queue_size=10),
                    'msg_type': msg_type
                }

            if isinstance(value, list):
                if all(isinstance(x, float) for x in value):
                    msg = Float32MultiArray(data=value)
                elif all(isinstance(x, int) for x in value):
                    msg = Int32MultiArray(data=value)
                else:
                    msg = String(data=str(value))
            else:
                msg = self.publishers[key]['msg_type'](data=value)

            self.publishers[key]['publisher'].publish(msg)

    def publish_raw_message(self, data):
        msg = RawData()
        msg.data = list(data)
        msg.data_direction = RawData.DATA_IN
        current_time = rospy.Time.now()
        msg.ds_header.io_time = current_time
        msg.header.stamp = current_time
        self.raw_pub.publish(msg)

    def run(self):
        while not rospy.is_shutdown():
            try:
                data, _ = self.sock.recvfrom(65536)  # Increased buffer size
                self.buffer += data

                if self.parse_json:
                    while self.buffer:
                        try:
                            result, index = self.decoder.raw_decode(self.buffer.decode('utf-8'))
                            message = self.buffer[:index].strip()
                            self.buffer = self.buffer[index:]

                            json_data = json.loads(message)
                            self.publish_message(json_data)
                        except json.JSONDecodeError:
                            # If we can't decode a complete object, wait for more data
                            break
                else:
                    while b'\n' in self.buffer:
                        newline_index = self.buffer.find(b'\n')
                        message = self.buffer[:newline_index]
                        self.publish_raw_message(message)
                        self.buffer = self.buffer[newline_index+1:]

            except socket.error as e:
                rospy.logerr(f"Error receiving UDP data: {str(e)}")

if __name__ == '__main__':
    try:
        node = UdpToRosNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
