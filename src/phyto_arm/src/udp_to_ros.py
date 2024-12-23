#!/usr/bin/env python3

import socket
import rospy
import json
from ds_core_msgs.msg import RawData
from std_msgs.msg import Float32, String, Int32, Float32MultiArray, Int32MultiArray

publishers = {}
raw_pub = None
sock = None
buffer = b""
decoder = json.JSONDecoder()

def get_ros_msg_type(value):
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

def publish_message(json_data, base_topic):
    for key, value in json_data.items():
        if key not in publishers:
            msg_type = get_ros_msg_type(value)
            publishers[key] = {
                'publisher': rospy.Publisher(f"{base_topic}/{key}", msg_type, queue_size=10),
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
            msg = publishers[key]['msg_type'](data=value)

        publishers[key]['publisher'].publish(msg)

def publish_raw_message(data):
    msg = RawData()
    msg.data = list(data)
    msg.data_direction = RawData.DATA_IN
    current_time = rospy.Time.now()
    msg.ds_header.io_time = current_time
    msg.header.stamp = current_time
    raw_pub.publish(msg)

def main():
    global raw_pub, sock, buffer
    rospy.init_node('udp_to_ros_node', anonymous=True)

    parse_json = rospy.get_param('~parse_json', False)
    base_topic = rospy.get_param('~topic', 'udp_stream')
    UDP_IP = "0.0.0.0"
    UDP_PORT = rospy.get_param('~port')

    rospy.loginfo(f"Listening for UDP data on {UDP_IP}:{UDP_PORT}...")
    rospy.loginfo(f"Base topic: {base_topic}")
    rospy.loginfo(f"JSON parsing: {'enabled' if parse_json else 'disabled'}")

    if not parse_json:
        raw_pub = rospy.Publisher(base_topic, RawData, queue_size=10)

    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind((UDP_IP, UDP_PORT))
    except socket.error as e:
        rospy.logerr(f"Failed to create UDP socket: {str(e)}")
        raise

    while not rospy.is_shutdown():
        try:
            data, _ = sock.recvfrom(65536)
            buffer += data

            if parse_json:
                while buffer:
                    try:
                        _, index = decoder.raw_decode(buffer.decode('utf-8'))
                        message = buffer[:index].strip()
                        buffer = buffer[index:]

                        json_data = json.loads(message)
                        publish_message(json_data, base_topic)
                    except json.JSONDecodeError:
                        break
            else:
                while b'\n' in buffer:
                    newline_index = buffer.find(b'\n')
                    message = buffer[:newline_index]
                    publish_raw_message(message)
                    buffer = buffer[newline_index+1:]

        except socket.error as e:
            rospy.logerr(f"Error receiving UDP data: {str(e)}")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
