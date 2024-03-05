#!/usr/bin/env python3

import socket
import rospy
from ds_core_msgs.msg import RawData


def publish_message(pub, data):
    msg = RawData()

    # Set io_time to the current time
    current_time = rospy.Time.now()
    msg.ds_header.io_time = current_time
    msg.header.stamp = current_time
    msg.data_direction = RawData.DATA_IN
    msg.data = list(data)
    pub.publish(msg)


def udp_to_ros_node():
    # Initialize the ROS node
    rospy.init_node('udp_to_ros_node', anonymous=True)

    # Create a ROS publisher
    pub = rospy.Publisher('udp_stream', RawData, queue_size=10)

    # Setup UDP socket
    UDP_IP = "0.0.0.0"
    UDP_PORT = rospy.get_param('~port')
    rospy.loginfo("Listening for UDP data on %s:%d...", UDP_IP, UDP_PORT)

    # Create UDP socket
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind((UDP_IP, UDP_PORT))
    except socket.error as e:
        rospy.logerr("Failed to create UDP socket: %s", str(e))
        return

    buffer = b""
    buffer_size = 32768  # 32KB buffer size
    
    while not rospy.is_shutdown():
        try:
            data, addr = sock.recvfrom(buffer_size)
            buffer += data

            # Process all complete messages in the buffer
            while b'\n' in buffer:
                newline_index = buffer.find(b'\n')

                # Extract the message up to the newline and process it
                message = buffer[:newline_index]
                publish_message(pub, message)

                # Remove the processed message from the buffer
                buffer = buffer[newline_index+1:]

        except socket.error as e:
            rospy.logerr("Error receiving UDP data: %s", str(e))


if __name__ == '__main__':
    try:
        udp_to_ros_node()
    except rospy.ROSInterruptException:
        pass
