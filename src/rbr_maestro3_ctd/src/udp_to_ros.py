#!/usr/bin/env python

import socket

import rospy
from std_msgs.msg import String

def udp_to_ros_node():
    # Initialize the ROS node
    rospy.init_node('udp_to_ros_node', anonymous=True)
    
    # Create a ROS publisher
    pub = rospy.Publisher('udp_stream', String, queue_size=10)

    # Setup UDP socket
    UDP_IP = "0.0.0.0"
    UDP_PORT = 12345
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_IP, UDP_PORT))

    rospy.loginfo("Listening for UDP data on %s:%d...", UDP_IP, UDP_PORT)

    while not rospy.is_shutdown():
        data, addr = sock.recvfrom(1024)
        # Publish the received data
        pub.publish(data)

if __name__ == '__main__':
    try:
        udp_to_ros_node()
    except rospy.ROSInterruptException:
        pass
