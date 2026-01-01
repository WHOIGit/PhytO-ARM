#!/usr/bin/env python3
import random
import rospy

from std_msgs.msg import Float64
from ds_sensor_msgs.msg import Ctd, DepthPressure

last_depth = 0.5  # Default depth value

def generate_mock_ctd():
    ctd = Ctd()

    # Generate random or fixed values for CTD parameters
    ctd.conductivity = random.uniform(0, 5)  # Example random value, in S/m
    ctd.temperature = random.uniform(-2, 30)  # Example random value, in Celsius
    ctd.pressure = random.uniform(0, 600)  # Example random value, in dbar
    ctd.sound_speed = random.uniform(1400, 1600)  # Example random value, in m/s

    # Set covariance fields to -1, indicating "not valid"
    ctd.conductivity_covar = ctd.temperature_covar = ctd.pressure_covar = \
        ctd.salinity_covar = ctd.sound_speed_covar = -1
    return ctd

def depth_callback(data):
    global last_depth
    last_depth = data.data  # Update the last received depth
    rospy.loginfo(f"Received mock depth: {last_depth}")

def publish_depth_message():
    # Publish mock depth message using the last received depth value
    depth_msg = DepthPressure()
    depth_msg.depth = last_depth
    depth_msg.pressure = last_depth * 10  # Example conversion to pressure
    depth_pub.publish(depth_msg)
    rospy.loginfo(f"Published mock depth: {depth_msg.depth}")

def main():
    global depth_pub
    rospy.init_node('mock_sensor_node')
    rospy.logwarn(f'Starting mock sensor node {rospy.get_name()}')

    # Subscribe to the mock depth topic from the 'mock_winch'
    rospy.Subscriber("mock_depth", Float64, depth_callback)

    # Publishers for mock depth and CTD messages
    depth_pub = rospy.Publisher('~depth', DepthPressure, queue_size=10)
    ctd_pub = rospy.Publisher('~ctd', Ctd, queue_size=10)

    # Publish mock CTD messages at a regular interval, along with the corresponding depth message
    rate = rospy.Rate(1)  # 1 Hz
    while not rospy.is_shutdown():
        publish_depth_message()  # Publish the latest (or default) depth message
        ctd_msg = generate_mock_ctd()
        ctd_pub.publish(ctd_msg)
        rospy.loginfo("Published mock CTD message")
        rate.sleep()

if __name__ == '__main__':
    main()
