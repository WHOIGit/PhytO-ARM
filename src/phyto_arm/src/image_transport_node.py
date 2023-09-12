#!/usr/bin/env python3
import functools
import io

import cv2
from cv_bridge import CvBridge
import numpy as np
from PIL import Image as PilImage
import rospy
from sensor_msgs.msg import CompressedImage, Image


def republish_image(raw_pub, image):
    bridge = CvBridge()

    # Convert PNG to raw and publish as Image message
    image_stream = io.BytesIO(image.data)
    pil_image = PilImage.open(image_stream)
    opencv_image = np.array(pil_image)
    # Convert RGB to BGR
    opencv_image = cv2.cvtColor(opencv_image, cv2.COLOR_RGB2BGR)
    image_message = bridge.cv2_to_imgmsg(opencv_image)
    image_message.header.stamp = rospy.Time.now()
    raw_pub.publish(image_message)


def main():
    rospy.init_node('image_transport_node')

    raw_pub = rospy.Publisher('/ifcb/roi/image/raw', Image, queue_size=5)
    rospy.Subscriber('/ifcb/roi/image', CompressedImage, functools.partial(republish_image, raw_pub))

    rospy.spin()

if __name__ == '__main__':
    main()
