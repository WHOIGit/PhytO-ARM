#!/usr/bin/env python3
import functools

import numpy as np
import rospy

# cv2 unused, but import required to solve unreported exception
# See https://answers.ros.org/question/362388/cv_bridge_boost-raised-unreported-exception-when-importing-cv_bridge/
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

from triton_api import Model, ImageInput, ClassificationOutput, ScalingMode, initialize_model
from triton_classifier.msg import Classification



def on_image(model, class_pub, image_msg):
    # Use the cv_bridge to convert to an OpenCV image object
    img = CvBridge().imgmsg_to_cv2(image_msg)
    
    # Ask the classifier to 
    result = model.infer(img)
    if len(result[0]) != 1 or len(result[0][0]) != 1:
        rospy.logerr('Unexpected result from classifier: %s', r)
        return
    class_pub.publish(result[0][0])


def main():
    rospy.init_node('classifier', anonymous=True)

    model = initialize_model(rospy.get_param('~triton_server_url'), rospy.get_param('~classifier_model'))
    model.input = ImageInput(scaling=ScalingMode.INCEPTION)
    model.output = ClassificationOutput(classes=1)

    # Advertise that we will publish a "/class" subtopic of the image topic
    class_pub = rospy.Publisher(
        rospy.get_param('~image_topic') + '/class',
        Classification,
        queue_size=1
    )

    # Subscribe to the raw image data
    rospy.Subscriber(
        rospy.get_param('~image_topic') + '/raw',
        Image,
        functools.partial(on_image, model, class_pub)
    )

    rospy.spin()


if __name__ == '__main__':
    main()
