#!/usr/bin/env python3
import functools

import numpy as np
import rospy

# cv2 unused, but import required to solve unreported exception
# See https://answers.ros.org/question/362388/cv_bridge_boost-raised-unreported-exception-when-importing-cv_bridge/
import cv2
from cv_bridge import CvBridge
from PIL import Image as PilImage
from sensor_msgs.msg import Image

from triton_api import Model, ImageInput, ClassificationOutput, ScalingMode, initialize_model
from triton_classifier.msg import Classification, ObjectHypothesisWithClassName



def on_image(model, class_pub, image_msg):
    # Use the cv_bridge to convert to an OpenCV image object
    img = CvBridge().imgmsg_to_cv2(image_msg)
    # Convert the OpenCV image to a PIL image
    pil_image = PilImage.fromarray(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))

    # Ask the classifier to infer result
    try:
        result = model.infer(pil_image)
    except ValueError as e:
        rospy.logerr('Error getting classification from Triton: %s', e)
        rospy.logerr(str(e))
        return
    if len(result.output) != 1 or len(result.output[0]) < 1:
        rospy.logerr('Unexpected result from classifier: %s', result)
        return
    classification = Classification()
    classification.header = image_msg.header
    classification.results = []
    for r in result.output[0]:
        h = ObjectHypothesisWithClassName()
        h.class_name = r.class_name
        h.score = r.score
        classification.results.append(h)
    class_pub.publish(classification)


def main():
    rospy.init_node('classifier', anonymous=True)

    model = initialize_model(rospy.get_param('~triton_server_url'), rospy.get_param('~classifier_model'))
    model.input = ImageInput(scaling=ScalingMode.INCEPTION)
    model.output = ClassificationOutput(classes=3)

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
