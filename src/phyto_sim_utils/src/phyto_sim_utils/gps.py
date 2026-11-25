import rospy

from sensor_msgs.msg import NavSatFix, NavSatStatus


# Use the barge antenna in Gazebo or a stationary receiver on ROS time
def receive_fixes(callback):
    topic = rospy.get_param('~fix_topic', '')
    if topic:
        return rospy.Subscriber(topic, NavSatFix, callback, queue_size=1)

    fix = NavSatFix(latitude=rospy.get_param('~latitude', 41.524),
                    longitude=rospy.get_param('~longitude', -70.672),
                    altitude=rospy.get_param('~altitude', 0.0))
    fix.header.frame_id = 'barge'
    fix.status.service = NavSatStatus.SERVICE_GPS

    def tick(event):
        fix.header.stamp = event.current_real
        callback(fix)

    return rospy.Timer(rospy.Duration(1), tick, reset=True)
