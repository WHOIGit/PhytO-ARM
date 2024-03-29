#!/usr/bin/env python3
import rospy
import numpy as np
from scipy.stats import norm

from ds_sensor_msgs.msg import DepthPressure
from phyto_arm.msg import MoveToDepthActionGoal, MoveToDepthActionResult, DepthProfile

class MockProfiler:
    def __init__(self):
        self.depth_sub = rospy.Subscriber(rospy.get_param('ctd_topic'), DepthPressure, self.depth_callback)
        self.action_goal_sub = rospy.Subscriber("winch/move_to_depth/goal", MoveToDepthActionGoal, self.action_goal_callback)
        self.action_result_sub = rospy.Subscriber("winch/move_to_depth/result", MoveToDepthActionResult, self.action_result_callback)

        self.profile_pub = {
            'all': rospy.Publisher('~', DepthProfile, queue_size=1),
            'down': rospy.Publisher('~downcast', DepthProfile, queue_size=1),
            'up': rospy.Publisher('~upcast', DepthProfile, queue_size=1),
        }

        self.depths = []
        self.is_recording = False

    def depth_callback(self, data):
        if self.is_recording:
            self.depths.append(data.depth)

    def action_goal_callback(self, data):
        rospy.loginfo("Received action goal, starting recording")
        self.is_recording = True
        self.depths = []

    def action_result_callback(self, data):
        rospy.loginfo("Received action result, stopping recording")
        self.is_recording = False

        # Generate and publish mocked DepthProfile messages
        profile_msg = DepthProfile()
        profile_msg.header.stamp = rospy.Time.now()
        profile_msg.goal_uuid = data.result.uuid
        profile_msg.data_topic = rospy.get_param('ctd_topic')
        profile_msg.data_field = "mocked_values"
        profile_msg.depths = self.depths

        # Generate mocked values using a bell curve
        mean = 5.0  # Mean depth of the bell curve
        std_dev = 1.5  # Standard deviation of the bell curve
        profile_msg.values = norm.pdf(self.depths, mean, std_dev).tolist()

        self.profile_pub['all'].publish(profile_msg)
        if self.depths[0] < self.depths[-1]:
            self.profile_pub['down'].publish(profile_msg)
        else:
            self.profile_pub['up'].publish(profile_msg)

        rospy.loginfo("Published mocked DepthProfile")

def main():
    rospy.init_node('mock_profiler_node')
    rospy.logwarn(f'Starting mock profiler node {rospy.get_name()}')

    profiler = MockProfiler()
    rospy.spin()

if __name__ == '__main__':
    main()
