#!/usr/bin/env python3
import unittest

import rospy
import rostest

from gps_common.msg import GPSFix
from sensor_msgs.msg import NavSatFix


class TestGPSMock(unittest.TestCase):
    def test_stationary_fix(self):
        for topic, cls in [('fix', NavSatFix), ('extended_fix', GPSFix)]:
            msg = rospy.wait_for_message('/gps/' + topic, cls, timeout=5)
            self.assertEqual((msg.latitude, msg.longitude), (42.1, -69.9))
            self.assertGreater(msg.header.stamp.to_sec(), 0)


if __name__ == '__main__':
    rospy.init_node('gpsd_client_mock_test')
    rostest.rosrun('phyto_sim_utils', 'gpsd_client_mock', TestGPSMock)
