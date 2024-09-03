#!/usr/bin/env python3
import rospy
import json
import requests
from requests.auth import HTTPDigestAuth

from phyto_arm.msg import OutletStatus

def control_outlet(msg):
    username = rospy.get_param('/digital_logger/username')
    password = rospy.get_param('/digital_logger/password')
    address = rospy.get_param('/digital_logger/address')
    outlets = rospy.get_param('/digital_logger/outlets')

    for outlet in outlets:
        if outlet['name'] == msg.name:
            outlet_num = int(outlet['outlet']) - 1

    status = msg.status == 'on'

    response = requests.put(f'http://{address}/restapi/relay/outlets/{outlet_num}/state/', auth=HTTPDigestAuth(username, password), data={'value': status}, headers={"X-CSRF": "x", "Accept": "application/json"})
    rospy.logwarn(f'sent: http://{address}/restapi/relay/outlets/{outlet_num}/state/, {status}, {response.text}')


def main():
    rospy.init_node('digital_logger_node')

    subscriber = rospy.Subscriber('/digital_logger/control', OutletStatus, control_outlet)

    username = rospy.get_param('/digital_logger/username')
    password = rospy.get_param('/digital_logger/password')
    address = rospy.get_param('/digital_logger/address')
    outlets = rospy.get_param('/digital_logger/outlets')
    num_outlets = len(outlets)

    outlet_publishers = {}
    for outlet_num in range(num_outlets):
        outlet_publishers[f'outlet_{outlet_num}'] = rospy.Publisher(f'/digital_logger/outlet/{outlet_num}/status/', OutletStatus, queue_size=10)

    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        response = requests.get(f'http://{username}:{password}@{address}/restapi/relay/outlets/=0,1,2,3,4,5,6,7/state/')

        result = json.loads(response.text)

        assert len(result) == num_outlets

        for outlet_index in range(len(result)):
            if result[outlet_index]:
                status = 'on'
            else:
                status = 'off'

            outlet_status = OutletStatus()
            outlet_status.name = outlets[outlet_index]['name']
            outlet_status.status = status

            outlet_publishers[f'outlet_{outlet_index}'].publish(outlet_status)

        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
