#!/usr/bin/env python3
import json

import requests
import rospy

from phyto_arm.msg import OutletStatus

def control_outlet(msg):
    """
    Send the given msg to the digital logger as an HTTP request.
    """
    username = rospy.get_param('~username')
    password = rospy.get_param('~password')
    address = rospy.get_param('~address')
    outlets = rospy.get_param('~outlets')

    for outlet in outlets:
        if outlet['name'] == msg.name:
            # outlet numbers in the config yaml start from 1 whereas outlet numbers in the API start from 0
            outlet_num = int(outlet['outlet']) - 1

    status = msg.status == 'on'

    response = requests.put(f'http://{address}/restapi/relay/outlets/{outlet_num}/state/', auth=requests.auth.HTTPDigestAuth(username, password), data={'value': str(status).lower()}, headers={"X-CSRF": "x", "Accept": "application/json"})
    rospy.loginfo(f'sent: http://{address}/restapi/relay/outlets/{outlet_num}/state/, auth={username},{password} status={str(status).lower()}, received: code {response.status_code} : {response.text}')


def main():
    rospy.init_node('digital_logger')

    # subscribe to the digital logger control topic
    subscriber = rospy.Subscriber('/digital_logger/control', OutletStatus, control_outlet)

    username = rospy.get_param('~username')
    password = rospy.get_param('~password')
    address = rospy.get_param('~address')
    outlets = rospy.get_param('~outlets')
    num_outlets = len(outlets)

    # create an independent publisher for each outlet
    outlet_publishers = {}
    for outlet_num in range(num_outlets):
        outlet_publishers[f'outlet_{outlet_num}'] = rospy.Publisher(f'/digital_logger/outlet/{outlet_num}/status/', OutletStatus, queue_size=10)

    # Monitor outlets at 1Hz
    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        outlet_list = ",".join(map(str, range(num_outlets)))
        response = requests.get(f'http://{username}:{password}@{address}/restapi/relay/outlets/={outlet_list}/state/')

        result = json.loads(response.text)

        assert len(result) == num_outlets

        # publish the status of each outlet to its specific topic
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
