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
            outlet_num = int(outlet['outlet'])

    status = msg.status == 'on'

    outlet_endpoint = f'http://{address}/restapi/relay/outlets/{outlet_num}/state/'
    response = requests.put(outlet_endpoint, auth=requests.auth.HTTPDigestAuth(username, password), data={'value': str(status).lower()}, headers={"X-CSRF": "x", "Accept": "application/json"})
    rospy.loginfo(f'sent: {outlet_endpoint}, status={str(status).lower()}, received: code {response.status_code} : {response.text}')


def run_digital_logger():
    """
    Run the digital logger node. Publishes outlet statuses at 
    /digital_logger/outlets/{outlet num}/status. The outlets can be controlled by publishing a 
    OutletStatus message to /digital_logger/control. 
    """
    rospy.init_node('digital_logger')

    # subscribe to the digital logger control topic
    rospy.Subscriber('/digital_logger/control', OutletStatus, control_outlet)

    username = rospy.get_param('~username')
    password = rospy.get_param('~password')
    address = rospy.get_param('~address')
    outlets = rospy.get_param('~outlets')
    num_outlets = len(outlets)

    # create an independent publisher for each outlet
    outlet_publishers = []
    for outlet_num in range(num_outlets):
        outlet_publishers.append(rospy.Publisher(f'/digital_logger/outlet/{outlet_num}/status/', OutletStatus, queue_size=10))

    # Monitor outlets at 1Hz
    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        outlet_list = ",".join(map(str, range(num_outlets)))
        response = requests.get(f'http://{username}:{password}@{address}/restapi/relay/outlets/={outlet_list}/state/')

        result = json.loads(response.text)

        assert len(result) == num_outlets

        # publish the status of each outlet to its specific topic
        for outlet_index, outlet_result in enumerate(result):
            if outlet_result:
                status = 'on'
            else:
                status = 'off'

            outlet_status = OutletStatus()
            outlet_status.name = outlets[outlet_index]['name']
            outlet_status.status = status

            outlet_publishers[outlet_index].publish(outlet_status)

        rate.sleep()


if __name__ == '__main__':
    try:
        run_digital_logger()
    except rospy.ROSInterruptException:
        pass