#!/usr/bin/env python3
"""
Functionality for monitoring and controlling a Digital Loggers Web Power Switch Pro model.
"""
import base64
import urllib.request

import rospy

from dli_power_switch.msg import OutletStatus

# Global variables relevant to digital logger control. These variables are defined once the digital
# logger node is initialized.
AUTH = ''
ADDRESS = ''
OUTLETS = ''
OUTLET_NAMES = ''


def control_outlet(msg):
    """
    Send the given msg to the digital logger as an HTTP request.
    """

    outlet_num = OUTLET_NAMES.get(msg.name)

    data = f'value={str(msg.is_active).lower()}'
    url = f'http://{ADDRESS}/restapi/relay/outlets/{outlet_num}/state/'

    # Create a PUT request
    req = urllib.request.Request(url, data=data.encode("utf-8"), method="PUT")
    req.add_header("Authorization", AUTH)
    req.add_header("X-CSRF", 'x')

    try:
        # Send the PUT request
        response = urllib.request.urlopen(req)
    except urllib.error.HTTPError as http_err:
        raise ValueError(f"HTTP Error: {http_err.code} : {http_err.reason}") from http_err
    except urllib.error.URLError as url_err:
        raise ValueError(f"URL Error: {url_err.reason}") from url_err

    result = response.read().decode('utf-8')

    rospy.loginfo(f'sent status={str(msg.is_active).lower()} to {ADDRESS}:{url}, received: code {response.status} : {result}')


def run_dli_power_switch_node():
    """
    Run the Digital Logger power switch node. Publishes outlet statuses at
    /digital_logger/OUTLETS/{outlet num}/status. The OUTLETS can be controlled by publishing a
    OutletStatus message to /digital_logger/control.
    """
    global AUTH, ADDRESS, OUTLETS, OUTLET_NAMES

    rospy.init_node('dli_power_switch')

    username = rospy.get_param('~username')
    password = rospy.get_param('~password')
    AUTH = f"Basic {base64.b64encode(f'{username}:{password}'.encode()).decode()}"
    ADDRESS = rospy.get_param('~address')
    OUTLETS = rospy.get_param('~outlets')
    OUTLET_NAMES = {outlet['name']: int(outlet['outlet']) for outlet in OUTLETS}

    # subscribe to the digital logger control topic
    rospy.Subscriber('/digital_logger/control', OutletStatus, control_outlet)

    outlet_publishers = []
    for outlet_num, _ in enumerate(OUTLETS):
        outlet_publishers.append(rospy.Publisher(f'/digital_logger/outlet/{outlet_num}/status/', OutletStatus, queue_size=10))

    # Monitor outlets at 1Hz
    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        # send a status request for each available outlet
        for outlet_index, _ in enumerate(OUTLETS):
            # Construct request
            url = f'http://{ADDRESS}/restapi/relay/outlets/{outlet_index}/state/'
            request = urllib.request.Request(url)
            request.add_header("Authorization", AUTH)

            try:
                # Send the GET request
                with urllib.request.urlopen(request) as response:
                    # Read and decode the response
                    response_data = response.read().decode('utf-8')
            except urllib.error.HTTPError as http_err:
                raise ValueError(f"HTTP Error: {http_err.code} : {http_err.reason}") from http_err
            except urllib.error.URLError as url_err:
                raise ValueError(f"URL Error: {url_err.reason}") from url_err

            # publish the status of each outlet to its specific topic
            outlet_status = OutletStatus()
            outlet_status.name = OUTLETS[outlet_index]['name']
            # DL API uses 'true' and 'false' to denote outlet status, which need to be converted to Python booleans
            outlet_status.is_active = response_data == 'true'

            outlet_publishers[outlet_index].publish(outlet_status)

        rate.sleep()


if __name__ == '__main__':
    try:
        run_dli_power_switch_node()
    except rospy.ROSInterruptException:
        pass
