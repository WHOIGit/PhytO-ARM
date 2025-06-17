#!/usr/bin/env python3
"""
Functionality for monitoring and controlling a Digital Loggers Web Power Switch Pro model.
"""
import base64
import urllib.request

import rospy

from std_msgs.msg import Bool

from dli_power_switch.msg import OutletStatus


# Global variables relevant to digital logger control. These variables are defined once the digital
# logger node is initialized.
AUTH = ''
ADDRESS = ''
OUTLETS = ''


def control_outlet(msg, number):
    """
    Send the given msg to the digital logger as an HTTP request.
    """
    # Create a PUT request
    url = f'http://{ADDRESS}/restapi/relay/outlets/{number}/state/'
    req = urllib.request.Request(
        url,
        data=f'value={str(msg.data).lower()}'.encode(),
        method='PUT',
        headers={
            'Authorization': AUTH,
            'X-CSRF': 'x',
        }
    )

    try:
        # Send the PUT request
        response = urllib.request.urlopen(req)
    except urllib.error.HTTPError as http_err:
        raise ValueError(f"HTTP Error: {http_err.code} : {http_err.reason}") from http_err
    except urllib.error.URLError as url_err:
        raise ValueError(f"URL Error: {url_err.reason}") from url_err

    result = response.read().decode('utf-8')

    rospy.loginfo(f'sent status={str(msg.data).lower()} to {ADDRESS}:{url}, '
                  f'received: code {response.status} : {result}')


def run_dli_power_switch_node():
    """
    Run the Digital Logger power switch node. Publishes outlet statuses at
    ~outlet/{name}/status. The outlets can be controlled by publishing a
    std_msgs/Bool message to ~outlet/{name}/control.
    """
    global AUTH, ADDRESS, OUTLETS

    rospy.init_node('dli_power_switch')

    username = rospy.get_param('~username')
    password = rospy.get_param('~password')
    AUTH = f"Basic {base64.b64encode(f'{username}:{password}'.encode()).decode()}"
    ADDRESS = rospy.get_param('~address')
    OUTLETS = rospy.get_param('~outlets')

    # Set up publisher and subscriber for each outlet
    outlet_pubs = {}
    for outlet in OUTLETS:
        name, num = outlet['name'], outlet['outlet']
        outlet_pubs[num] = rospy.Publisher(
            f'~outlet/{name}/status',
            OutletStatus,
            queue_size=10
        )
        rospy.Subscriber(f'~outlet/{name}/control', Bool, control_outlet, num)

    # Monitor outlets at 1Hz
    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        # send a status request for each configured outlet
        for outlet in OUTLETS:
            num = outlet['outlet']
            url = f'http://{ADDRESS}/restapi/relay/outlets/{num}/state/'
            request = urllib.request.Request(url)
            request.add_header("Authorization", AUTH)

            try:
                with urllib.request.urlopen(request) as response:
                    response_data = response.read().decode('utf-8')
            except urllib.error.HTTPError as http_err:
                raise ValueError(f"HTTP Error: {http_err.code} : {http_err.reason}") from http_err
            except urllib.error.URLError as url_err:
                raise ValueError(f"URL Error: {url_err.reason}") from url_err

            # publish the status with timestamps and active flag
            outlet_status = OutletStatus()
            outlet_status.header.stamp = outlet_status.ds_header.io_time = \
                rospy.Time.now()
            outlet_status.is_active = response_data == 'true'
            outlet_pubs[num].publish(outlet_status)

        rate.sleep()


if __name__ == '__main__':
    try:
        run_dli_power_switch_node()
    except rospy.ROSInterruptException:
        pass
