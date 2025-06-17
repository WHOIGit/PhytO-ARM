#!/usr/bin/env python3
"""
Functionality for monitoring and controlling a Digital Loggers Web Power Switch Pro model.
"""
import urllib.request

import rospy

from std_msgs.msg import Bool

from dli_power_switch.msg import OutletStatus

from typing import Any, Dict, List, Optional


# Global variables set at node initialization
ADDRESS: str = ''
OUTLETS: List[Dict[str, Any]] = []
OPENER: Optional[urllib.request.OpenerDirector] = None


# Configure a urllib opener for HTTP Digest authentication
def configure_digest_opener(username: str, password: str) -> None:
    global OPENER
    pwd_mgr = urllib.request.HTTPPasswordMgrWithDefaultRealm()
    pwd_mgr.add_password(None, f'http://{ADDRESS}/', username, password)
    handler = urllib.request.HTTPDigestAuthHandler(pwd_mgr)
    OPENER = urllib.request.build_opener(handler)


def set_outlet_state(number: int, is_active: bool) -> None:
    OPENER.open(urllib.request.Request(
        f'http://{ADDRESS}/restapi/relay/outlets/{number}/state/',
        data=f'value={str(is_active).lower()}'.encode(),
        method='PUT',
        headers={
            'X-CSRF': 'x',
        }
    ))


def get_outlet_state(number: int) -> bool:
    response = OPENER.open(urllib.request.Request(
        f'http://{ADDRESS}/restapi/relay/outlets/{number}/state/'
    ))
    return response.read() == b'true'


def control_outlet(msg: Bool, number: int) -> None:
    """
    Send the given msg to the digital logger as an HTTP request.
    """
    set_outlet_state(number, msg.data)
    rospy.loginfo(f'Switched {"ON" if msg.data else "OFF"} outlet {number}')


def run_dli_power_switch_node() -> None:
    """
    Run the Digital Logger power switch node. Publishes outlet statuses at
    ~outlet/{name}/status. The outlets can be controlled by publishing a
    std_msgs/Bool message to ~outlet/{name}/control.
    """
    global ADDRESS, OUTLETS

    rospy.init_node('dli_power_switch')

    ADDRESS = rospy.get_param('~address')
    OUTLETS = rospy.get_param('~outlets')

    configure_digest_opener(
        rospy.get_param('~username'),
        rospy.get_param('~password')
    )

    # Set up publisher and subscriber for each outlet
    outlet_pubs: Dict[int, rospy.Publisher] = {}
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
            outlet_status = OutletStatus()
            outlet_status.is_active = get_outlet_state(num)
            outlet_status.header.stamp = outlet_status.ds_header.io_time = \
                rospy.Time.now()
            outlet_pubs[num].publish(outlet_status)

        rate.sleep()


if __name__ == '__main__':
    try:
        run_dli_power_switch_node()
    except rospy.ROSInterruptException:
        pass
