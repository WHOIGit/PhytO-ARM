#!/usr/bin/env python3
"""
Functionality for monitoring and controlling a Digital Loggers Web Power Switch Pro model.
"""
import functools

import rospy

from typing import Dict

from std_msgs.msg import Bool

from api import DLIClient
from dli_power_switch.msg import OutletStatus


def control_outlet(client: DLIClient, number: int, msg: Bool) -> None:
    """
    Send the given msg to the digital logger as an HTTP request.
    """
    client.set_outlet_state(number - 1, msg.data)
    rospy.loginfo(f'Switched {"ON" if msg.data else "OFF"} outlet {number}')


def run_dli_power_switch_node() -> None:
    """
    Run the Digital Logger power switch node. Publishes outlet statuses at
    ~outlet/{name}/status. The outlets can be controlled by publishing a
    std_msgs/Bool message to ~outlet/{name}/control.
    """
    rospy.init_node('dli_power_switch')

    outlets = rospy.get_param('~outlets')

    client = DLIClient(
        rospy.get_param('~address'),
        rospy.get_param('~username'),
        rospy.get_param('~password')
    )

    # Set up publisher and subscriber for each outlet
    outlet_pubs: Dict[int, rospy.Publisher] = {}
    for outlet in outlets:
        name, num = outlet['name'], outlet['outlet']
        outlet_pubs[num] = rospy.Publisher(
            f'~outlet/{name}/status',
            OutletStatus,
            queue_size=10
        )
        rospy.Subscriber(
            f'~outlet/{name}/control',
            Bool,
            functools.partial(control_outlet, client, num)
        )

    # Monitor outlets at 1Hz
    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        # send a status request for each configured outlet
        for outlet in outlets:
            num = outlet['outlet']
            outlet_status = OutletStatus()
            outlet_status.is_active = client.get_outlet_state(num - 1)
            outlet_status.header.stamp = outlet_status.ds_header.io_time = \
                rospy.Time.now()
            outlet_pubs[num].publish(outlet_status)

        rate.sleep()


if __name__ == '__main__':
    try:
        run_dli_power_switch_node()
    except rospy.ROSInterruptException:
        pass
