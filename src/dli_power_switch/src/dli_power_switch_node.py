#!/usr/bin/env python3
"""
Functionality for monitoring and controlling a Digital Loggers Web Power Switch Pro model.
"""
import functools
from typing import Dict, Optional

import rospy

from std_msgs.msg import Bool

from api import DLIClient
from dli_power_switch.msg import OutletStatus
from scheduling import should_outlet_be_on


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

    # Dictionary of outlets to last scheduled state. Using this instead of polled outlet status because 
    # 1) there's no atomicity with network calls and we could fire redundant commands, and
    # 2) more importantly, users may manually switch DL values and we don't want to fight
    #    them by resending commands when state doesn't match schedule.
    last_scheduled_states: Dict[int, Optional[bool]] = {}

    # Monitor outlets at 1Hz
    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        # send a status request for each configured outlet
        for outlet in outlets:
            num = outlet['outlet']
            name = outlet['name']

            # Load config every iteration in case config is updated at runtime
            switch_on_times = outlet.get('switch_on_times', [])
            switch_off_times = outlet.get('switch_off_times', [])

            if switch_on_times or switch_off_times:
                scheduled_state = should_outlet_be_on(switch_on_times, switch_off_times)

                if scheduled_state is not None:
                    # Only act if the scheduled state has changed
                    if num not in last_scheduled_states or last_scheduled_states[num] != scheduled_state:
                        client.set_outlet_state(num - 1, scheduled_state)
                        last_scheduled_states[num] = scheduled_state
                        rospy.loginfo(f'Scheduled switch {"ON" if scheduled_state else "OFF"} outlet {num} ({name})')

            # Publish current status
            outlet_status = OutletStatus()
            outlet_status.is_active = client.get_outlet_state(num - 1)
            outlet_status.header.stamp = outlet_status.ds_header.io_time = \
                rospy.Time.now()
            outlet_pubs[num].publish(outlet_status)

            if scheduled_state and outlet_status.is_active != scheduled_state:
                expected = "on" if scheduled_state else "off"
                actual = "on" if outlet_status.is_active else "off"
                rospy.logwarn(f"Outlet {num} scheduled to be {expected} but is {actual}")

        rate.sleep()


if __name__ == '__main__':
    try:
        run_dli_power_switch_node()
    except rospy.ROSInterruptException:
        pass
