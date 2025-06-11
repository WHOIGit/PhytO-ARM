#!/usr/bin/env python3
import json
import urllib.request

import rospy

from seatrac.msg import PowerLevel
from std_msgs.msg import Bool as StdBool

from phyto_arm.msg import OutletStatus


def send_alerts(alert_config, message):
    for alert in alert_config:
        assert alert['type'] == 'slack' and alert['url']
        urllib.request.urlopen(
            alert['url'],
            json.dumps({'text': message}).encode()
        )


def main():
    rospy.init_node('seatrac_starlink_manager')
    shutdown_reminder = rospy.get_param('~shutdown_reminder', None)
    force_on_threshold = rospy.get_param('~soc_force_threshold', 95)
    on_threshold = rospy.get_param('~soc_threshold', 80)
    off_delay = rospy.get_param('~off_delay', 86400)
    alerts = rospy.get_param('/alerts', [])

    outlet_pub = rospy.Publisher('/seatrac/outlet/starlink/control', StdBool,
                                 queue_size=1)

    manual_shutdown = False
    last_off = None
    nag_timer = None

    enable_sent = False
    last_state = None
    soc = 0.0

    def nag_operator() -> None:
        nonlocal alerts, nag_timer
        nag_timer = None
        send_alerts(alerts, 'Starlink has been active for a while')

    def outlet_status_cb(msg: OutletStatus) -> None:
        nonlocal manual_shutdown, last_off, nag_timer, last_state
        now = rospy.Time.now().to_sec()
        if last_state is None:
            if not msg.is_active:
                manual_shutdown = True
                last_off = now
        elif last_state and not msg.is_active:
            manual_shutdown = True
            last_off = now
        elif manual_shutdown and msg.is_active:
            manual_shutdown = False
        last_state = msg.is_active

        if shutdown_reminder is not None and last_state and nag_timer is None:
            nag_timer = rospy.Timer(
                rospy.Duration(shutdown_reminder),
                nag_operator,
                oneshot=True
            )
        elif not msg.is_active and nag_timer:
            nag_timer.shutdown()
            nag_timer = None

    def power_level_cb(msg: PowerLevel) -> None:
        nonlocal enable_sent, last_state, soc, manual_shutdown, last_off
        if last_state is None:
            return

        soc = msg.soc_percentage
        is_charging = msg.pack_current < 0
        now = rospy.Time.now().to_sec()

        if (
            soc >= force_on_threshold
            and is_charging
            and not enable_sent
            and not manual_shutdown
        ):
            rospy.loginfo(
                f'Enabling Starlink: SOC {soc}% > {force_on_threshold} and charging'
            )
            outlet_pub.publish(StdBool(True))
            enable_sent = True
        elif (
            soc >= on_threshold
            and not enable_sent
            and not manual_shutdown
            and (last_off is None or now - last_off > off_delay)
        ):
            rospy.loginfo(
                f'Enabling Starlink: SOC {soc}% > {on_threshold}'
                f' and last_off {int(now-last_off)}s > {off_delay}'
            )
            outlet_pub.publish(StdBool(True))
            enable_sent = True
        elif soc < on_threshold:
            enable_sent = False

    rospy.Subscriber('/seatrac/outlet/starlink/status', OutletStatus,
                     outlet_status_cb)
    rospy.Subscriber('/seatrac/power', PowerLevel, power_level_cb)

    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
