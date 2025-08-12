#!/usr/bin/env python3
import datetime
import json
import urllib.request

import rospy

from seatrac.msg import PowerLevel, OutletStatus
from std_msgs.msg import Bool as StdBool


def send_alerts(alert_config, message):
    for alert in alert_config:
        assert alert['type'] == 'slack' and alert['url']
        urllib.request.urlopen(
            alert['url'],
            json.dumps({'text': message}).encode()
        )


def main():
    rospy.init_node('seatrac_starlink_manager')

    # Read parameters
    alerts = rospy.get_param('/alerts', [])
    high_threshold = rospy.get_param('~soc_threshold', {}).get('high', 95)
    medium_threshold = rospy.get_param('~soc_threshold', {}).get('medium', 80)

    shutdown_reminder  = rospy.get_param('~shutdown_reminder', None)
    if shutdown_reminder is not None:
        shutdown_reminder = rospy.Duration(shutdown_reminder)

    scheduled_on = rospy.get_param('~scheduled_on', None)
    if scheduled_on is not None:
        scheduled_on = datetime.datetime.strptime(scheduled_on, '%H:%M').time()


    # Create a publisher for controlling the Starlink outlet
    outlet_pub = rospy.Publisher('/seatrac/outlet/starlink/control', StdBool,
                                 queue_size=1)


    # Subscribe to power level messages and record the state of charge, and
    # turn Starlink on automatically if the SOC is high enough.
    charging = False

    already_sent = False
    last_state = None
    soc = 0.0

    def power_level_cb(msg: PowerLevel) -> None:
        nonlocal already_sent, charging, soc
        if last_state is None:
            return

        soc = msg.soc_percentage
        charging = msg.pack_current < 0

        if soc >= high_threshold and charging and \
            not last_state and not already_sent:
            rospy.loginfo(
                f'Enabling Starlink: SOC {soc}% >= {high_threshold}% and charging'
            )
            outlet_pub.publish(StdBool(True))
            already_sent = True
        elif soc < medium_threshold:
            already_sent = False

    rospy.Subscriber('/seatrac/power', PowerLevel, power_level_cb)


    # Nag the operators if Starlink has been active for too long
    nag_timer = None

    def nag_operator() -> None:
        nonlocal nag_timer
        nag_timer = None
        send_alerts(alerts, 'Starlink has been active for a while')

    def outlet_status_cb(msg: OutletStatus) -> None:
        nonlocal nag_timer, last_state
        last_state = msg.is_active

        # Set up the nag timer if Starlink is active (except if we are at a
        # high state of charge and charging). When the outlet is off, cancel
        # the nag timer.
        if shutdown_reminder is None:
            return

        if msg.is_active and nag_timer is None and \
           not (soc >= high_threshold and charging):
            # Assume we'll be getting outlet status messages regularly, so we
            # can do a one-shot timer.
            nag_timer = rospy.Timer(
                shutdown_reminder,
                nag_operator,
                oneshot=True
            )
        elif not msg.is_active and nag_timer:
            nag_timer.shutdown()
            nag_timer = None

    rospy.Subscriber('/seatrac/outlet/starlink/status', OutletStatus,
                     outlet_status_cb)


    # Schedule Starlink to turn on at a specific time every day
    daily_timer = None

    def schedule_daily_timer():
        nonlocal daily_timer
        now = datetime.datetime.now(datetime.timezone.utc)
        target = now.replace(hour=scheduled_on.hour,
                             minute=scheduled_on.minute,
                             second=0, microsecond=0)
        if target <= now:
            target += datetime.timedelta(days=1)

        daily_timer = rospy.Timer(
            rospy.Duration((target - now).total_seconds()),
            daily_timer_cb,
            oneshot=True
        )

    def daily_timer_cb(event):
        nonlocal already_sent

        # Starlink is already enabled, no need to enable it again. We don't
        # check already_sent because the operator may have manually turned it
        # off yesterday, and we want to turn it on today.
        if last_state:
            return

        if soc >= medium_threshold:
            rospy.loginfo(
                f'Enabling Starlink: SOC {soc:.1f}% >= {medium_threshold}% '
                'at scheduled time'
            )
            outlet_pub.publish(StdBool(True))
            already_sent = True
        elif soc < medium_threshold:
            send_alerts(
                alerts,
                'Not enabling Starlink at scheduled time because '
                f'SOC {soc:.1f}% below medium threshold {medium_threshold}%'
            )
        schedule_daily_timer()

    if scheduled_on is not None:
        schedule_daily_timer()

    rospy.spin()


if __name__ == '__main__':
    main()
