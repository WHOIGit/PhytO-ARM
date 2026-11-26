#!/usr/bin/env python3
import datetime
import json
import urllib.request

import rclpy

from phyto_sim_utils.parameters import declare_matching_parameters, declare_number
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from seatrac.msg import OutletStatus, PowerLevel
from std_msgs.msg import Bool as StdBool


def send_alerts(alert_config, message):
    for alert in alert_config:
        assert alert['type'] == 'slack' and alert['url']
        urllib.request.urlopen(
            alert['url'],
            json.dumps({'text': message}).encode()
        )


def configure_node(node: Node) -> None:
    # Read parameters
    entries = {}
    for name, parameter in node.get_parameters_by_prefix('alerts').items():
        index, key = name.split('.', 1)
        entries.setdefault(int(index), {})[key] = parameter.value
    alerts = [entries[i] for i in sorted(entries)]
    thresholds = {key: param.value for key, param in node.get_parameters_by_prefix('soc_threshold').items()}
    high_threshold = thresholds.get('high', 95)
    medium_threshold = thresholds.get('medium', 80)

    shutdown_reminder = node.get_parameter_or('shutdown_reminder').value
    scheduled_on = node.get_parameter_or('scheduled_on').value
    if scheduled_on is not None:
        scheduled_on = datetime.datetime.strptime(scheduled_on, '%H:%M').time()


    # Create a publisher for controlling the Starlink outlet
    outlet_pub = node.create_publisher(StdBool, '/seatrac/outlet/starlink/control', 1)


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
            node.get_logger().info(
                f'Enabling Starlink: SOC {soc}% >= {high_threshold}% and charging'
            )
            outlet_pub.publish(StdBool(data=True))
            already_sent = True
        elif soc < medium_threshold:
            already_sent = False

    node.create_subscription(PowerLevel, '/seatrac/power', power_level_cb, 10,
                             callback_group=MutuallyExclusiveCallbackGroup())


    # Nag the operators if Starlink has been active for too long
    nag_timer = None

    def one_shot(seconds, callback):
        def fire():
            try:
                callback()
            except Exception as err:
                node.get_logger().error(f'Starlink timer callback failed: {err}')
                return
            node.destroy_timer(timer)

        timer = node.create_timer(seconds, fire)
        return timer

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
            nag_timer = one_shot(shutdown_reminder, nag_operator)
        elif not msg.is_active and nag_timer:
            node.destroy_timer(nag_timer)
            nag_timer = None

    node.create_subscription(OutletStatus, '/seatrac/outlet/starlink/status', outlet_status_cb, 10,
                             callback_group=MutuallyExclusiveCallbackGroup())


    # Schedule Starlink to turn on at a specific time every day
    def schedule_daily_timer():
        now = datetime.datetime.now(datetime.timezone.utc)
        target = now.replace(hour=scheduled_on.hour,
                             minute=scheduled_on.minute,
                             second=0, microsecond=0)
        if target <= now:
            target += datetime.timedelta(days=1)

        one_shot((target - now).total_seconds(), daily_timer_cb)

    def daily_timer_cb():
        nonlocal already_sent

        # Starlink is already enabled, no need to enable it again. We don't
        # check already_sent because the operator may have manually turned it
        # off yesterday, and we want to turn it on today.
        if last_state:
            return

        if soc >= medium_threshold:
            node.get_logger().info(
                f'Enabling Starlink: SOC {soc:.1f}% >= {medium_threshold}% '
                'at scheduled time'
            )
            outlet_pub.publish(StdBool(data=True))
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



def main():
    rclpy.init()
    node = rclpy.create_node('seatrac_starlink_manager')
    executor = MultiThreadedExecutor()
    try:
        declare_matching_parameters(node, r'alerts\.[0-9]+\.(type|url)', Parameter.Type.STRING)
        declare_number(node, 'soc_threshold.high', 95)
        declare_number(node, 'soc_threshold.medium', 80)
        declare_number(node, 'shutdown_reminder')
        node.declare_parameter('scheduled_on', Parameter.Type.STRING)

        configure_node(node)
        rclpy.spin(node, executor=executor)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
