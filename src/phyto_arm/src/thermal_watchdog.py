#!/usr/bin/env python3
'''
Watches the IFCB's housekeeping temperature/humidity sensor and powers the IFCB
off if it overheats. Once tripped, a lockout file keeps everything off until a
person removes it and restarts PhytO-ARM.

Shutdown sequence:
  1. Write the lockout file (so any restart honors it)
  2. Pump off
  3. Shut down the IFCB computer over SSH
  4. Wait, then cut IFCB power at the Digital Logger
'''
import json
import subprocess
import threading
import urllib.request

import rospy

from dli_power_switch.msg import OutletStatus
from sensor_msgs.msg import Temperature, RelativeHumidity
from std_msgs.msg import Bool

from thermal import ThermalMonitor, lockout_active, read_lockout, write_lockout


def send_alerts(text):
    for alert in rospy.get_param('/alerts', []):
        try:
            assert alert['type'] == 'slack' and alert['url']
            urllib.request.urlopen(
                alert['url'],
                json.dumps({'text': text}).encode(),
                timeout=10,
            )
        except Exception:
            rospy.logerr('Failed to send thermal shutdown alert', exc_info=True)


def ssh_shutdown(cfg):
    host = cfg.get('host') or rospy.get_param('/ifcb/address')
    cmd = ['ssh',
           '-o', 'BatchMode=yes',
           '-o', 'StrictHostKeyChecking=accept-new',
           '-o', 'ConnectTimeout=10']
    if cfg.get('key_file'):
        cmd += ['-i', cfg['key_file']]
    cmd += [f'{cfg["user"]}@{host}', cfg['command']]

    rospy.logwarn(f'Shutting down IFCB over SSH: {" ".join(cmd)}')
    try:
        # Power is cut regardless of the outcome; this only affects the alert.
        # An immediate shutdown may drop the connection and exit nonzero.
        result = subprocess.run(cmd, capture_output=True, text=True,
                                timeout=cfg.get('timeout', 30))
        rospy.logwarn(f'SSH shutdown exited {result.returncode}: '
                      f'{result.stdout.strip()} {result.stderr.strip()}')
        return result.returncode == 0
    except Exception:
        rospy.logerr('SSH shutdown failed', exc_info=True)
        return False


def shut_down_ifcb(cfg, lockout_file, reason, monitor):
    deployment = rospy.get_param('/name', '')
    rospy.logerr(f'THERMAL SHUTDOWN: {reason}')

    write_lockout(lockout_file, reason,
                  temperature=monitor.last_temperature,
                  humidity=monitor.last_humidity)

    send_alerts(f'*IFCB thermal shutdown*\n - Deployment: _{deployment}_\n'
                f' - Reason: {reason}\n - Last temperature: {monitor.last_temperature} C\n'
                f' - Last humidity: {monitor.last_humidity} %')

    outlets = rospy.get_param('outlets', {})
    pump_pub = rospy.Publisher(
        f'/digital_logger/outlet/{outlets.get("pump", "pump")}/control', Bool, queue_size=1, latch=True)
    ifcb_pub = rospy.Publisher(
        f'/digital_logger/outlet/{outlets.get("ifcb", "ifcb")}/control', Bool, queue_size=1, latch=True)

    # Give new publishers a moment to connect before sending
    rospy.sleep(1)
    pump_pub.publish(Bool(data=False))
    rospy.sleep(2)

    ssh_ok = ssh_shutdown(cfg['ssh'])

    delay = cfg['power_off_delay']
    rospy.logwarn(f'Cutting IFCB power in {delay} s')
    rospy.sleep(delay)
    ifcb_pub.publish(Bool(data=False))
    rospy.sleep(2)

    write_lockout(lockout_file, reason, completed=True, ssh_shutdown_ok=ssh_ok,
                  temperature=monitor.last_temperature,
                  humidity=monitor.last_humidity)
    rospy.logerr(f'IFCB powered off. Remove {lockout_file} and restart PhytO-ARM to resume.')
    send_alerts(f'*IFCB powered off* (_{deployment}_). SSH shutdown '
                f'{"succeeded" if ssh_ok else "did not report success"}. Manual restart required.')


def main():
    rospy.init_node('thermal_watchdog')

    cfg = rospy.get_param('thermal_shutdown', {})
    if not cfg.get('enabled', False):
        rospy.logwarn('Thermal shutdown disabled')
        return

    lockout_file = cfg['lockout_file']
    lockout_pub = rospy.Publisher('~lockout', Bool, queue_size=1, latch=True)

    # Already locked out from a previous run. If the shutdown sequence did not
    # finish (e.g. PhytO-ARM restarted partway through), finish it now.
    if lockout_active(lockout_file):
        record = read_lockout(lockout_file)
        lockout_pub.publish(Bool(data=True))
        rospy.logerr(f'Thermal lockout active since {record.get("time")}: {record.get("reason")}')
        if not record.get('completed', False):
            rospy.logwarn('Previous thermal shutdown did not complete; resuming it')
            monitor = ThermalMonitor(0, 0, 0, 0)
            shut_down_ifcb(cfg, lockout_file, record.get('reason', 'unknown'), monitor)
        rospy.spin()
        return

    lockout_pub.publish(Bool(data=False))

    now = rospy.get_time()
    lo, hi = cfg.get('valid_temperature_range', [-20.0, 85.0])
    monitor = ThermalMonitor(
        max_temperature=cfg['max_temperature'],
        max_humidity=cfg.get('max_humidity'),
        trip_duration=cfg['trip_duration'],
        min_readings=cfg['min_readings'],
        stale_timeout=cfg['stale_timeout'],
        valid_range=(lo, hi),
        now=now,
    )

    # The IFCB node publishes temperature and humidity separately, but both
    # come from the same reply, so pair them up by timestamp.
    pending = {}
    pending_lock = threading.Lock()

    def on_temperature(msg):
        with pending_lock:
            pending['temperature'] = (msg.header.stamp, msg.temperature)
            flush()

    def on_humidity(msg):
        with pending_lock:
            pending['humidity'] = (msg.header.stamp, msg.relative_humidity * 100.0)
            flush()

    def flush():
        if 'temperature' in pending and 'humidity' in pending and \
           pending['temperature'][0] == pending['humidity'][0]:
            temperature = pending.pop('temperature')[1]
            humidity = pending.pop('humidity')[1]
            rospy.loginfo(f'IFCB environment: {temperature:.2f} C, {humidity:.1f} %')
            monitor.reading(temperature, humidity, rospy.get_time())

    def on_outlet(msg):
        monitor.set_powered(msg.is_active, rospy.get_time())

    ifcb_outlet = rospy.get_param('outlets', {}).get('ifcb', 'ifcb')
    rospy.Subscriber('/ifcb/temperature', Temperature, on_temperature)
    rospy.Subscriber('/ifcb/humidity', RelativeHumidity, on_humidity)
    rospy.Subscriber(f'/digital_logger/outlet/{ifcb_outlet}/status', OutletStatus, on_outlet)

    if monitor.stale_timeout > 0 and rospy.get_param('/ifcb/environment_poll_interval', 0) <= 0:
        rospy.logerr('ifcb/environment_poll_interval is not set; with no readings the '
                     'stale_timeout check will shut down the IFCB')

    rospy.loginfo(f'Thermal watchdog armed: max {monitor.max_temperature} C '
                  f'for {monitor.trip_duration} s')

    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        reason = monitor.check(rospy.get_time())
        if reason is not None:
            lockout_pub.publish(Bool(data=True))
            shut_down_ifcb(cfg, lockout_file, reason, monitor)
            break
        rate.sleep()

    # Stay up so the latched lockout status remains visible
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
