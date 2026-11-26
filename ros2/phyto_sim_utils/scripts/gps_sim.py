#!/usr/bin/env python3
import socket
import subprocess

import rospy

from datetime import datetime, timedelta, timezone
from functools import reduce
from operator import xor

from phyto_sim_utils.gps import receive_fixes
from rospy.impl.node import _get_node
from sensor_msgs.msg import NavSatStatus


# NMEA coordinates use whole degrees followed by decimal minutes
def coordinate(value, width):
    minutes = round(abs(value) * 60, 5)
    degrees, minutes = divmod(minutes, 60)
    return f'{int(degrees):0{width}d}{minutes:08.5f}'


def sentence(body):
    data = body.encode('ascii')
    return b'$%s*%02X\r\n' % (data, reduce(xor, data))


# Report time, position and altitude for the GPS antenna on the fixed barge
def nmea_fix(fix, utc):
    stamp = utc.strftime('%H%M%S.%f')[:-3]
    if fix.status.status == NavSatStatus.STATUS_NO_FIX:
        return (sentence(f'GPRMC,{stamp},V,,,,,,,{utc:%d%m%y},,,N')
                + sentence(f'GPGGA,{stamp},,,,,0,00,,,,,,,'))

    lat = coordinate(fix.latitude, 2)
    lon = coordinate(fix.longitude, 3)
    ns = 'N' if fix.latitude >= 0 else 'S'
    ew = 'E' if fix.longitude >= 0 else 'W'
    position = f'{lat},{ns},{lon},{ew}'
    rmc = f'GPRMC,{stamp},A,{position},0.0,0.0,{utc:%d%m%y},,,A'
    gga = f'GPGGA,{stamp},{position},1,08,1.0,{fix.altitude:.3f},M,0.0,M,,'
    return sentence(rmc) + sentence(gga)


def main():
    rospy.init_node('gps_sim')
    port = rospy.get_param('~port')
    address = ('127.0.0.1', port)
    epoch = None

    # Anchor receiver UTC at the first fix, then advance on ROS time
    def on_fix(fix):
        nonlocal epoch
        elapsed = timedelta(seconds=fix.header.stamp.to_sec())
        if epoch is None:
            epoch = datetime.now(timezone.utc) - elapsed
        receiver.sendto(nmea_fix(fix, epoch + elapsed), address)

    # gpsd reads NMEA over UDP and serves clients over TCP on the same port
    device = f'udp://127.0.0.1:{port}'
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as receiver:
        with subprocess.Popen(['gpsd', '-N', '-n', '-b', '-S', str(port),
                               device]) as daemon:
            rospy.on_shutdown(daemon.terminate)
            receive_fixes(_get_node(), on_fix)
            daemon.wait()
            if not rospy.is_shutdown():
                raise RuntimeError('gpsd exited while the simulator was active')


if __name__ == '__main__':
    main()
