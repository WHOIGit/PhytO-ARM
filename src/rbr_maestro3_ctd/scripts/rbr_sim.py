#!/usr/bin/env python3
"""Emit a Maestro3 CSV record for coincident simulated CTD and depth readings."""
import datetime
import math

import message_filters
import rospy
from ds_core_msgs.msg import RawData
from ds_sensor_msgs.msg import Ctd, DepthPressure


def main():
    rospy.init_node('rbr_sim')
    publisher = rospy.Publisher('sim/rbr', RawData, queue_size=5)

    def publish_record(ctd, depth):
        stamp = ctd.header.stamp
        when = datetime.datetime.fromtimestamp(stamp.secs, datetime.timezone.utc)
        when = when.replace(microsecond=stamp.nsecs // 1000)

        # Above the water surface, the model marks conductivity and temperature as missing.
        conductivity = math.nan if ctd.conductivity == Ctd.CTD_NO_DATA else ctd.conductivity
        temperature = math.nan if ctd.temperature == Ctd.CTD_NO_DATA else ctd.temperature

        # Convert conductivity from S/m to mS/cm for the Maestro3 record.
        conductivity_ms_per_cm = conductivity * 10
        record = (f'{when:%Y-%m-%d %H:%M:%S.%f}, {conductivity_ms_per_cm:.6f}, '
                  f'{temperature:.6f}, {depth.pressure:.6f}, {depth.depth:.6f}\r\n')

        msg = RawData(header=ctd.header, ds_header=ctd.ds_header)
        msg.data_direction = RawData.DATA_IN
        msg.data = record.encode('ascii')
        publisher.publish(msg)

    ctd = message_filters.Subscriber('sim/ctd', Ctd)
    depth = message_filters.Subscriber('sim/depth', DepthPressure)
    pairs = message_filters.TimeSynchronizer([ctd, depth], 5)
    pairs.registerCallback(publish_record)
    rospy.spin()


if __name__ == '__main__':
    main()
