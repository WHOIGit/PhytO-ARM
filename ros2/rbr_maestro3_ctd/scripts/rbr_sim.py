#!/usr/bin/env python3
"""Emit a Maestro3 CSV record for coincident simulated CTD and depth readings."""
import datetime
import math

import message_filters
import rclpy

from ds_sensor_msgs.msg import Ctd, DepthPressure
from rclpy.executors import ExternalShutdownException
from wr2_msgs.msg import RawData


def main():
    rclpy.init()
    node = rclpy.create_node('rbr_sim')

    def publish_record(ctd, depth):
        stamp = ctd.header.stamp
        when = datetime.datetime.fromtimestamp(stamp.sec, datetime.timezone.utc)
        when = when.replace(microsecond=stamp.nanosec // 1000)

        # Above the water surface, the model marks conductivity and temperature as missing.
        conductivity = math.nan if ctd.conductivity == Ctd.CTD_NO_DATA else ctd.conductivity
        temperature = math.nan if ctd.temperature == Ctd.CTD_NO_DATA else ctd.temperature

        # Convert conductivity from S/m to mS/cm for the Maestro3 record.
        conductivity_ms_per_cm = conductivity * 10
        record = (f'{when:%Y-%m-%d %H:%M:%S.%f}, {conductivity_ms_per_cm:.6f}, '
                  f'{temperature:.6f}, {depth.pressure:.6f}, {depth.depth:.6f}\r\n')

        msg = RawData(io_stamp=ctd.ds_header.io_time)
        msg.data_direction = RawData.DIRECTION_DATA_IN
        msg.data = record.encode('ascii')
        publisher.publish(msg)

    try:
        publisher = node.create_publisher(RawData, 'sim/rbr', 5)

        ctd = message_filters.Subscriber(node, Ctd, 'sim/ctd')
        depth = message_filters.Subscriber(node, DepthPressure, 'sim/depth')
        pairs = message_filters.TimeSynchronizer([ctd, depth], 5)
        pairs.registerCallback(publish_record)
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
