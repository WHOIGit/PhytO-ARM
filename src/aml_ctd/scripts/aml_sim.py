#!/usr/bin/env python3
from itertools import count

import message_filters
import rclpy

from ds_sensor_msgs.msg import Ctd, DepthPressure
from wr2_msgs.msg import RawData

from aml_ctd.amlxparser import checksum


# Encode a monitor record using the port layout from the AMLx capture
def monitor_record(number, ctd, depth):
    stamp = ctd.header.stamp
    fields = [f'mux[meta=time,{stamp.sec}.{stamp.nanosec:09d},s]']
    derived = f'derive[data=Depth,{depth.depth:.6f},m]'

    # The conductivity and temperature probes only measure submerged water
    if ctd.conductivity != Ctd.CTD_NO_DATA:
        fields.append(f'port1[data=Cond,{ctd.conductivity * 10:.6f},mS/cm]'
                      f'[rawi=ADC,0,none][data=TempCT,{ctd.temperature:.6f},C]'
                      '[rawi=ADC,0,none]')
        derived += (f'[data=SV,{ctd.sound_speed:.6f},m/s]'
                    f'[data=SalC,{ctd.salinity:.6f},psu]')

    # Pressure and depth are reported even when the probe is in air
    fields.append(f'port6[data=Pressure,{depth.pressure:.6f},dBar]'
                  '[rawi=ADC,0,2sComp]')
    fields.append(derived)
    record = 'msg%d{%s}' % (number, ','.join(fields))
    return f'{record}*{checksum(record):02X}\r\n'.encode('ascii')


def main():
    rclpy.init()
    node = rclpy.create_node('aml_sim')
    pub = node.create_publisher(RawData, 'sim/aml', 5)
    sequence = count()

    def on_sample(ctd, depth):
        msg = RawData(io_stamp=ctd.ds_header.io_time)
        msg.data_direction = RawData.DIRECTION_DATA_IN
        msg.data = monitor_record(next(sequence), ctd, depth)
        pub.publish(msg)

    # Pair the measurements taken at the same probe position observation
    ctd = message_filters.Subscriber(node, Ctd, 'sim/ctd')
    depth = message_filters.Subscriber(node, DepthPressure, 'sim/depth')
    samples = message_filters.TimeSynchronizer([ctd, depth], 5)
    samples.registerCallback(on_sample)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
