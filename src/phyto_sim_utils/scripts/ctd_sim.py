#!/usr/bin/env python3
import math

import rospy

from ds_sensor_msgs.msg import Ctd, DepthPressure
from geometry_msgs.msg import PointStamped


# Sample a synthetic shallow water column at the probe observation time
def sample_ctd(position, surface_z):
    ctd = Ctd()
    depth = DepthPressure()
    for msg in (ctd, depth):
        msg.header.frame_id = position.header.frame_id
        msg.header.stamp = position.header.stamp
        msg.ds_header.io_time = position.header.stamp

    # Hydrostatic sea pressure in dbar at 1025 kg/m³ and standard gravity
    immersion = surface_z - position.point.z
    depth.depth = max(0.0, immersion)
    depth.pressure = 1025 * 9.80665 * depth.depth / 10000
    depth.pressure_raw = depth.pressure
    depth.pressure_raw_unit = DepthPressure.UNIT_PRESSURE_DBAR
    depth.pressure_covar = -1
    depth.tare = 0.0
    depth.latitude = DepthPressure.DEPTH_PRESSURE_NO_DATA

    ctd.pressure = depth.pressure
    ctd.conductivity_covar = ctd.pressure_covar = ctd.salinity_covar = -1
    ctd.sound_speed_covar = ctd.temperature_covar = -1
    if immersion < 0:
        ctd.conductivity = ctd.salinity = Ctd.CTD_NO_DATA
        ctd.sound_speed = ctd.temperature = Ctd.CTD_NO_DATA
        return ctd, depth

    # A warm surface layer over cooler seawater of practical salinity 35
    t = 12 + 3 * math.exp(-immersion / 5)
    ctd.temperature = t
    ctd.salinity = 35.0

    # PSS-78 reference conductivity in S/m at surface pressure (UNESCO 1983)
    t68 = t * 1.00024
    ratio = (0.6766097 + 0.0200564 * t68 + 1.104259e-4 * t68**2
             - 6.9698e-7 * t68**3 + 1.0031e-9 * t68**4)
    ctd.conductivity = 4.2914 * ratio

    # Mackenzie (1981) sound speed in m/s for salinity 35
    ctd.sound_speed = (1448.96 + 4.591*t - 5.304e-2*t**2 + 2.374e-4*t**3
                       + 1.630e-2*immersion + 1.675e-7*immersion**2
                       - 7.139e-13*t*immersion**3)
    return ctd, depth


def main():
    rospy.init_node('ctd_sim')
    surface_z = rospy.get_param('~water_surface_z', 0.0)

    ctd_pub = rospy.Publisher('ctd/ctd', Ctd, queue_size=5)
    depth_pub = rospy.Publisher('ctd/depth', DepthPressure, queue_size=5)

    # Publish both CTD messages from the same physics observation
    def on_position(msg):
        if not math.isfinite(msg.point.z):
            rospy.logwarn_throttle(5, 'Skipping non-finite CTD height')
            return
        ctd, depth = sample_ctd(msg, surface_z)
        ctd_pub.publish(ctd)
        depth_pub.publish(depth)

    rospy.Subscriber('sim/ctd_position', PointStamped, on_position,
                     queue_size=5)
    rospy.spin()


if __name__ == '__main__':
    main()
