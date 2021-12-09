#!/usr/bin/env python3
import asyncio
import datetime
import functools
import math

import rospy

import amlxparser

from ds_core_msgs.msg import RawData
from ds_sensor_msgs.msg import Ctd, DepthPressure


# All messages received from ROS are placed into this queue. Note that
# asyncio.Queue is _not_ thread safe.
ros_msg_q = asyncio.Queue()



# Before ROS tears down our subscriptions, take time to clean up.
def on_shutdown(loop, *args):
    def impl():
        # Executed on the main thread
        pass

    # Helper coroutine
    async def impl_async():
        impl()

    # If this isn't invoked on the main thread, then we would want to schedule
    # a coroutine to run on the main event loop and block on its completion.
    #
    # But if this *is* invoked on the main thread, then doing so would deadlock.
    # So we just invoke the function directly.
    if loop == asyncio.get_event_loop():
        impl()
    else:
        future = asyncio.run_coroutine_threadsafe(impl_async(), loop)
        future.result()  # block until complete

    # Stop the main event loop
    loop.stop()


async def main():
    # XXX Prefer asyncio.get_running_loop() on Python >= 3.7
    loop = asyncio.get_event_loop()

    # Initialize the ROS node
    rospy.init_node('aml_ctd', anonymous=True, disable_signals=True)
    rospy.core.add_preshutdown_hook(functools.partial(on_shutdown, loop))

    # Create a mapping of topics to publish on
    publishers = {
        Ctd: rospy.Publisher(f'{rospy.get_name()}/ctd', Ctd, queue_size=5),
        DepthPressure: rospy.Publisher(f'{rospy.get_name()}/depth',
            DepthPressure, queue_size=5),
    }

    # Subscribe to incoming comms messages
    handler = lambda msg: asyncio.run_coroutine_threadsafe(ros_msg_q.put(msg),
                                                           loop)
    rospy.Subscriber('/aml_comms/in', RawData, handler)

    # TODO:
    # To start logging we should just need to send b"\r\r\rMMONITOR\r" (sic)

    # Process incoming messages in a loop
    while True:
        msg = await ros_msg_q.get()

        # Parse the message data
        # FIXME: Unsure if we ought to decode to a string or leave as bytes
        parsed = amlxparser.parseAMLx(msg.data.decode())

        # Create a lookup table of measurements
        data = {}
        for v in parsed.values():
            if isinstance(v, dict):
                data.update({
                    k: vv for k, vv in v.items()
                    if isinstance(vv, amlxparser.Measurement)
                })
        
        # Validate assumptions about the measurement units
        assert data['Depth'].unit == 'm' if 'Depth' in data else True
        assert data['Cond'].unit == 'mS/cm'
        assert data['TempCT'].unit == 'C'
        assert data['Pressure'].unit == 'dbar'
        assert data['SV'].unit == 'm/s'

        # Assume the timestamp is UTC
        timestamp = parsed['mux']['time']
        timestamp = timestamp.replace(tzinfo=datetime.timezone.utc)

        # Construct the Ctd message
        ctd = Ctd()
        ctd.conductivity = data['Cond'].value * 0.1  # convert to S/m
        ctd.temperature = data['TempCT'].value
        ctd.pressure = data['Pressure'].value
        ctd.sound_speed = data['SV'].value

        # Clear fields with no measurement
        ctd.salinity = math.nan

        # Set covariance fields to -1, the standard "not valid" value
        ctd.conductivity_covar = ctd.temperature_covar = ctd.pressure_covar = \
            ctd.salinity_covar = ctd.sound_speed_covar = -1
        
        # Construct the DepthPressure message
        dp = DepthPressure()
        dp.depth = data['Depth'].value if 'Depth' in data else \
            DepthPressure.DEPTH_PRESSURE_NO_DATA
        dp.latitude = DepthPressure.DEPTH_PRESSURE_NO_DATA
        dp.tare = DepthPressure.DEPTH_PRESSURE_NO_DATA
        dp.pressure_raw = data['Pressure'].value
        dp.pressure_raw_unit = DepthPressure.UNIT_PRESSURE_DBAR
        dp.pressure = data['Pressure'].value

        # Set the appropriate message timestamps
        for m in [ctd, dp]:
            # The standard ROS header timestamp reflects when the instrument
            # says the sample was taken. The DsHeader reflects the I/O time.
            m.header.stamp = rospy.Time.from_sec(timestamp.timestamp())
            m.ds_header.io_time = msg.ds_header.io_time

        # Publish the messages
        for m in [ctd, dp]:
            publishers[type(m)].publish(m)


if __name__ == '__main__':
    asyncio.get_event_loop().run_until_complete(main())  # Python <3.7
