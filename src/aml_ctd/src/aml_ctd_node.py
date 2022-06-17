#!/usr/bin/env python3
import asyncio
import datetime
import functools
import math
import re

import rospy

import amlxparser

from ds_core_msgs.msg import RawData
from ds_sensor_msgs.msg import Ctd, DepthPressure

from aml_ctd.msg import AmlMeasurement


# Converts an arbitrary string to a ROS-safe name per http://wiki.ros.org/Names.
def ros_safe(s, keep_slashes=False):
    # Spaces and optionally slashes to underscores
    s = s.replace(' ', '_')
    if not keep_slashes:
        s = s.replace('/', '_')

    # Split up camelCase, without breaking acronyms
    s = re.sub(r'([A-Z]+)([A-Z][a-z])', r'\g<1>_\g<2>', s)
    s = re.sub(r'([a-z])([A-Z])', r'\g<1>_\g<2>', s)

    # Drop other characters and reduce to lowercase
    s = re.sub(r'[^a-z0-9_]', '', s.lower())
    if not re.match(r'^[a-z]', s):
        s = 'x' + s  # must start with a letter
    return s


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
        '~ctd': rospy.Publisher('~', Ctd, queue_size=5),
        '~depth': rospy.Publisher('~depth', DepthPressure, queue_size=5),
    }

    # Subscribe to incoming comms messages
    handler = lambda msg: asyncio.run_coroutine_threadsafe(ros_msg_q.put(msg),
                                                           loop)
    rospy.Subscriber('~in', RawData, handler)

    # TODO:
    # To start logging we should just need to send b"\r\r\rMMONITOR\r" (sic)

    # Process incoming messages in a loop
    while True:
        msg = await ros_msg_q.get()

        # Parse the message data
        # FIXME: Unsure if we ought to decode to a string or leave as bytes
        try:
            parsed = amlxparser.parseAMLx(msg.data.decode())
        except:
            rospy.logerr(f'Failed to parse: {msg.data.decode()}')
            continue

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

        # The parser assumes the timestamp is in UTC
        timestamp = parsed['mux']['time']

        # Construct AmlMeasurement messages from amlxparser.Measurement objects
        aml_msgs = []
        for obj in data.values():
            aml_msg = AmlMeasurement()
            for field in set(obj._fields) - {'rawvalue'}:
                setattr(aml_msg, field, getattr(obj, field))

            # Special case: String fields cannot be None
            aml_msg.rawname = aml_msg.rawname or ''
            aml_msg.rawunit = aml_msg.rawunit or ''

            # Special case: rawvalue might be an integer or a float or None
            if obj.rawvalue is None:
                aml_msg.rawvalue_f = math.nan
            elif isinstance(obj.rawvalue, float):
                aml_msg.rawvalue_f = obj.rawvalue
            elif isinstance(obj.rawvalue, int):
                aml_msg.rawvalue_i = obj.rawvalue
            else:
                raise TypeError(f'Unexpected rawvalue {type(obj.rawvalue)}')

            aml_msgs.append(aml_msg)

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

        # Set the appropriate message timestamps and metadata
        for m in [ctd, dp] + aml_msgs:
            # Associate a frame_id so we can connect these messages together
            m.header.frame_id = f'msg{parsed["msgnum"]}'

            # The standard ROS header timestamp reflects when the instrument
            # says the sample was taken. The DsHeader reflects the I/O time.
            m.header.stamp = rospy.Time.from_sec(timestamp.timestamp())
            m.ds_header.io_time = msg.ds_header.io_time

        # Publish the messages
        publishers['~ctd'].publish(ctd)
        publishers['~depth'].publish(dp)
        for m in aml_msgs:
            top = f'~aml/{ros_safe(m.port)}/{ros_safe(m.name)}'
            if top not in publishers:
                publishers[top] = rospy.Publisher(top, type(m), queue_size=5)
            publishers[top].publish(m)


if __name__ == '__main__':
    asyncio.get_event_loop().run_until_complete(main())  # Python <3.7
