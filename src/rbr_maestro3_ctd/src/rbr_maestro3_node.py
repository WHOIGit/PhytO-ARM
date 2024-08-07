#!/usr/bin/env python3
import asyncio
import datetime
import functools
import math
import re

import rospy

from ds_core_msgs.msg import RawData
from ds_sensor_msgs.msg import Ctd, DepthPressure

from rbr_maestro3_ctd.msg import RbrMeasurement


# Converts an arbitrary string to a ROS-safe name per http://wiki.ros.org/Names.
# TODO: Copied from aml_ctd_node.py; move to a common library.
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
    rospy.init_node('rbr_maestro3_ctd', anonymous=True, disable_signals=True)
    rospy.core.add_preshutdown_hook(functools.partial(on_shutdown, loop))

    # Load the list of channels to publish from the ROS parameter server. The
    # channels list should correspond with the `outputformat channelslist`
    # configuration of the instrument.
    #
    # The channels are parsed into a (channel, unit) tuple:
    #     O2_concentration(umol/L) -> ('O2_concentration', 'umol/L')
    #
    # TODO: Query the instrument for its outputformat configuration.
    channels = rospy.get_param('~channels', [])
    if isinstance(channels, str):
        channels = channels.split('|')
    channels = [
        tuple(c.strip(')').split('(')) if '(' in c else (c, '')
        for c in channels
    ]

    
    # Check a few channel types we are inflexible about
    has_depth = False
    for c, u in channels:
        if c.lower() == 'depth':
            has_depth = True
            assert u == 'm'
        if c.lower() == 'pressure':
            assert u == 'dbar'


    # Create publishers for each channel
    ctd_publisher = rospy.Publisher('~', Ctd, queue_size=5)
    if has_depth:
        depth_publisher = rospy.Publisher('~depth', DepthPressure, queue_size=5)
    channel_publishers = []
    for c, u in channels:
        if c == 'depth':
            continue
        channel_publishers.append(rospy.Publisher(f'~{ros_safe(c)}', RbrMeasurement, queue_size=5))

    # Subscribe to incoming comms messages
    handler = lambda msg: asyncio.run_coroutine_threadsafe(ros_msg_q.put(msg),
                                                           loop)
    rospy.Subscriber('~in', RawData, handler)

    # Process incoming messages in a loop
    while not rospy.is_shutdown():
        msg = await ros_msg_q.get()

        # Crummy parser
        fields = [ x.strip() for x in msg.data.split(b',') ]
        if len(fields) != len(channels) + 1:
            rospy.logwarn('Discarding malformed RBR message - expected '
                          f'{len(channels)+1} fields but received '
                          f'{len(fields)}')
            continue

        try:
            # Parse all channel values as floats
            values = {
                c.lower(): float(x) for (c, u), x in zip(channels, fields[1:])
            }

            # Parse the timestamp as UTC
            timestamp = datetime.datetime.strptime(fields[0].decode(),
                                                   '%Y-%m-%d %H:%M:%S.%f')
            timestamp = timestamp.replace(tzinfo=datetime.timezone.utc)
        except ValueError as e:
            rospy.logwarn(f'Discarding RBR message due to error during parsing: {e}')
            continue


        # Construct the Ctd message
        ctd = Ctd()
        ctd.conductivity = values.get('conductivity', math.nan)
        ctd.temperature = values.get('temperature', math.nan)
        ctd.pressure = values.get('pressure', math.nan)

        # Clear fields with no measurement
        ctd.salinity = math.nan
        ctd.sound_speed = math.nan

        # Set covariance fields to -1, the standard "not valid" value
        ctd.conductivity_covar = ctd.temperature_covar = ctd.pressure_covar = \
            ctd.salinity_covar = ctd.sound_speed_covar = -1


        # Construct the DepthPressure message
        dp = DepthPressure()
        dp.depth = values.get('depth', math.nan)
        dp.latitude = DepthPressure.DEPTH_PRESSURE_NO_DATA
        dp.tare = DepthPressure.DEPTH_PRESSURE_NO_DATA
        dp.pressure_raw = values.get('pressure', math.nan)
        dp.pressure_raw_unit = DepthPressure.UNIT_PRESSURE_DBAR
        dp.pressure = values.get('pressure', math.nan)

        # Construct the RbrMeasurement messages
        rbr_msgs = []
        for c, u in channels:
            rbr_msg = RbrMeasurement()
            rbr_msg.channel = c
            rbr_msg.unit = u
            rbr_msg.value = values[c.lower()]
            rbr_msgs.append(rbr_msg)


        # Assemble all messages and their publishers
        all_msgs = [(ctd, ctd_publisher)]
        all_msgs.extend(zip(rbr_msgs, channel_publishers))
        if has_depth:
            all_msgs.append((dp, depth_publisher))

        # Set the appropriate message timestamps and publish
        for m, publisher in all_msgs:
            # The standard ROS header timestamp reflects when the instrument
            # says the sample was taken. The DsHeader reflects the I/O time.
            m.header.stamp = rospy.Time.from_sec(timestamp.timestamp())
            m.ds_header.io_time = msg.ds_header.io_time

            publisher.publish(m)


if __name__ == '__main__':
    asyncio.get_event_loop().run_until_complete(main())  # Python <3.7
