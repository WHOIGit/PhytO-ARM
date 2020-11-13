#!/usr/bin/env python3
import asyncio
import datetime
import functools
import math

import rospy

from ds_core_msgs.msg import RawData
from ds_sensor_msgs.msg import Ctd


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

    # We will publish CTD messages
    publisher = rospy.Publisher(f'/rbr_maestro3/ctd', Ctd, queue_size=5)

    # Initialize the ROS node
    rospy.init_node('rbr_maestro3', anonymous=True, disable_signals=True)
    rospy.core.add_preshutdown_hook(functools.partial(on_shutdown, loop))

    # Subscribe to incoming comms messages
    rospy.Subscriber('/rbr_comms/in', RawData,
                     lambda msg: asyncio.run_coroutine_threadsafe(
                         ros_msg_q.put(msg), loop))

    # Process incoming messages in a loop
    while True:
        msg = await ros_msg_q.get()

        # Crummy parser
        fields = [ x.strip() for x in msg.data.split(b',') ]

        # outputformat channelslist = [timestamp|]conductivity(mS/cm)|temperature(C)|pressure(dbar)|chlorophyll(ug/L)|dissolved-O2(%)|PAR(umol/m2/s)|turbidity(NTU)|phycocyanin(cells/mL)
        conductivity, temperature, pressure, chlorophyll, \
            dissolved_O2, PAR, turbidity, phycocyanin = \
            [ float(x) for x in fields[1:] ]

        # Parse the timestamp as UTC
        timestamp = datetime.datetime.strptime(fields[0].decode('utf-8'),
                                               '%Y-%m-%d %H:%M:%S.%f')
        timestamp = timestamp.replace(tzinfo=datetime.timezone.utc)

        # Construct the CTD message
        ctd = Ctd()
        ctd.conductivity = conductivity
        ctd.temperature = temperature
        ctd.pressure = pressure

        # Clear fields with no measurement
        ctd.salinity = math.nan
        ctd.sound_speed = math.nan

        # Set covariance fields to -1, the standard "not valid" value
        ctd.conductivity_covar = ctd.temperature_covar = ctd.pressure_covar = \
            ctd.salinity_covar, ctd.sound_speed_covar = -1

        # The standard ROS header timestamp reflects when the instrument
        # says the sample was taken. The DsHeader reflects the I/O time.
        ctd.header.stamp = rospy.Time.from_sec(timestamp.timestamp())
        ctd.ds_header.io_time = msg.ds_header.io_time

        # Publish it
        publisher.publish(ctd)


if __name__ == '__main__':
    asyncio.get_event_loop().run_until_complete(main())  # Python <3.7
