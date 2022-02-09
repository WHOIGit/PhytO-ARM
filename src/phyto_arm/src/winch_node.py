#!/usr/bin/env python3
import asyncio
import functools
import math

import actionlib
import rospy

from scipy.integrate import quad

from ds_sensor_msgs.msg import DepthPressure
from jvl_motor.msg import Motion

from phyto_arm.msg import MoveToDepthAction, MoveToDepthFeedback, \
    MoveToDepthResult


# COMMAND LINE NOTE
#
# rostopic pub -1 /winch/move_to_depth/goal phyto_arm/MoveToDepthActionGoal \
#     '{ goal: { depth: 2.0 } }'


# Holds a value. Coroutines can await changes to this value.
#
# /!\ The value can change out from under you every time you yield back to the
# event loop. Copy the value to a local variable if necessary.
class AwaitableValue:
    def __init__(self, loop, initial=None):
        self.loop, self.value = loop, initial
        self.event = asyncio.Event()
    
    async def update(self, value):
        self.value = value
        self.event.set()
        self.event.clear()
    
    def update_soon(self, value):
        asyncio.run_coroutine_threadsafe(self.update(value), self.loop)

    async def wait(self):
        await self.event.wait()


# A wrapper arond actionlib.SimpleActionServer that can invoke a callback
# coroutine.
class AsyncSimpleActionServer:
    def __init__(self, name, action, callback, loop):
        self.callback, self.loop = callback, loop
        self.server = actionlib.SimpleActionServer(
            name,
            action,
            self._execute,
            False  # required by API
        )

    def _execute(self, goal):
        asyncio.run_coroutine_threadsafe(self.callback(self, goal), self.loop)\
               .result()  # block on completion

    def __getattr__(self, name):
        return getattr(self.server, name)


# Global variables for the last known depth and motor readings
depth = None
motor = None


# This function returns a velocity function with the given parameters.
# 
# This is a sigmoid function ranging from - to +max_speed with zero at the
# target distance, reaching 1/2*max_speed at half_speed_dist from target.
#
# The smaller the value of half_speed_dist, the more abruptly we will stop.
def velocity_f(target, max_speed, half_speed_dist):
    return (lambda x: max_speed * (2/math.pi) * \
            math.atan((target - x) * (1 / half_speed_dist)))


# Compute an estimate of the move duration using an approximate integral of
# the velocity function.
#
# Note we must estimate the time to our within-epsilon stopping distance, or
# else the integral will not converge.
def estimate_time(v, start, target, epsilon):
    t, _ = quad(
        lambda x: 1/v(x),
        start,
        target + (-1 if start < target else 1) * epsilon
    )
    return rospy.Duration.from_sec(t)


def dist_to_encoder_counts(dist):
    # return 8192 * gear_ratio * dist / spool_circumference
    return 8192 * 60 * dist / 0.6




async def move_to_depth(server, goal):
    # Get an initial depth fix and motor status message
    try:
        await asyncio.wait([
            asyncio.create_task(depth.wait()),
            asyncio.create_task(motor.wait()),
        ], timeout=2.0, return_when=asyncio.ALL_COMPLETED)
    except asyncio.TimeoutError:
        server.set_aborted(text='Timed out waiting for initial status')
        return

    # Safety check: The motor should not be in motion
    if motor.value.mode != Motion.MODE_PASSIVE:
        server.set_aborted(text='Motor is already in motion')
        return
    
    # Safety check: The depth reading should be valid
    if depth.value.depth == DepthPressure.DEPTH_PRESSURE_NO_DATA:
        server.set_aborted(text='Depth reading from CTD is invalid')
        return

    # Save our initial depth
    start_depth = depth.value.depth
    rospy.loginfo(f'Starting depth is {start_depth} m')

    # Velocity function
    v = velocity_f(goal.depth, 0.02, 0.05)
    epsilon = 0.01

    # Estimate the time it should take to reach the destination
    expected_time = estimate_time(v, start_depth, goal.depth, epsilon)
    rospy.loginfo(f'Estimated movement time is {expected_time.to_sec():.0f} s')

    time_limit = max(
        1.10 * expected_time,
        expected_time + rospy.Duration.from_sec(10.0)
    )

    # Compute some position bounds
    if goal.depth < start_depth:
        lower_bound = motor.value.position - \
            dist_to_encoder_counts((start_depth - goal.depth) + 0.10)
        upper_bound = motor.value.position + dist_to_encoder_counts(0.10)
    else:
        lower_bound = motor.value.position - dist_to_encoder_counts(0.10)
        upper_bound = motor.value.position + \
            dist_to_encoder_counts((start_depth - goal.depth) + 0.10)

    # TODO IMPORTANT:
    # Complete the position envelope calculation including checking for
    # overflow.

    # Record the time that we start moving
    start_time = rospy.Time.now()

    success = False
    while True:
        elapsed = rospy.Time.now() - start_time

        # If preempted, stop here
        if server.is_preempt_requested():
            msg = 'Goal was preempted'
            rospy.loginfo(msg)
            server.set_preempted(text=msg)
            break

        if elapsed > time_limit:
            msg = 'Time limited exceeded'
            rospy.logerr(msg)
            server.set_aborted(text=msg)
            break

        # if motor.value.mode == Motion.MODE_PASSIVE:
        #     server.set_aborted(text='Motor unexpectedly stopped')
        #     break
        
        feedback = MoveToDepthFeedback()
        feedback.time_elapsed.data = elapsed
        feedback.depth = depth.value.depth
        server.publish_feedback(feedback)

        await asyncio.sleep(1.0)


    # If we fall out of the loop for some reason, 
    if not success:
        if server.is_active():
            server.set_aborted(text='Failed for unknown reason')

        # TODO:
        # Send motor Stop message
        return

    # Send a success message
    result = MoveToDepthResult()
    result.time_elapsed.data = (rospy.Time.now() - start_time)


async def main():
    # XXX Prefer asyncio.get_running_loop() on Python >= 3.7
    loop = asyncio.get_event_loop()

    # Initialize our AwaitableValues with the event loop reference
    global depth, motor
    depth = AwaitableValue(loop, None)
    motor = AwaitableValue(loop, None) 

    # Initialize the ROS node
    rospy.init_node('winch', anonymous=True, disable_signals=True)
    rospy.core.add_preshutdown_hook(loop.stop)

    # Subscribe to incoming messages
    rospy.Subscriber('/ctd/depth', DepthPressure, depth.update_soon)
    rospy.Subscriber('/jvl_motor/motion', Motion, motor.update_soon)

    # Create an action server for the MoveToDepth action
    server = AsyncSimpleActionServer(
        '/winch/move_to_depth',
        MoveToDepthAction,
        move_to_depth,
        loop
    )
    server.start()

    # Spin so that other tasks can run
    while True:
        await asyncio.sleep(1.0)


if __name__ == '__main__':
    asyncio.get_event_loop().run_until_complete(main())  # Python <3.7
