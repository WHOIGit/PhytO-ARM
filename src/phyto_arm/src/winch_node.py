#!/usr/bin/env python3
import asyncio
import math
import signal
import uuid

import actionlib
import rospy

from scipy.integrate import quad

from ds_sensor_msgs.msg import DepthPressure
from jvl_motor.msg import Motion
from jvl_motor.srv import SetPositionEnvelopeCmd, SetVelocityCmd, StopCmd

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


# Convenience methods for making it easier to print a log message at the same
# time as updating a goal's status.
class BetterActionServerLoggingMixin:
    def set_aborted(self, result=None, text="", log=rospy.logerr):
        if text and callable(log):
            log(text)
        return self.server.set_aborted(result, text=text)

    def set_preempted(self, result=None, text="", log=rospy.loginfo):
        if text and callable(log):
            log(text)
        return self.server.set_preempted(result, text=text)


# A wrapper arond actionlib.SimpleActionServer that can invoke a callback
# coroutine.
class AsyncSimpleActionServer(BetterActionServerLoggingMixin):
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


# Service proxies for interacting with the motor
set_position_envelope = rospy.ServiceProxy('motor/set_position_envelope',
    SetPositionEnvelopeCmd)
set_velocity = rospy.ServiceProxy('motor/set_velocity', SetVelocityCmd)
stop = rospy.ServiceProxy('motor/stop', StopCmd)


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


# This is our entrypoint to the MoveToDepth action, which has some additional
# guardrails to prevent a runaway motor.
async def move_to_depth_chk(server, goal):
    try:
        await move_to_depth(server, goal)
    except:
        server.set_aborted(text='An exception occurred')
        raise
    finally:
        # No matter what happened, the motor should be stopped by now
        stop()  # blocking


async def move_to_depth(server, goal):
    # Put initial status tasks in a lookup dict for traceability
    tasks = {
        asyncio.create_task(depth.wait()): "depth status",
        asyncio.create_task(motor.wait()): "motor status"
    }
    # Get an initial depth fix and motor status message
    _, pending = await asyncio.wait(tasks.keys(), timeout=2.0, return_when=asyncio.ALL_COMPLETED)

    if pending:
        err_text = f'Timed out waiting for {", ".join([tasks[p] for p in pending])}'
        server.set_aborted(text=err_text)
        return

    # Safety check: The depth reading should be valid
    if depth.value.depth == DepthPressure.DEPTH_PRESSURE_NO_DATA:
        server.set_aborted(text='Depth reading from CTD is invalid')
        return

    # Safety check: The motor should not be in motion
    if motor.value.mode != Motion.MODE_PASSIVE:
        server.set_aborted(text='Motor is already in motion')
        return

    # Safety check: Try to stop the motor (synchronously). This is a no-op but
    # if there's an error, it spares us from being unable to stop it later.
    stop()  # blocking

    # Save our initial depth and compute depth limits
    start_depth = depth.value.depth
    depth_margin = rospy.get_param('~safety_envelopes').get('position', 0.1)
    depth_min = min(start_depth, goal.depth) - depth_margin  # m
    depth_max = max(start_depth, goal.depth) + depth_margin  # m

    rospy.logdebug(f'Starting depth is {start_depth:.2f} m')
    rospy.loginfo('Setting depth envelope to ' +
                  f'({depth_min:.2f}, {depth_max:.2f}) m')

    # Velocity function
    max_speed = rospy.get_param('~max_speed' )
    half_speed_dist = rospy.get_param('~half_speed_dist')
    # Assert both are not None
    assert max_speed is not None and half_speed_dist is not None, 'Winch speed config invalid'
    assert goal.velocity < max_speed, 'Goal velocity exceeds max speed'
    v = velocity_f(goal.depth, goal.velocity, half_speed_dist)

    # Estimate the time it should take to reach the destination
    epsilon = rospy.get_param('~epsilon', 0.01)
    assert epsilon is not None, 'Winch epsilon config invalid'
    expected_time = estimate_time(v, start_depth, goal.depth, epsilon)
    rospy.loginfo(f'Estimated movement time is {expected_time.to_sec():.0f} s')

    # Set a time limit based on the estimate
    time_limit = max(
        1.10 * expected_time,
        expected_time + rospy.Duration.from_sec(10.0)
    )

    rospy.loginfo(f'Setting time limit to {time_limit.to_sec():.0f} s')

    # Compute the RPM to m/s ratio
    gear_ratio = rospy.get_param('~gear_ratio')
    spool_circumference = rospy.get_param('~spool_circumference')
    rpm_ratio = 60 * gear_ratio / spool_circumference

    # Create a function to convert distances to encoder counts
    dist2encoder = \
        lambda d: int(round(8192 * gear_ratio * d / spool_circumference))

    # Set some position bounds on the motor itself
    rospy.logdebug(f'Current motor position is {motor.value.position:.2f}')

    if goal.depth < start_depth:
        lower_bound = motor.value.position - \
            dist2encoder((start_depth - goal.depth) + depth_margin)
        upper_bound = motor.value.position + dist2encoder(depth_margin)
    else:
        lower_bound = motor.value.position - dist2encoder(depth_margin)
        upper_bound = motor.value.position + \
            dist2encoder((goal.depth - start_depth) + depth_margin)

    if lower_bound < -(2**24) + 1 or upper_bound > 2**24 - 1:
        server.set_aborted(text='Encoder position could wrap -- reset offset')
        return

    rospy.loginfo('Motor position envelope is ' + 
                  f'({lower_bound:.2f}, {upper_bound:.2f})')

    # Record the time that we start moving
    start_time = rospy.Time.now()

    success, started = False, False
    while True:
        elapsed = rospy.Time.now() - start_time

        # If we reached our target, stop
        if abs(depth.value.depth - goal.depth) < epsilon:
            rospy.loginfo('Reached target!')
            success = True
            break

        # If preempted, stop
        if server.is_preempt_requested():
            server.set_preempted(text='Goal was preempted')
            break

        # If we exceed the programmed position envelope, the motor will stop
        # automatically. Take this as a sign something bad happened.
        if started and motor.value.mode != Motion.MODE_VELOCITY:
            server.set_aborted(text='Motor unexpectedly stopped')
            if not (pos_min < motor.value.position < pos_max):
                rospy.logerr(
                    'Position envelope exceeded -- '
                    f'{motor.value.position} not in ({pos_min, pos_max})'
                )
            break
        elif not started and motor.value.mode == Motion.MODE_VELOCITY:
            # We started moving, so turn on the check above
            started = True

        # If too much time has elapsed, stop
        if elapsed > time_limit:
            server.set_aborted(text=f'Time limit exceeded: {elapsed}/{time_limit}')
            break

        # If depth is outside of bounds, stop
        if not (depth_min <= depth.value.depth <= depth_max):
            server.set_aborted(text=f'Exceeded depth bounds: {depth.value.depth} not in ({depth_min}, {depth_max})')
            break

        # Update bounds so we do not travel far from this position
        pos_margin = dist2encoder(depth_margin)
        pos_min = max(lower_bound, motor.value.position - pos_margin)
        pos_max = min(upper_bound, motor.value.position + pos_margin)
        set_position_envelope(min=pos_min, max=pos_max)

        # Compute and command a new velocity according to our current depth
        velocity = v(depth.value.depth)
        rpm = rpm_ratio * velocity

        set_velocity(
            velocity=rpm,       # RPM
            acceleration=1200,  # RPM/s
            torque=3.0          # rated torque
        )

        # Publish feedback about our current progress
        feedback = MoveToDepthFeedback()
        feedback.time_elapsed.data = elapsed
        feedback.depth = depth.value.depth
        feedback.velocity = velocity
        server.publish_feedback(feedback)

        await asyncio.sleep(1.0)

    # Record duration of movement
    elapsed_time = rospy.Time.now() - start_time

    # However we broke out of the loop, stop the motor immediately
    rospy.logdebug('Loop finished, stopping motor')
    stop()  # blocking

    # Disable the position envelope
    set_position_envelope(min=0, max=0)

    # If we fell out of the loop unsuccessfully for any reason, end here
    if not success:
        if server.is_active():
            server.set_aborted(text='Failed for unknown reason')
        return

    # Send a success message
    result = MoveToDepthResult()
    result.uuid = str(uuid.uuid4())  # for linking with simultaneous profile msg
    result.time_elapsed.data = elapsed_time
    server.set_succeeded(result)

    # Suggest a new conversion factor
    if abs(start_depth - goal.depth) >= 1.0:
        time_ratio = expected_time / elapsed_time
        rospy.set_param('~spool_circumference',
                        spool_circumference * time_ratio)
        rospy.loginfo(f'Adjusting RPM ratio from {rpm_ratio:.4f} to '
                      f'{rpm_ratio / time_ratio:.4f}')


async def main():
    # XXX Prefer asyncio.get_running_loop() on Python >= 3.7
    loop = asyncio.get_event_loop()

    # Initialize our AwaitableValues with the event loop reference
    global depth, motor
    depth = AwaitableValue(loop, None)
    motor = AwaitableValue(loop, None)

    # Initialize the ROS node
    rospy.init_node('winch', anonymous=True, disable_signals=True)

    # Register signal handlers the same way rospy would
    for sig in [signal.SIGTERM, signal.SIGINT]:
        loop.add_signal_handler(sig, rospy.signal_shutdown,
                                f'signal-{sig.value}')
    rospy.core.add_preshutdown_hook(lambda reason: loop.stop())

    # Wait for services to become available
    for svc in [ set_velocity, set_position_envelope, stop ]:
        svc.wait_for_service()

    # Subscribe to incoming messages
    rospy.Subscriber(rospy.get_param('ctd_topic'), DepthPressure, depth.update_soon)
    rospy.Subscriber('motor/motion', Motion, motor.update_soon)

    # Create an action server for the MoveToDepth action
    server = AsyncSimpleActionServer(
        'winch/move_to_depth',
        MoveToDepthAction,
        move_to_depth_chk,
        loop
    )
    server.start()

    # Spin so that other tasks can run
    while True:
        await asyncio.sleep(1.0)


if __name__ == '__main__':
    asyncio.run(main())
