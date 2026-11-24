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
from jvl_motor.srv import SetPositionEnvelopeCmd, SetVelocityCmd, StopCmd, ZeroCmd

from phyto_arm.msg import MoveToDepthAction, MoveToDepthFeedback, \
    MoveToDepthResult
from rosgraph_msgs.msg import Clock


# COMMAND LINE NOTE
#
# rostopic pub -1 /winch/move_to_depth/goal phyto_arm/MoveToDepthActionGoal \
#     '{ goal: { depth: 2.0 } }'


# Holds a value. Coroutines can await changes to this value.
#
# /!\ The value can change out from under you every time you yield back to the
# event loop. Copy the value to a local variable if necessary.
class AwaitableValue:
    def __init__(self, loop, initial=None, *, now=rospy.Time.now):
        self.loop, self.value = loop, initial
        self.now = now
        self.next_receipt = loop.create_future()
        self.received = None

    def update(self, value):
        self.value = value
        self.received = self.now()
        self.next_receipt.set_result(self.received)
        self.next_receipt = self.loop.create_future()

    def update_soon(self, value):
        self.loop.call_soon_threadsafe(self.update, value)

    async def wait(self):
        await asyncio.shield(self.next_receipt)
        return self.value

    def __await__(self):
        return self.wait().__await__()


# Remember a gap even if several readings arrive before the winch resumes
class WinchStatus(AwaitableValue):
    timeout = rospy.Duration(2)

    def __init__(self, clock):
        super().__init__(clock.loop, now=clock.now)
        self.clock = clock
        self.gaps = 0

    # Readings received at time zero age from the first clock tick
    @property
    def deadline(self):
        received = self.received or self.clock.first_tick
        if received is not None:
            return received + self.timeout

    def update(self, value):
        deadline = self.deadline
        super().update(value)
        if deadline is not None and self.received > deadline:
            self.gaps += 1


# Wake the control loop on clock ticks and remember rewinds between updates
class AwaitableRosClock:
    def __init__(self, loop):
        self.loop = loop
        self.changed = loop.create_future()
        self.stamp = None
        self.epoch = 0
        self.first_tick = None
        self.rewind_amount = 0
        if rospy.get_param('/use_sim_time', False):
            self.subscription = rospy.Subscriber(
                '/clock', Clock,
                lambda msg: loop.call_soon_threadsafe(self.tick, msg.clock)
            )
            self.stamp = rospy.Time.now()

    def tick(self, stamp):
        previous = self.stamp
        if stamp < previous:
            self.epoch += 1
            self.rewind_amount = (previous - stamp).to_sec()
            self.first_tick = None
        if previous.is_zero() and not stamp.is_zero():
            self.first_tick = stamp
        self.stamp = stamp
        self.changed.set_result(None)
        self.changed = self.loop.create_future()

    def now(self):
        return rospy.Time.now() if self.stamp is None else self.stamp

    async def wait_for_change(self, deadline, updates=()):
        previous = self.now()
        epoch = self.epoch
        timeout = None
        if self.stamp is None:
            timeout = max(0, (deadline - previous).to_sec())
        await asyncio.wait([self.changed, *updates], timeout=timeout,
                           return_when=asyncio.FIRST_COMPLETED)

        # A rewind can be followed by another tick before the waiter resumes
        if self.epoch != epoch:
            raise rospy.exceptions.ROSTimeMovedBackwardsException(
                self.rewind_amount)
        now = self.now()
        if now < previous:
            raise rospy.exceptions.ROSTimeMovedBackwardsException(
                (previous - now).to_sec())


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


# ROS clock and the last known depth and motor readings
clock = None
depth = None
motor = None


# Service proxies for interacting with the motor
zero_position = rospy.ServiceProxy('motor/zero_position', ZeroCmd)
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


# Wait up to timeout ROS seconds for all values to update
async def wait_for_updates(clock, values, timeout):
    if not isinstance(timeout, rospy.Duration):
        raise TypeError('timeout must be a rospy.Duration')

    receipts = [value.next_receipt for value in values]
    if not receipts:
        return

    # Gazebo can be paused at zero before the first clock tick
    start = clock.now()
    while start.is_zero():
        await clock.wait_for_change(start + timeout)
        start = clock.first_tick or clock.now()

    deadline = start + timeout
    while not all(receipt.done() and receipt.result() <= deadline
                  for receipt in receipts):
        if clock.now() >= deadline:
            raise asyncio.TimeoutError
        pending = [receipt for receipt in receipts if not receipt.done()]
        await clock.wait_for_change(deadline, pending)


# Yield elapsed ROS time at the start of a move and on each status update
async def motion_updates(start, time_limit):
    now = start
    streams = (('depth', depth, depth.gaps), ('motor', motor, motor.gaps))
    while True:
        elapsed = now - start
        if elapsed >= time_limit:
            raise asyncio.TimeoutError(
                f'Time limit exceeded: {elapsed}/{time_limit}')

        # Each stream must stay fresh throughout the move
        deadline = start + time_limit
        for name, status, gaps in streams:
            expires = status.deadline
            if now >= expires or status.gaps != gaps:
                raise asyncio.TimeoutError(
                    f'Timed out waiting for {name} status')
            deadline = min(deadline, expires)

        updates = (depth.next_receipt, motor.next_receipt)
        yield elapsed

        # Blocking motor calls can span a wall clock adjustment
        previous, now = now, clock.now()
        if now < previous:
            raise rospy.exceptions.ROSTimeMovedBackwardsException(
                (previous - now).to_sec())

        # Wait for either status stream or the earliest deadline
        while now < deadline and not any(update.done() for update in updates):
            await clock.wait_for_change(deadline, updates)
            now = clock.now()


# This is our entrypoint to the MoveToDepth action, which has some additional
# guardrails to prevent a runaway motor.
async def move_to_depth_chk(server, goal):
    loop = asyncio.get_running_loop()
    task = asyncio.current_task()
    server.register_preempt_callback(
        lambda: loop.call_soon_threadsafe(task.cancel)
    )

    try:
        if server.is_preempt_requested():
            raise asyncio.CancelledError

        await move_to_depth(server, goal)
    except rospy.exceptions.ROSTimeMovedBackwardsException:
        server.set_aborted(text='ROS time moved backwards')
    except asyncio.CancelledError:
        if not server.is_preempt_requested():
            raise
        server.set_preempted(text='Goal was preempted')
    except Exception:
        server.set_aborted(text='An exception occurred')
        raise
    finally:
        # No matter what happened, the motor should be stopped by now
        stop()  # blocking


async def move_to_depth(server, goal):
    # Get an initial depth fix and motor status message
    try:
        await wait_for_updates(clock, [depth, motor], WinchStatus.timeout)
    except asyncio.TimeoutError:
        server.set_aborted(text='Timed out waiting for initial winch status')
        return

    # Safety check: The depth reading should be valid
    if depth.value.depth == DepthPressure.DEPTH_PRESSURE_NO_DATA:
        server.set_aborted(text='Depth reading from CTD is invalid')
        return

    # Safety check: The motor should not be in motion
    if motor.value.mode != Motion.MODE_PASSIVE:
        server.set_aborted(text=f'Motor is already in motion, mode: {motor.value.mode}')
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
    dist2encoder = lambda d: int(round(8192 * gear_ratio * d / spool_circumference))

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

    rospy.loginfo(f'Motor position envelope is ({lower_bound:.2f}, {upper_bound:.2f})')

    # Record the time that we start moving
    start_time = clock.now()

    envelope_depth = None
    envelope_position = None
    started = False
    velocity = None
    try:
        async for elapsed in motion_updates(start_time, time_limit):
            # If we reached our target, stop
            if abs(depth.value.depth - goal.depth) < epsilon:
                rospy.loginfo('Reached target!')
                break

            # If we exceed the programmed position envelope, the motor will stop
            # automatically. Take this as a sign something bad happened.
            if started and motor.value.mode != Motion.MODE_VELOCITY:
                server.set_aborted(
                    text=f'Motor unexpectedly stopped, mode: {motor.value.mode}'
                )
                if not (pos_min < motor.value.position < pos_max):
                    rospy.logerr(
                        'Position envelope exceeded -- '
                        f'{motor.value.position} not in ({pos_min, pos_max})'
                    )
                return
            elif not started and motor.value.mode == Motion.MODE_VELOCITY:
                # We started moving, so turn on the check above
                started = True

            # If depth is outside of bounds, stop
            if not (depth_min <= depth.value.depth <= depth_max):
                server.set_aborted(
                    text=f'Exceeded depth bounds: {depth.value.depth} '
                         f'not in ({depth_min}, {depth_max})')
                return

            # Advance the envelope only with fresh depth and a new position
            if (depth.value is not envelope_depth and
                    motor.value.position != envelope_position):
                envelope_depth = depth.value
                envelope_position = motor.value.position
                pos_margin = dist2encoder(depth_margin)
                pos_min = max(lower_bound, envelope_position - pos_margin)
                pos_max = min(upper_bound, envelope_position + pos_margin)
                set_position_envelope(min=pos_min, max=pos_max)

            # Compute and command a new velocity according to our current depth
            new_velocity = v(depth.value.depth)
            if new_velocity != velocity:
                velocity = new_velocity
                set_velocity(
                    velocity=rpm_ratio * velocity,  # RPM
                    acceleration=1200,  # RPM/s
                    torque=3.0          # rated torque
                )

            # Publish feedback about our current progress
            feedback = MoveToDepthFeedback()
            feedback.time_elapsed.data = elapsed
            feedback.depth = depth.value.depth
            feedback.velocity = velocity
            server.publish_feedback(feedback)

        # Record duration of movement
        elapsed_time = clock.now() - start_time
    except asyncio.TimeoutError as err:
        server.set_aborted(text=str(err))
        return
    finally:
        # However we broke out of the loop, stop the motor immediately
        rospy.logdebug('Loop finished, stopping motor')
        stop()  # blocking

        # Disable the position envelope
        set_position_envelope(min=0, max=0)

    # Send a success message
    result = MoveToDepthResult()
    result.uuid = str(uuid.uuid4())  # for linking with simultaneous profile msg
    result.time_elapsed.data = elapsed_time
    server.set_succeeded(result)

    # Suggest a new conversion factor
    if abs(start_depth - goal.depth) >= 1.0 and elapsed_time.to_sec() > 0:
        time_ratio = expected_time / elapsed_time
        rospy.set_param('~spool_circumference',
                        spool_circumference * time_ratio)
        rospy.loginfo(f'Adjusting RPM ratio from {rpm_ratio:.4f} to '
                      f'{rpm_ratio / time_ratio:.4f}')


async def main():
    # XXX Prefer asyncio.get_running_loop() on Python >= 3.7
    loop = asyncio.get_event_loop()

    # Initialize the ROS node
    rospy.init_node('winch', anonymous=True, disable_signals=True)

    # Register signal handlers the same way rospy would
    for sig in [signal.SIGTERM, signal.SIGINT]:
        loop.add_signal_handler(sig, rospy.signal_shutdown, f'signal-{sig.value}')

    # Wait for services to become available
    for svc in [ zero_position, set_velocity, set_position_envelope, stop ]:
        svc.wait_for_service()

    # Reset the position encoder to zero
    zero_position()

    # Subscribe to incoming messages
    global clock, depth, motor
    clock = AwaitableRosClock(loop)
    depth = WinchStatus(clock)
    motor = WinchStatus(clock)
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

    # Wake main() from the rospy shutdown thread so asyncio can exit cleanly.
    shutdown = asyncio.Event()
    rospy.core.add_preshutdown_hook(
        lambda reason: loop.call_soon_threadsafe(shutdown.set)
    )
    if not rospy.core.is_shutdown_requested():
        await shutdown.wait()


if __name__ == '__main__':
    asyncio.run(main())
