#!/usr/bin/env python3
import asyncio
import math
import signal
import uuid
from concurrent.futures import ThreadPoolExecutor
from contextlib import asynccontextmanager, contextmanager
from functools import partial

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

    def update(self, value, stamp=None):
        self.value = value
        self.received = self.now() if stamp is None else stamp
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

    def update(self, value, stamp=None):
        deadline = self.deadline
        super().update(value, stamp)
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


# Forward each observation to the move before the next sample can replace it
class WinchReadings:
    def __init__(self, clock):
        self.clock = clock
        self.depth = WinchStatus(clock)
        self.motor = WinchStatus(clock)
        self.move = None

    # In simulation the loop processes each reading in order with /clock ticks
    def subscribe(self, topic, message_type, update):
        def receive(msg):
            stamp = self.clock.now() if self.clock.stamp is None else None
            self.clock.loop.call_soon_threadsafe(update, msg, stamp)

        return rospy.Subscriber(topic, message_type, receive)

    def update_depth(self, msg, stamp=None):
        self.depth.update(msg, stamp)
        if self.move is not None:
            self.move.observe_depth(msg.depth)

    def update_motor(self, msg, stamp=None):
        self.motor.update(msg, stamp)
        if self.move is not None:
            self.move.observe_motor(msg.mode, msg.position)


readings = None


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


# Keep the limits and observations for one move, in the order they arrive
class DepthMove:
    def __init__(self, target, speed, depth, position, *, epsilon, gear_ratio,
                 half_speed_dist, margin, spool_circumference):
        self.start_depth = depth
        self.target = target
        self.epsilon = epsilon
        self.speed_at = velocity_f(target, speed, half_speed_dist)
        self.expected_time = rospy.Duration(0)
        if abs(depth - target) >= epsilon:
            self.expected_time = estimate_time(self.speed_at, depth, target,
                                               epsilon)
        self.time_limit = max(1.10 * self.expected_time,
                              self.expected_time + rospy.Duration(10))

        # Convert line travel to motor revolutions and encoder counts
        turns_per_metre = gear_ratio / spool_circumference
        self.rpm_ratio = 60 * turns_per_metre
        self.spool_circumference = spool_circumference

        def counts(distance):
            return round(8192 * gear_ratio * distance / spool_circumference)

        # Limit total travel and the distance allowed from one encoder reading
        self.depth_min = min(depth, target) - margin
        self.depth_max = max(depth, target) + margin
        self.lower = position + counts(min(0, target - depth) - margin)
        self.upper = position + counts(max(0, target - depth) + margin)
        self.margin = counts(margin)
        if self.lower < -(2**24) + 1 or self.upper > 2**24 - 1:
            raise ValueError('Encoder position could wrap -- reset offset')

        self.error = None
        self.last_position = None
        self.position = position
        self.reached = False
        self.started = False
        self.observe_depth(depth)

    def observe_depth(self, depth):
        self.depth = depth
        self.fresh_depth = True
        if not self.depth_min <= depth <= self.depth_max:
            self.error = self.error or (
                f'Exceeded depth bounds: {depth} not in '
                f'({self.depth_min}, {self.depth_max})')
        if abs(depth - self.target) < self.epsilon:
            self.reached = True

    def observe_motor(self, mode, position):
        self.position = position
        if mode == Motion.MODE_VELOCITY:
            self.started = True
        elif self.started and self.error is None:
            self.error = f'Motor unexpectedly stopped, mode: {mode}'
            if self.last_position is not None:
                low, high = self.envelope
                if not low < position < high:
                    self.error += (
                        f', position envelope exceeded: {position} '
                        f'not in ({low}, {high})')

    # Advance the envelope once per fresh depth and changed encoder position
    def next_envelope(self):
        if not self.fresh_depth or self.position == self.last_position:
            return None
        self.fresh_depth = False
        self.last_position = self.position
        self.envelope = (max(self.lower, self.position - self.margin),
                         min(self.upper, self.position + self.margin))
        return self.envelope


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


# Enforce the move and sensor deadlines, including gaps during motor calls
class MoveWatchdog:
    def __init__(self, readings, duration):
        self.clock = readings.clock
        self.start = self.previous = self.clock.now()
        self.epoch = self.clock.epoch
        self.end = self.start + duration
        self.streams = (('depth', readings.depth, readings.depth.gaps),
                        ('motor', readings.motor, readings.motor.gaps))

    def next_updates(self):
        return [status.next_receipt for _, status, _ in self.streams]

    def check_deadlines(self):
        now = self.clock.now()
        if self.clock.epoch != self.epoch or now < self.previous:
            raise rospy.exceptions.ROSTimeMovedBackwardsException(
                max(self.clock.rewind_amount, (self.previous - now).to_sec()))
        self.previous = now

        deadline = self.end
        if now >= deadline:
            raise asyncio.TimeoutError(
                f'Time limit exceeded: {now - self.start}/'
                f'{self.end - self.start}')
        for name, status, gaps in self.streams:
            expires = status.deadline
            if now >= expires or status.gaps != gaps:
                raise asyncio.TimeoutError(
                    f'Timed out waiting for {name} status')
            deadline = min(deadline, expires)
        return deadline

    async def wait(self, updates):
        while not any(update.done() for update in updates):
            await self.clock.wait_for_change(self.check_deadlines(), updates)


# Serialize motor writes so cleanup follows any command already in flight
class MotorCommands:
    def __init__(self, worker, server):
        self.worker = worker
        self.server = server
        self.used_envelope = False

    def submit(self, service, **kwargs):
        loop = asyncio.get_running_loop()
        return loop.run_in_executor(
            self.worker, partial(service, **kwargs))

    async def set_envelope(self, limits):
        self.used_envelope = True
        await self.submit(set_position_envelope, min=limits[0], max=limits[1])

    async def set_speed(self, rpm):
        def write():
            if self.server.is_preempt_requested():
                raise asyncio.CancelledError
            set_velocity(velocity=rpm, acceleration=1200, torque=3.0)

        await self.submit(write)

    def finish(self):
        stop()
        if self.used_envelope:
            set_position_envelope(min=0, max=0)


# Wake a preempted move once and let its motor cleanup finish
@contextmanager
def cancel_on_preempt(server):
    loop = asyncio.get_running_loop()
    task = asyncio.current_task()

    def cancel():
        nonlocal task
        if task is not None:
            task.cancel()
            task = None

    server.register_preempt_callback(lambda: loop.call_soon_threadsafe(cancel))
    try:
        if server.is_preempt_requested():
            raise asyncio.CancelledError
        yield
    finally:
        task = None


# A goal always stops the motor before reporting its result
@asynccontextmanager
async def motor_commands(server):
    with ThreadPoolExecutor(max_workers=1) as worker:
        motor = MotorCommands(worker, server)
        try:
            yield motor
        finally:
            # Shutdown can interrupt cleanup already queued behind a write
            cleanup = motor.submit(motor.finish)
            interrupted = False
            while not cleanup.done():
                try:
                    await asyncio.shield(cleanup)
                except asyncio.CancelledError:
                    interrupted = True
            cleanup.result()
            if interrupted:
                raise asyncio.CancelledError


async def move_to_depth_chk(server, goal):
    try:
        async with motor_commands(server) as motor:
            with cancel_on_preempt(server):
                move, elapsed = await move_to_depth(server, goal, motor)
    except rospy.exceptions.ROSTimeMovedBackwardsException:
        server.set_aborted(text='ROS time moved backwards')
        return
    except asyncio.CancelledError:
        if not server.is_preempt_requested():
            raise
        server.set_preempted(text='Goal was preempted')
        return
    except (asyncio.TimeoutError, rospy.ServiceException, ValueError) as err:
        server.set_aborted(text=str(err))
        return

    result = MoveToDepthResult()
    result.uuid = str(uuid.uuid4())  # for linking with simultaneous profile msg
    result.time_elapsed.data = elapsed
    server.set_succeeded(result)

    # Suggest a new conversion factor
    if (abs(move.start_depth - goal.depth) >= 1.0 and
            move.expected_time.to_sec() > 0 and elapsed.to_sec() > 0):
        time_ratio = move.expected_time / elapsed
        rospy.set_param('~spool_circumference',
                        move.spool_circumference * time_ratio)
        rospy.loginfo(f'Adjusting RPM ratio from {move.rpm_ratio:.4f} to '
                      f'{move.rpm_ratio / time_ratio:.4f}')


# Validate the starting conditions and read the limits for this goal
def prepare_move(goal, depth, motor):
    if depth == DepthPressure.DEPTH_PRESSURE_NO_DATA:
        raise ValueError('Depth reading from CTD is invalid')
    if motor.mode != Motion.MODE_PASSIVE:
        raise ValueError(f'Motor is already in motion, mode: {motor.mode}')
    if not goal.velocity < rospy.get_param('~max_speed'):
        raise ValueError('Goal velocity exceeds max speed')

    move = DepthMove(
        goal.depth, goal.velocity, depth, motor.position,
        epsilon=rospy.get_param('~epsilon', 0.01),
        gear_ratio=rospy.get_param('~gear_ratio'),
        half_speed_dist=rospy.get_param('~half_speed_dist'),
        margin=rospy.get_param('~safety_envelopes').get('position', 0.1),
        spool_circumference=rospy.get_param('~spool_circumference'))
    rospy.logdebug(f'Starting depth is {depth:.2f} m')
    rospy.loginfo(f'Setting depth envelope to '
                  f'({move.depth_min:.2f}, {move.depth_max:.2f}) m')
    rospy.loginfo(f'Estimated movement time is '
                  f'{move.expected_time.to_sec():.0f} s')
    rospy.loginfo(f'Setting time limit to {move.time_limit.to_sec():.0f} s')
    rospy.loginfo(f'Motor position envelope is ({move.lower}, {move.upper})')
    return move


# Check every observation before issuing another command
async def drive_winch(server, move, motor, watchdog):
    def moving():
        if server.is_preempt_requested():
            raise asyncio.CancelledError
        watchdog.check_deadlines()
        if move.error:
            raise ValueError(move.error)
        return not move.reached

    velocity = None
    while moving():
        updates = watchdog.next_updates()
        limits = move.next_envelope()
        if limits is not None:
            await motor.set_envelope(limits)
            if not moving():
                break

        new_velocity = move.speed_at(move.depth)
        if new_velocity != velocity:
            await motor.set_speed(move.rpm_ratio * new_velocity)
            velocity = new_velocity
            if not moving():
                break

        # Publish feedback about our current progress
        feedback = MoveToDepthFeedback()
        feedback.time_elapsed.data = watchdog.clock.now() - watchdog.start
        feedback.depth = move.depth
        feedback.velocity = velocity
        server.publish_feedback(feedback)
        await watchdog.wait(updates)


async def move_to_depth(server, goal, motor):
    # Get an initial depth fix and motor status message
    try:
        await wait_for_updates(readings.clock, [readings.depth, readings.motor],
                               WinchStatus.timeout)
    except asyncio.TimeoutError:
        raise asyncio.TimeoutError('Timed out waiting for initial winch status')

    move = prepare_move(goal, readings.depth.value.depth, readings.motor.value)
    watchdog = MoveWatchdog(readings, move.time_limit)
    readings.move = move
    try:
        # Check communication with the stopped motor before enabling velocity
        await motor.submit(stop)
        await drive_winch(server, move, motor, watchdog)
        rospy.loginfo('Reached target!')
        return move, readings.clock.now() - watchdog.start
    finally:
        readings.move = None


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
    global readings
    readings = WinchReadings(AwaitableRosClock(loop))
    readings.subscribe(rospy.get_param('ctd_topic'), DepthPressure,
                       readings.update_depth)
    readings.subscribe('motor/motion', Motion, readings.update_motor)

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
