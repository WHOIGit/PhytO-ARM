#!/usr/bin/env python3
import asyncio
import ctypes
import math
import os
import random
import signal
import time
import uuid

import rclpy

from threading import Event, Thread

from action_msgs.msg import GoalStatus
from ds_sensor_msgs.msg import DepthPressure
from jvl_motor_msgs.msg import Motion
from jvl_motor_msgs.srv import SetPositionEnvelopeCmd, SetVelocityCmd, StopCmd, ZeroCmd
from phyto_arm.queued_action import QueuedActionServer
from phyto_arm.sensor_subscription import subscribe_sensor
from phyto_arm_msgs.action import MoveToDepth
from phyto_arm_msgs.msg import MoveToDepthFinished, MoveToDepthStarted
from phyto_sim_utils.parameters import declare_number
from rclpy.clock import Clock, ClockType
from rclpy.context import Context
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.parameter import Parameter
from scipy.integrate import quad


# COMMAND LINE NOTE
#
# ros2 action send_goal /winch/move_to_depth phyto_arm_msgs/action/MoveToDepth \
#     '{depth: 2.0, velocity: 0.05}'


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


# Global variables for the last known depth and motor readings
depth = None
motor = None


# Service clients for interacting with the motor
zero_position = set_position_envelope = set_velocity = stop = None


def call_motor(client, **fields):
    ready = Event()
    future = client.call_async(client.srv_type.Request(**fields))
    future.add_done_callback(lambda _: ready.set())
    deadline = time.monotonic() + 10.0
    while not ready.wait(0.1):
        if not client.service_is_ready():
            future.cancel()
            raise RuntimeError(f'Motor service {client.srv_name!r} disappeared')
        if time.monotonic() >= deadline:
            future.cancel()
            raise TimeoutError(f'Service call to {client.srv_name!r} timed out')
    response = future.result()
    if not response.success:
        raise RuntimeError(f'{client.srv_name}: {response.message}')
    return response


def abort(node, server, text):
    node.get_logger().error(text)
    server.finish(GoalStatus.STATUS_ABORTED, text=text)


# This function returns a velocity function with the given parameters.
#
# This is a sigmoid function ranging from - to +max_speed with zero at the
# target distance, reaching 1/2*max_speed at half_speed_dist from target.
#
# The smaller the value of half_speed_dist, the more abruptly we will stop.
def velocity_f(target, max_speed, half_speed_dist):
    scale = math.atanh(0.5) / half_speed_dist
    return lambda x: max_speed * math.tanh((target - x) * scale)


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
    return Duration(seconds=t)


# This is our entrypoint to the MoveToDepth action, which has some additional
# guardrails to prevent a runaway motor.
async def move_to_depth_chk(node, server):
    try:
        await move_to_depth(node, server, server.handle.request)
    except:
        abort(node, server, 'An exception occurred')
        raise
    finally:
        # No matter what happened, the motor should be stopped by now
        call_motor(stop)  # blocking


async def move_to_depth(node, server, goal):
    # Put initial status tasks in a lookup dict for traceability
    tasks = {
        asyncio.create_task(depth.wait()): "depth status",
        asyncio.create_task(motor.wait()): "motor status"
    }

    # Get an initial depth fix and motor status message
    _, pending = await asyncio.wait(tasks.keys(), timeout=2.0, return_when=asyncio.ALL_COMPLETED)

    if pending:
        err_text = f'Timed out waiting for {", ".join([tasks[p] for p in pending])}'
        abort(node, server, err_text)
        return

    # Safety check: The depth reading should be valid
    if depth.value.depth == DepthPressure.DEPTH_PRESSURE_NO_DATA:
        abort(node, server, 'Depth reading from CTD is invalid')
        return

    # Safety check: The motor should not be in motion
    if motor.value.mode != Motion.MODE_PASSIVE:
        abort(node, server, f'Motor is already in motion, mode: {motor.value.mode}')
        return

    # Safety check: Try to stop the motor (synchronously). This is a no-op but
    # if there's an error, it spares us from being unable to stop it later.
    call_motor(stop)  # blocking

    # Save our initial depth and compute depth limits
    start_depth = depth.value.depth
    depth_margin = node.get_parameter('safety_envelopes.position').value
    depth_min = min(start_depth, goal.depth) - depth_margin  # m
    depth_max = max(start_depth, goal.depth) + depth_margin  # m

    node.get_logger().debug(f'Starting depth is {start_depth:.2f} m')
    node.get_logger().info('Setting depth envelope to ' +
                           f'({depth_min:.2f}, {depth_max:.2f}) m')

    # Velocity function
    max_speed = node.get_parameter('max_speed').value
    half_speed_dist = node.get_parameter('half_speed_dist').value

    # Assert both are not None
    assert max_speed is not None and half_speed_dist is not None, 'Winch speed config invalid'

    # ROS rounds goal.velocity to float32 on the wire. Round the configured
    # limit too, so requesting exactly the maximum speed passes the check.
    max_speed = ctypes.c_float(max_speed).value
    assert goal.velocity <= max_speed, 'Goal velocity exceeds max speed'
    v = velocity_f(goal.depth, goal.velocity, half_speed_dist)

    # Estimate the time it should take to reach the destination
    epsilon = node.get_parameter('epsilon').value
    assert epsilon is not None, 'Winch epsilon config invalid'
    expected_time = estimate_time(v, start_depth, goal.depth, epsilon)
    node.get_logger().info(f'Estimated movement time is {expected_time.nanoseconds * 1e-9:.0f} s')

    # Set a time limit based on the estimate
    time_limit_ns = max(int(1.10 * expected_time.nanoseconds),
                        expected_time.nanoseconds + 10_000_000_000)

    node.get_logger().info(f'Setting time limit to {time_limit_ns * 1e-9:.0f} s')

    # Compute the RPM to m/s ratio
    gear_ratio = node.get_parameter('gear_ratio').value
    spool_circumference = node.get_parameter('spool_circumference').value
    rpm_ratio = 60 * gear_ratio / spool_circumference

    # Create a function to convert distances to encoder counts
    dist2encoder = lambda d: int(round(8192 * gear_ratio * d / spool_circumference))

    # Set some position bounds on the motor itself
    node.get_logger().debug(f'Current motor position is {motor.value.position:.2f}')

    if goal.depth < start_depth:
        lower_bound = motor.value.position - \
            dist2encoder((start_depth - goal.depth) + depth_margin)
        upper_bound = motor.value.position + dist2encoder(depth_margin)
    else:
        lower_bound = motor.value.position - dist2encoder(depth_margin)
        upper_bound = motor.value.position + \
            dist2encoder((goal.depth - start_depth) + depth_margin)

    if lower_bound < -(2**24) + 1 or upper_bound > 2**24 - 1:
        abort(node, server, 'Encoder position could wrap -- reset offset')
        return

    node.get_logger().info(f'Motor position envelope is ({lower_bound:.2f}, {upper_bound:.2f})')

    # Record the time that we start moving
    start_time = node.get_clock().now()

    success, started = False, False
    while True:
        elapsed = node.get_clock().now() - start_time

        # If we reached our target, stop
        if abs(depth.value.depth - goal.depth) < epsilon:
            node.get_logger().info('Reached target!')
            success = True
            break

        # If preempted, stop
        if server.preempt_requested:
            node.get_logger().info('Goal was preempted')
            server.finish(GoalStatus.STATUS_CANCELED, text='Goal was preempted')
            break

        # If we exceed the programmed position envelope, the motor will stop
        # automatically. Take this as a sign something bad happened.
        if started and motor.value.mode != Motion.MODE_VELOCITY:
            abort(node, server, f'Motor unexpectedly stopped, mode: {motor.value.mode}')
            if not (pos_min < motor.value.position < pos_max):
                node.get_logger().error(
                    'Position envelope exceeded -- '
                    f'{motor.value.position} not in ({pos_min, pos_max})'
                )
            break
        elif not started and motor.value.mode == Motion.MODE_VELOCITY:
            # We started moving, so turn on the check above
            started = True

        # If too much time has elapsed, stop
        if elapsed.nanoseconds > time_limit_ns:
            abort(node, server, f'Time limit exceeded: {elapsed.nanoseconds}/{time_limit_ns}')
            break

        # If depth is outside of bounds, stop
        if not (depth_min <= depth.value.depth <= depth_max):
            abort(node, server, f'Exceeded depth bounds: {depth.value.depth} not in ({depth_min}, {depth_max})')
            break

        # Update bounds so we do not travel far from this position
        pos_margin = dist2encoder(depth_margin)
        pos_min = max(lower_bound, motor.value.position - pos_margin)
        pos_max = min(upper_bound, motor.value.position + pos_margin)
        call_motor(set_position_envelope, min=pos_min, max=pos_max)

        # Compute and command a new velocity according to our current depth
        velocity = v(depth.value.depth)
        rpm = rpm_ratio * velocity

        call_motor(
            set_velocity,
            velocity=rpm,         # RPM
            acceleration=1200.0,  # RPM/s
            torque=3.0            # rated torque
        )

        # Publish feedback about our current progress
        feedback = MoveToDepth.Feedback()
        feedback.time_elapsed = elapsed.to_msg()
        feedback.depth = depth.value.depth
        feedback.velocity = velocity
        server.handle.publish_feedback(feedback)

        await asyncio.sleep(1.0)

    # Record duration of movement
    elapsed_time = node.get_clock().now() - start_time

    # However we broke out of the loop, stop the motor immediately
    node.get_logger().debug('Loop finished, stopping motor')
    call_motor(stop)  # blocking

    # Disable the position envelope
    call_motor(set_position_envelope, min=0, max=0)

    # If we fell out of the loop unsuccessfully for any reason, end here
    if not success:
        if not server.result.done():
            abort(node, server, 'Failed for unknown reason')
        return

    # Send a success message
    result = MoveToDepth.Result()
    result.uuid = str(uuid.uuid4())  # for linking with simultaneous profile msg
    result.time_elapsed = elapsed_time.to_msg()
    server.finish(GoalStatus.STATUS_SUCCEEDED, result)

    # Suggest a new conversion factor
    if abs(start_depth - goal.depth) >= 1.0:
        time_ratio = expected_time.nanoseconds / elapsed_time.nanoseconds
        node.set_parameters([Parameter('spool_circumference', value=spool_circumference * time_ratio)])
        node.get_logger().info(f'Adjusting RPM ratio from {rpm_ratio:.4f} to '
                               f'{rpm_ratio / time_ratio:.4f}')


async def create_action_server(node, loop, shutdown):
    node.declare_parameter('ctd_topic', Parameter.Type.STRING)

    for name in ('gear_ratio', 'half_speed_dist', 'max_speed', 'spool_circumference'):
        declare_number(node, name)

    declare_number(node, 'epsilon', 0.01)
    declare_number(node, 'safety_envelopes.position', 0.1)

    # Initialize our AwaitableValues with the event loop reference
    global depth, motor
    depth = AwaitableValue(loop, None)
    motor = AwaitableValue(loop, None)

    global zero_position, set_velocity, set_position_envelope, stop
    zero_position = node.create_client(ZeroCmd, 'motor/zero_position')
    set_velocity = node.create_client(SetVelocityCmd, 'motor/set_velocity')
    set_position_envelope = node.create_client(SetPositionEnvelopeCmd, 'motor/set_position_envelope')
    stop = node.create_client(StopCmd, 'motor/stop')

    # Wait for services to become available
    for client in [zero_position, set_velocity, set_position_envelope, stop]:
        while not client.service_is_ready():
            if shutdown.is_set():
                return
            await asyncio.sleep(0.1)

    # Reset the position encoder to zero
    call_motor(zero_position)

    # Subscribe to incoming messages
    sensors = {node.get_parameter('ctd_topic').value: (DepthPressure, depth.update_soon),
               'motor/motion': (Motion, motor.update_soon)}

    def discover():
        for topic, (msg_type, callback) in list(sensors.items()):
            if subscribe_sensor(node, topic, callback, msg_type) is not None:
                del sensors[topic]
        if not sensors:
            timer.cancel()

    timer = node.create_timer(0.1, discover, clock=Clock(clock_type=ClockType.STEADY_TIME))

    action = 'winch/move_to_depth'
    started = node.create_publisher(MoveToDepthStarted, action + '/goal', 50)
    finished = node.create_publisher(MoveToDepthFinished, action + '/result', 50)

    def on_accept(handle):
        started.publish(MoveToDepthStarted(goal_info=handle._goal_info,
                                           depth=handle.request.depth, velocity=handle.request.velocity))

    def on_finish(handle, result, text):
        status = GoalStatus(goal_info=handle._goal_info, status=handle.status)
        finished.publish(MoveToDepthFinished(stamp=node.get_clock().now().to_msg(), status=status,
                                             text=text, uuid=result.uuid, time_elapsed=result.time_elapsed))

    def execute(goal):
        if shutdown.is_set():
            goal.finish(GoalStatus.STATUS_ABORTED)
            return
        asyncio.run_coroutine_threadsafe(move_to_depth_chk(node, goal), loop).result()

    return QueuedActionServer(node, MoveToDepth, action, execute, on_accept=on_accept, on_finish=on_finish)


async def main():
    loop = asyncio.get_running_loop()
    shutdown = asyncio.Event()
    context = Context()
    rclpy.init(context=context)
    node = rclpy.create_node(f'winch_{os.getpid()}_{random.getrandbits(32):08x}', context=context)
    for sig in (signal.SIGTERM, signal.SIGINT):
        loop.add_signal_handler(sig, shutdown.set)

    executor = MultiThreadedExecutor(context=context)
    executor.add_node(node)
    thread = Thread(target=executor.spin)
    thread.start()
    server = None
    try:
        server = await create_action_server(node, loop, shutdown)
        await shutdown.wait()
    finally:
        # Finish stopping the motor while the service clients are still available.
        if server is not None:
            with server.lock:
                if server.current is not None:
                    server.current.preempt_requested = True
                worker = server.worker
            if worker is not None:
                await loop.run_in_executor(None, worker.join)
        executor.shutdown()
        thread.join()
        if server is not None:
            server.server.destroy()
        node.destroy_node()
        rclpy.try_shutdown(context=context)


if __name__ == '__main__':
    asyncio.run(main())
