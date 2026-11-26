#!/usr/bin/env python3
import math

import numpy as np
import rclpy

from datetime import datetime, timedelta
from threading import Event

from phyto_arm.arm_base import ArmBase, Task, arm_node
from phyto_arm_msgs.action import RunIFCB
from phyto_arm_msgs.msg import DepthProfile
from phyto_sim_utils.parameters import declare_number
from rclpy.action import ActionClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.clock import ClockType
from rclpy.duration import Duration
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time
from std_msgs.msg import Bool


class ArmIFCB(ArmBase):
    profile_activity = Event()
    latest_profile = None

    scheduled_depths = []
    last_scheduled_time = None
    next_scheduled_index = 0

    profiler_peak_depth = None
    profiler_peak_value = None

    last_cart_debub_time = None
    last_bead_time = None

    ifcb_connected = False
    ifcb_connection_event = Event()

    # Primary method for determining the next task to execute
    # Each Task object takes:
    #  - name: a string identifying the task
    #  - callback: a function to call when the task is complete
    #  - depth: the depth to move to (optional, won't move if not provided)
    #  - speed: the speed to move at (optional, will use config max if not provided)
    def get_next_task(self, last_task):
        # Check if we should pause tasks when IFCB is disconnected
        pause_on_disconnect = self.node.get_parameter_or('tasks.pause_tasks_until_connected').value
        if pause_on_disconnect and not self.ifcb_connected:
            self.node.get_logger().warning('IFCB is disconnected. Waiting for connection...')
            return Task('await_ifcb_connection', await_ifcb_connection)

        if not self.node.get_parameter('winch_enabled').value:
            return Task('no_winch', handle_nowinch)

        # If close to scheduled wiz time, prioritize first
        if its_wiz_time():
            wiz_depth = compute_wiz_depth(self.profiler_peak_depth)
            return Task('wiz_probe', lambda _: await_wiz_probe(self.start_next_task), wiz_depth)

        # Otherwise, start off at min
        preupcast_tasks = ['scheduled_depth', 'profiler_peak_depth', 'wiz_probe', 'await_ifcb_connection']
        if last_task is None or last_task.name in preupcast_tasks:
            return Task('upcast', self.start_next_task, self.node.get_parameter('winch.range.min').value)

        # Then perform a downcast to get a full profile
        if last_task.name == 'upcast':
            return Task('downcast', handle_downcast, self.node.get_parameter('winch.range.max').value)

        # Then go to peak depth, unless it's scheduled depth time
        if last_task.name == 'downcast':
            if its_scheduled_depth_time(self.profiler_peak_value):
                return Task('scheduled_depth', handle_target_depth, scheduled_depth())
            if self.profiler_peak_depth is not None:
                return Task('profiler_peak_depth', handle_target_depth, self.profiler_peak_depth)

            # If we don't have a peak depth, we can't do anything, so just run scheduled depth
            return Task('scheduled_depth', handle_target_depth, scheduled_depth())

        raise ValueError(f'Unhandled next-task state where last task={last_task.name}')


# Global reference to action provided by other node
ifcb_runner = None

# Global reference to arm state
arm = None


def on_profile_msg(msg):
    # Alert waiting threads to the new profile message
    arm.latest_profile = msg
    arm.profile_activity.set()
    arm.profile_activity.clear()


def on_ifcb_connection_status(msg):
    arm.ifcb_connected = msg.data
    if arm.ifcb_connected:
        arm.node.get_logger().info('IFCB connection established')
        arm.ifcb_connection_event.set()
        arm.ifcb_connection_event.clear()
    else:
        arm.node.get_logger().warning('IFCB connection lost')


def await_ifcb_connection():
    while not arm.ifcb_connected and rclpy.ok(context=arm.node.context):
        arm.node.get_logger().info('Waiting for IFCB connection...')
        arm.ifcb_connection_event.wait(30)

    if arm.ifcb_connected:
        arm.node.get_logger().info('IFCB connection restored, resuming tasks')
        arm.start_next_task()

# Convert HH:MM to a ROS timestamp. Take into account the current time,
# pushing the requested time to the next day if it's already passed.
def wiz_to_rostime(hhmm_time):
    today_time = datetime.now().time()

    # Convert HH:MM to a timestamp
    wiz_time = datetime.strptime(hhmm_time, '%H:%M').time()
    wiz_date = datetime.now().date()

    # wiz_time should always come after the current time, even if it's the next day
    if wiz_time < today_time:
        wiz_date = datetime.now().date() + timedelta(days=1)

    unix_timestamp = datetime.combine(wiz_date, wiz_time).timestamp()

    return Time(seconds=unix_timestamp, clock_type=ClockType.ROS_TIME)


def find_next_wiz_time():
    # Get array of HH:MM times
    wiz_probe_times = arm.node.get_parameter('tasks.wiz_probe.times').value

    # Find the next time
    next_time = None
    for wiz_time in wiz_probe_times:
        wiz_time = wiz_to_rostime(wiz_time)

        # wiz_time will always be in the future, just find the earliest one
        if next_time is None or wiz_time < next_time:
            next_time = wiz_time

    return next_time


# Check if it's time to run the wiz probe routine
def its_wiz_time():
    prep_window = Duration(seconds=60*arm.node.get_parameter('tasks.wiz_probe.preparation_window').value)

    # If we're within the preparation window, it's wizin' time
    return (find_next_wiz_time() - arm.node.get_clock().now()) <= prep_window


def compute_wiz_depth(peak_depth):
    wiz_depth = arm.node.get_parameter('tasks.wiz_probe.default_depth').value
    if arm.node.get_parameter('tasks.wiz_probe.use_profiler_peak').value:
        if peak_depth is not None:
            wiz_depth = peak_depth + arm.node.get_parameter('tasks.wiz_probe.peak_offset').value
        else:
            arm.node.get_logger().warning('No profiler peak depth available, using default wiz probe depth')
            wiz_depth = arm.node.get_parameter('tasks.wiz_probe.default_depth').value

    # Check that adjustments haven't caused us to exceed bounds. Go with bound if so
    if wiz_depth > arm.node.get_parameter('winch.range.max').value:
        arm.node.get_logger().warning('Preferred wiz probe depth exceeds max depth, using max depth')
        wiz_depth = arm.node.get_parameter('winch.range.max').value
    if wiz_depth < arm.node.get_parameter('winch.range.min').value:
        arm.node.get_logger().warning('Preferred wiz probe depth is less than min depth, using min depth')
        wiz_depth = arm.node.get_parameter('winch.range.min').value

    return wiz_depth


def await_wiz_probe(callback):
    # Find the next time to run the wiz probe
    next_time = find_next_wiz_time()

    # Ensure we're still in the preparation window
    prep_window = Duration(seconds=60*arm.node.get_parameter('tasks.wiz_probe.preparation_window').value)
    preptime_remaining = next_time - arm.node.get_clock().now()
    if preptime_remaining > prep_window:
        raise ValueError(f'Next wiz time is {preptime_remaining.nanoseconds / 1e9}s away, should be less than \
                          {prep_window.nanoseconds / 1e9}s. Preparation window likely too short.')

    # Wait for the rest of the window + the wiz probe duration
    arm.node.get_logger().info('Waiting for wiz probe to finish')
    wiz_duration = 60*arm.node.get_parameter('tasks.wiz_probe.duration').value
    if arm.stopping.wait(wiz_duration + preptime_remaining.nanoseconds / 1e9):
        return

    # TODO: Run IFCB while waiting

    arm.node.get_logger().info('Wiz probe wait time complete')
    callback()


def its_scheduled_depth_time(peak_value):
    # Decide if we want to use the scheduled depth
    scheduled_interval = Duration(seconds=60*arm.node.get_parameter('tasks.scheduled_depth.every').value)

    run_schedule = False
    if math.isclose(scheduled_interval.nanoseconds / 1e9, 0.0):  # disabled
        run_schedule = False
    elif arm.last_scheduled_time is None:
        run_schedule = True
    elif arm.node.get_clock().now() - arm.last_scheduled_time > scheduled_interval:
        run_schedule = True
    elif peak_value is not None and peak_value < arm.node.get_parameter('tasks.profiler_peak.threshold').value:
        run_schedule = True
    return run_schedule


def scheduled_depth():
    scheduled_depths = np.linspace(
        arm.node.get_parameter('tasks.scheduled_depth.range.first').value,
        arm.node.get_parameter('tasks.scheduled_depth.range.last').value,
        arm.node.get_parameter('tasks.scheduled_depth.range.count').value,
    )

    target_depth = scheduled_depths[arm.next_scheduled_index % scheduled_depths.shape[0]]

    arm.node.get_logger().info(f'Using scheduled depth of {target_depth:.2f} m')

    arm.next_scheduled_index += 1
    arm.last_scheduled_time = arm.node.get_clock().now()
    return target_depth


def send_ifcb_action():
    ready = Event()
    goal = ifcb_runner.send_goal_async(RunIFCB.Goal())
    goal.add_done_callback(lambda _: ready.set())
    ready.wait()
    handle = goal.result()
    if handle.accepted:
        handle.get_result()


def handle_downcast(move_result):
    # Wait for the profile data for this cast
    while arm.latest_profile is None or \
        arm.latest_profile.goal_uuid != move_result.uuid:
        notified = arm.profile_activity.wait(60)

        # For short downcasts profiling will likely fail; switch to scheduled depth
        if not notified:
            arm.node.get_logger().warning('No profile data received')
            arm.profiler_peak_depth = None
            arm.profiler_peak_value = None
            arm.start_next_task()
            return

    # Find the maximal value in the profile
    argmax = max(range(len(arm.latest_profile.values)), key=lambda i: arm.latest_profile.values[i])
    arm.profiler_peak_value = arm.latest_profile.values[argmax]
    arm.profiler_peak_depth = arm.latest_profile.depths[argmax]

    arm.node.get_logger().info(f'Profile peak is {arm.profiler_peak_value:.2f} at {arm.profiler_peak_depth:.2f} m')
    arm.start_next_task()


def handle_target_depth(move_result):
    send_ifcb_action()
    arm.start_next_task()


def handle_nowinch():
    send_ifcb_action()
    arm.start_next_task()


def main():
    global arm
    global ifcb_runner
    with arm_node('arm_ifcb') as node:
        node.declare_parameters('', [
            ('tasks.pause_tasks_until_connected', Parameter.Type.BOOL),
            ('tasks.scheduled_depth.range.count', Parameter.Type.INTEGER),
            ('tasks.wiz_probe.times', Parameter.Type.STRING_ARRAY),
            ('tasks.wiz_probe.use_profiler_peak', Parameter.Type.BOOL),
        ])

        for name in (
            'tasks.profiler_peak.threshold', 'tasks.scheduled_depth.every',
            'tasks.scheduled_depth.range.first', 'tasks.scheduled_depth.range.last',
            'tasks.wiz_probe.default_depth', 'tasks.wiz_probe.duration',
            'tasks.wiz_probe.peak_offset', 'tasks.wiz_probe.preparation_window'
        ):
            declare_number(node, name)

        winch_name = None
        if node.get_parameter_or('winch_enabled').value:
            winch_name = 'winch/move_to_depth'
        arm = ArmIFCB(node, winch_name)

        # Subscribe to profiler messages that follow each transit
        node.create_subscription(DepthProfile, 'profiler', on_profile_msg, 10)

        # Subscribe to IFCB connection status
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        node.create_subscription(Bool, '/ifcb/connected', on_ifcb_connection_status, latched)

        # Setup action client for running IFCB
        ifcb_runner = ActionClient(node, RunIFCB, 'ifcb_runner/sample',
                                   callback_group=MutuallyExclusiveCallbackGroup())
        node.get_logger().info(f'Arm {arm.arm_name} awaiting IFCB-run action server')
        ifcb_runner.wait_for_server()
        node.get_logger().info(f'Arm {arm.arm_name} IFCB-run action server acquired')

        # Set a fake timestamp for having run beads and catridge debubble, so that
        # we don't run it every startup and potentially waste time or bead supply.
        arm.last_cart_debub_time = node.get_clock().now()
        arm.last_bead_time = node.get_clock().now()

        arm.loop()


if __name__ == '__main__':
    main()
