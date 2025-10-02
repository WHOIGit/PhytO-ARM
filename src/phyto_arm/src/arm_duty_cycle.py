#!/usr/bin/env python3
"""Duty cycle arm implementation for PhytO-ARM.

This module implements a duty cycle arm that turns the IFCB on and off based on
a configurable schedule while performing profiling and sampling tasks.
"""
import math

from datetime import datetime, timedelta
from threading import Event

import actionlib
import numpy as np
import rospy

from arm_base import ArmBase, Task
from phyto_arm.msg import DepthProfile, RunIFCBGoal, RunIFCBAction
from std_msgs.msg import Bool


class ArmDutyCycle(ArmBase):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

        # Profile tracking
        self.profile_activity = Event()
        self.latest_profile = None

        # Scheduled depth tracking
        self.scheduled_depths = []
        self.last_scheduled_time = None
        self.next_scheduled_index = 0

        # Profiler peak tracking
        self.profiler_peak_depth = None
        self.profiler_peak_value = None

        # Maintenance tracking
        self.last_cart_debub_time = None
        self.last_bead_time = None

        # IFCB connection tracking
        self.ifcb_connected = False
        self.ifcb_connection_event = Event()

        # Duty cycle state tracking
        self.is_sampling_active = False
        self.last_duty_check_time = None
        self.next_duty_transition_time = None

    def get_next_task(self, last_task):
        # Check if we should pause tasks when IFCB is disconnected
        pause_on_disconnect = rospy.get_param('tasks/pause_tasks_until_connected', False)
        if pause_on_disconnect and not self.ifcb_connected:
            rospy.logwarn('IFCB is disconnected. Waiting for connection...')
            return Task('await_ifcb_connection', await_ifcb_connection)

        # Check duty cycle schedule
        if self.should_transition_duty_state():
            if self.is_sampling_active:
                # Time to shut down
                return Task('shutdown_sampling', shutdown_sampling)
            else:
                # Time to start up
                return Task('startup_sampling', startup_sampling)

        # If we're not supposed to be sampling, just wait
        if not self.is_sampling_active:
            return Task('await_duty_cycle', await_duty_cycle)

        if not rospy.get_param('winch_enabled'):
            return Task('no_winch', handle_nowinch)

        # Start off at min depth for upcast
        preupcast_tasks = ['scheduled_depth', 'profiler_peak_depth', 'await_ifcb_connection',
                          'startup_sampling', 'await_duty_cycle']
        if last_task is None or last_task.name in preupcast_tasks:
            return Task('upcast', self.start_next_task, rospy.get_param('winch/range/min'))

        # Then perform a downcast to get a full profile
        if last_task.name == 'upcast':
            return Task('downcast', handle_downcast, rospy.get_param('winch/range/max'))

        # Then go to peak depth, unless it's scheduled depth time
        if last_task.name == 'downcast':
            if its_scheduled_depth_time(self.profiler_peak_value):
                return Task('scheduled_depth', handle_target_depth, scheduled_depth(self))
            if self.profiler_peak_depth is not None:
                return Task('profiler_peak_depth', handle_target_depth, self.profiler_peak_depth)

            # If we don't have a peak depth, we can't do anything, so just run scheduled depth
            return Task('scheduled_depth', handle_target_depth, scheduled_depth(self))

        raise ValueError(f'Unhandled next-task state where last task={last_task.name}')

    def should_transition_duty_state(self):
        """Check if it's time to transition between sampling and non-sampling states"""
        current_time = rospy.Time.now()

        # Initialize duty cycle tracking if needed
        if self.last_duty_check_time is None:
            self.last_duty_check_time = current_time
            self.update_duty_cycle_state()
            return False

        # Check if we've reached the next transition time
        if self.next_duty_transition_time and current_time >= self.next_duty_transition_time:
            return True

        return False

    def update_duty_cycle_state(self):
        """Update the duty cycle state based on current schedule"""
        current_time = datetime.now()
        schedule = rospy.get_param('tasks/duty_cycle/schedule', [])

        if not schedule:
            # No schedule defined, always sample
            self.is_sampling_active = True
            self.next_duty_transition_time = None
            return

        # Find current and next schedule entries
        current_entry = None
        next_entry = None

        # Find next entry (future schedule entry)
        for entry in schedule:
            entry_time = datetime.strptime(entry['time'], '%H:%M').time()
            entry_datetime = datetime.combine(current_time.date(), entry_time)

            # Adjust for next day if time has passed
            if entry_datetime <= current_time:
                entry_datetime += timedelta(days=1)

            if next_entry is None:
                next_entry = {'datetime': entry_datetime, 'active': entry['active']}
            elif entry_datetime < next_entry['datetime']:
                next_entry = {'datetime': entry_datetime, 'active': entry['active']}

        # Find the most recent entry that applies to current time
        for entry in schedule:
            entry_time = datetime.strptime(entry['time'], '%H:%M').time()
            entry_datetime = datetime.combine(current_time.date(), entry_time)

            # Check today and yesterday
            for days_back in [0, 1]:
                check_datetime = entry_datetime - timedelta(days=days_back)
                if check_datetime <= current_time:
                    if current_entry is None:
                        current_entry = {'datetime': check_datetime, 'active': entry['active']}
                    elif check_datetime > current_entry['datetime']:
                        current_entry = {'datetime': check_datetime, 'active': entry['active']}

        # Update state
        if current_entry:
            self.is_sampling_active = current_entry['active']
        else:
            # Default to inactive if no schedule entry found
            self.is_sampling_active = False

        if next_entry:
            self.next_duty_transition_time = rospy.Time.from_sec(next_entry['datetime'].timestamp())
        else:
            self.next_duty_transition_time = None

        rospy.loginfo(f'Duty cycle state: sampling_active={self.is_sampling_active}, '
                     f'next_transition={self.next_duty_transition_time}')


# Global reference to action provided by other node
ifcb_runner = None

# Global reference to arm state
arm = None

# DL outlet publishers
dl_ifcb_pub = None


def on_profile_msg(msg):
    """Handle incoming profile messages.

    Args:
        msg: The DepthProfile message
    """
    # Alert waiting threads to the new profile message
    arm.latest_profile = msg
    arm.profile_activity.set()
    arm.profile_activity.clear()


def on_ifcb_connection_status(msg):
    """Handle IFCB connection status updates.

    Args:
        msg: Bool message indicating connection status
    """
    arm.ifcb_connected = msg.data
    if arm.ifcb_connected:
        rospy.loginfo('IFCB connection established')
        arm.ifcb_connection_event.set()
        arm.ifcb_connection_event.clear()
    else:
        rospy.logwarn('IFCB connection lost')


def await_ifcb_connection():
    """Wait for IFCB connection to be established."""
    while not arm.ifcb_connected and not rospy.is_shutdown():
        rospy.loginfo('Waiting for IFCB connection...')
        arm.ifcb_connection_event.wait(30)

    if arm.ifcb_connected:
        rospy.loginfo('IFCB connection restored, resuming tasks')
        arm.start_next_task()


def await_duty_cycle():
    """Wait for the next duty cycle transition"""
    rospy.loginfo('Outside sampling window, waiting for next duty cycle')

    # Check duty cycle every minute
    check_interval = rospy.get_param('tasks/duty_cycle/check_interval', 60)

    while not arm.should_transition_duty_state() and not rospy.is_shutdown():
        rospy.sleep(check_interval)

    rospy.loginfo('Duty cycle transition time reached')
    arm.start_next_task()


def startup_sampling():
    """Start IFCB sampling by turning on the digital logger outlet"""
    rospy.loginfo('Starting IFCB sampling - turning on IFCB power')
    start_ifcb()

    # Update duty cycle state
    arm.is_sampling_active = True
    arm.update_duty_cycle_state()

    arm.start_next_task()


def shutdown_sampling():
    """Stop IFCB sampling by turning off the digital logger outlet"""
    rospy.loginfo('Stopping IFCB sampling - turning off IFCB power')

    # Wait for current sample to finish
    shutdown_duration = rospy.get_param('tasks/duty_cycle/shutdown_wait_duration', 60)
    rospy.logwarn(f'Waiting for IFCB to finish current sample for {shutdown_duration} seconds')
    rospy.sleep(shutdown_duration)

    stop_ifcb()

    # Update duty cycle state
    arm.is_sampling_active = False
    arm.update_duty_cycle_state()

    arm.start_next_task()


def start_ifcb():
    """Turn on IFCB via digital logger"""
    if dl_ifcb_pub:
        dl_ifcb_pub.publish(Bool(data=True))

        # Wait for IFCB to connect
        restart_duration = rospy.get_param('tasks/duty_cycle/restart_wait_duration', 120)
        rospy.loginfo(f'Waiting for IFCB to connect (timeout: {restart_duration} seconds)')

        # If already connected, proceed immediately
        if arm.ifcb_connected:
            rospy.loginfo('IFCB already connected, proceeding')
            return

        # Wait for connection with timeout
        if arm.ifcb_connection_event.wait(timeout=restart_duration):
            rospy.loginfo('IFCB connected successfully')
        else:
            rospy.logerr('IFCB failed to connect within timeout')


def stop_ifcb():
    """Turn off IFCB via digital logger"""
    if dl_ifcb_pub:
        dl_ifcb_pub.publish(Bool(data=False))


def its_scheduled_depth_time(peak_value):
    """Determine if it's time to run a scheduled depth task.

    Args:
        peak_value: The current profiler peak value

    Returns:
        bool: True if scheduled depth should be run
    """
    # Decide if we want to use the scheduled depth
    scheduled_interval = rospy.Duration(60*rospy.get_param('tasks/scheduled_depth/every'))

    run_schedule = False
    if math.isclose(scheduled_interval.to_sec(), 0.0):  # disabled
        run_schedule = False
    elif arm.last_scheduled_time is None:
        run_schedule = True
    elif rospy.Time.now() - arm.last_scheduled_time > scheduled_interval:
        run_schedule = True
    elif peak_value is not None and peak_value < rospy.get_param('tasks/profiler_peak/threshold'):
        run_schedule = True
    return run_schedule


def scheduled_depth(arm_instance):
    """Calculate the next scheduled depth to sample.

    Args:
        arm_instance: The ArmDutyCycle instance

    Returns:
        float: The target depth in meters
    """
    scheduled_depths = np.linspace(
        rospy.get_param('tasks/scheduled_depth/range/first'),
        rospy.get_param('tasks/scheduled_depth/range/last'),
        rospy.get_param('tasks/scheduled_depth/range/count'),
    )

    target_depth = scheduled_depths[arm_instance.next_scheduled_index % scheduled_depths.shape[0]]

    rospy.loginfo(f'Using scheduled depth of {target_depth:.2f} m')

    arm_instance.next_scheduled_index += 1
    arm_instance.last_scheduled_time = rospy.Time.now()
    return target_depth


def send_ifcb_action():
    """Send an IFCB sampling action and wait for completion."""
    goal = RunIFCBGoal()
    ifcb_runner.send_goal(goal)
    ifcb_runner.wait_for_result()


def handle_downcast(move_result):
    """Handle completion of a downcast movement and process profile data.

    Args:
        move_result: The result from the movement action
    """
    # Wait for the profile data for this cast
    while arm.latest_profile is None or \
        arm.latest_profile.goal_uuid != move_result.uuid:
        notified = arm.profile_activity.wait(60)

        # For short downcasts profiling will likely fail; switch to scheduled depth
        if not notified:
            rospy.logwarn('No profile data received')
            arm.profiler_peak_depth = None
            arm.profiler_peak_value = None
            arm.start_next_task()
            return

    # Find the maximal value in the profile
    argmax = max(range(len(arm.latest_profile.values)),
                 key=lambda i: arm.latest_profile.values[i])
    arm.profiler_peak_value = arm.latest_profile.values[argmax]
    arm.profiler_peak_depth = arm.latest_profile.depths[argmax]

    rospy.loginfo(f'Profile peak is {arm.profiler_peak_value:.2f} at '
                 f'{arm.profiler_peak_depth:.2f} m')
    arm.start_next_task()


def handle_target_depth(move_result):
    """Handle arrival at target depth and trigger IFCB sampling.

    Args:
        move_result: The result from the movement action
    """
    send_ifcb_action()
    arm.start_next_task()


def handle_nowinch():
    """Handle IFCB sampling when winch is disabled."""
    send_ifcb_action()
    arm.start_next_task()


def main():
    global arm
    global ifcb_runner
    global dl_ifcb_pub

    rospy.init_node('arm_duty_cycle', log_level=rospy.DEBUG)

    winch_name = None
    if rospy.get_param('winch_enabled') is True:
        winch_name = rospy.get_namespace() + 'winch/move_to_depth'
    arm = ArmDutyCycle(rospy.get_name(), winch_name)

    # Set a fake timestamp for having run beads and cartridge debubble, so that
    # we don't run it every startup and potentially waste time or bead supply.
    arm.last_cart_debub_time = rospy.Time.now()
    arm.last_bead_time = rospy.Time.now()

    # Subscribe to profiler messages that follow each transit
    rospy.Subscriber('profiler', DepthProfile, on_profile_msg)

    # Subscribe to IFCB connection status
    rospy.Subscriber('/ifcb/connected', Bool, on_ifcb_connection_status)

    # Setup publisher for DL outlet control
    dl_ifcb_pub = rospy.Publisher('/digital_logger/outlet/ifcb/control', Bool, queue_size=10)

    # Setup action client for running IFCB
    ifcb_runner = actionlib.SimpleActionClient('ifcb_runner/sample', RunIFCBAction)
    rospy.loginfo(f'Arm {rospy.get_name()} awaiting IFCB-run action server')
    ifcb_runner.wait_for_server()
    rospy.loginfo(f'Arm {rospy.get_name()} IFCB-run action server acquired')

    # Initialize duty cycle state
    arm.update_duty_cycle_state()

    arm.loop()

if __name__ == '__main__':
    main()
