#!/usr/bin/env python3
import math

from datetime import datetime, timedelta
from threading import Event

import actionlib
import numpy as np
import rospy

from arm_base import ArmBase, Task
from phyto_arm.msg import DepthProfile, RunIFCBGoal, RunIFCBAction


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

    # Primary method for determining the next task to execute
    # Each Task object takes:
    #  - name: a string identifying the task
    #  - callback: a function to call when the task is complete
    #  - depth: the depth to move to (optional, won't move if not provided)
    #  - speed: the speed to move at (optional, will use config max if not provided)
    def get_next_task(self, last_task):
        if not rospy.get_param('winch/enabled'):
            return Task('no_winch', handle_nowinch)

        # If close to scheduled wiz time, prioritize first
        if its_wiz_time():
            wiz_depth = compute_wiz_depth(self.profiler_peak_depth)
            return Task('wiz_probe', lambda _: await_wiz_probe(self.start_next_task), wiz_depth)

        # Othrwise, start off at min
        if last_task is None or last_task.name in ['scheduled_depth', 'profiler_peak_depth', 'wiz_probe']:
            return Task('upcast', self.start_next_task, rospy.get_param('winch/range/min'))

        # Then perform a downcast to get a full profile
        if last_task.name == 'upcast':
            return Task('downcast', handle_downcast, rospy.get_param('winch/range/max'))

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


# Convert HH:MM to a rospy.Time datetime. Take into account the current time,
# pushing the requested time to the next day if it's already passed.
def wiz_to_rostime(hhmm_time):
    today_time = datetime.now().time()

    # Convert HH:MM to a timestamp
    wiz_time = datetime.strptime(hhmm_time, '%H:%M').time()
    wiz_date = datetime.now().date()

    # wiz_time should always come after the current time, even if it's the next day
    if wiz_time < today_time:
        wiz_date = datetime.now().date() + timedelta(days=1)

    # Create a rospy.Time object
    unix_timestamp = datetime.combine(wiz_date, wiz_time).timestamp()

    # Create a rospy.Time object
    return rospy.Time.from_sec(unix_timestamp)


def find_next_wiz_time():
    # Get array of HH:MM times
    wiz_probe_times = rospy.get_param('tasks/wiz_probe/times')

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
    prep_window = rospy.Duration(60*rospy.get_param('tasks/wiz_probe/preparation_window'))

    # If we're within the preparation window, it's wizin' time
    return (find_next_wiz_time() - rospy.Time.now()) <= prep_window


def compute_wiz_depth(peak_depth):
    wiz_depth = rospy.get_param('tasks/wiz_probe/default_depth')
    if rospy.get_param('tasks/wiz_probe/use_profiler_peak'):
        if peak_depth is not None:
            wiz_depth = peak_depth + rospy.get_param('tasks/wiz_probe/peak_offset')
        else:
            rospy.logwarn('No profiler peak depth available, using default wiz probe depth')
            wiz_depth = rospy.get_param('tasks/wiz_probe/default_depth')

    # Check that adjustments haven't caused us to exceed bounds. Go with bound if so
    if wiz_depth > rospy.get_param('winch/range/max'):
        rospy.logwarn('Preferred wiz probe depth exceeds max depth, using max depth')
        wiz_depth = rospy.get_param('winch/range/max')
    if wiz_depth < rospy.get_param('winch/range/min'):
        rospy.logwarn('Preferred wiz probe depth is less than min depth, using min depth')
        wiz_depth = rospy.get_param('winch/range/min')

    return wiz_depth


def await_wiz_probe(callback):
    # Find the next time to run the wiz probe
    next_time = find_next_wiz_time()

    # Ensure we're still in the preparation window
    prep_window = rospy.Duration(60*rospy.get_param('tasks/wiz_probe/preparation_window'))
    preptime_remaining = next_time - rospy.Time.now()
    if preptime_remaining > prep_window:
        raise ValueError(f'Next wiz time is {preptime_remaining.to_sec()}s away, should be less than \
                          {prep_window.to_sec()}s. Preparation window likely too short.')

    # Wait for the rest of the window + the wiz probe duration
    rospy.loginfo('Waiting for wiz probe to finish')
    wiz_duration = 60*rospy.get_param('tasks/wiz_probe/duration')
    rospy.sleep(wiz_duration + preptime_remaining.to_sec())

    # TODO: Run IFCB while waiting

    rospy.loginfo('Wiz probe wait time complete')
    callback()


def its_scheduled_depth_time(peak_value):
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


def scheduled_depth():
    scheduled_depths = np.linspace(
        rospy.get_param('tasks/scheduled_depth/range/first'),
        rospy.get_param('tasks/scheduled_depth/range/last'),
        rospy.get_param('tasks/scheduled_depth/range/count'),
    )

    target_depth = scheduled_depths[arm.next_scheduled_index % scheduled_depths.shape[0]]

    rospy.loginfo(f'Using scheduled depth of {target_depth:.2f} m')

    arm.next_scheduled_index += 1
    arm.last_scheduled_time = rospy.Time.now()
    return target_depth


def send_ifcb_action():
    goal = RunIFCBGoal()
    ifcb_runner.send_goal(goal)
    ifcb_runner.wait_for_result()


def handle_downcast(move_result):
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
    argmax = max(range(len(arm.latest_profile.values)), key=lambda i: arm.latest_profile.values[i])
    arm.profiler_peak_value = arm.latest_profile.values[argmax]
    arm.profiler_peak_depth = arm.latest_profile.depths[argmax]

    rospy.loginfo(f'Profile peak is {arm.profiler_peak_value:.2f} at {arm.profiler_peak_depth:.2f} m')
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

    rospy.init_node('arm_ifcb', log_level=rospy.DEBUG)

    winch_name = None
    if rospy.get_param('winch/enabled') is True:
        winch_name = rospy.get_namespace() + 'winch/move_to_depth'
    arm = ArmIFCB(rospy.get_name(), winch_name)

    # Subscribe to profiler messages that follow each transit
    rospy.Subscriber('profiler', DepthProfile, on_profile_msg)

    # Setup action client for running IFCB
    ifcb_runner = actionlib.SimpleActionClient('ifcb_runner/sample', RunIFCBAction)
    rospy.loginfo(f'Arm {rospy.get_name()} awaiting IFCB-run action server')
    ifcb_runner.wait_for_server()
    rospy.loginfo(f'Arm {rospy.get_name()} IFCB-run action server acquired')

    # Set a fake timestamp for having run beads and catridge debubble, so that
    # we don't run it every startup and potentially waste time or bead supply.
    arm.last_cart_debub_time = rospy.Time.now()
    arm.last_bead_time = rospy.Time.now()

    arm.loop()

if __name__ == '__main__':
    main()
