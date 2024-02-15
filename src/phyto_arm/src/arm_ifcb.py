#!/usr/bin/env python3
from dataclasses import dataclass
import functools
import math
from queue import Queue
from threading import Event, Condition

import actionlib
import numpy as np
import rospy

from ifcbclient.protocol import parse_response as parse_ifcb_msg
from ifcb.instrumentation import parse_marker as parse_ifcb_marker

from ds_core_msgs.msg import RawData

from ifcb.srv import RunRoutine


from phyto_arm.msg import DepthProfile, RunIFCBGoal, RunIFCBAction

from arm_base import ArmBase, Task


class ArmIFCB(ArmBase):
    profile_activity = Event()
    latest_profile = None

    scheduled_depths = []
    last_scheduled_time = None
    next_scheduled_index = 0


# Global reference to action provided by other node
ifcb_runner = None
# Global reference to arm state
arm = None


def on_profile_msg(msg):
    # Alert waiting threads to the new profile message
    arm.latest_profile = msg
    arm.profile_activity.set()
    arm.profile_activity.clear()


def is_scheduled_interval(peak_value):
    # Decide if we want to use the scheduled depth
    scheduled_interval = rospy.Duration(60*rospy.get_param('tasks/scheduled_depth/every'))

    if math.isclose(scheduled_interval.to_sec(), 0.0):  # disabled
        run_schedule = False
    elif arm.last_scheduled_time is None:
        run_schedule = True
    elif rospy.Time.now() - arm.last_scheduled_time > scheduled_interval:
        run_schedule = True
    elif peak_value < rospy.get_param('tasks/phy_peak/threshold'):
        run_schedule = True
    else:
        run_schedule = False
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
            rospy.logwarn('No profile data received, switching to scheduled depth')
            arm.tasks.put(Task(name="scheduled_depth", depth=scheduled_depth(), callback=handle_target_depth))
            arm.start_next_task()
            return

    # Find the maximal value in the profile
    argmax = max(range(len(arm.latest_profile.values)), key=lambda i: arm.latest_profile.values[i])
    target_value = arm.latest_profile.values[argmax]
    target_depth = arm.latest_profile.depths[argmax]

    rospy.loginfo(f'Profile peak is {target_value:.2f} at {target_depth:.2f} m')

    # Next task dependent on whether we should run a scheduled interval now
    if is_scheduled_interval(target_value):
        arm.tasks.put(Task(name="scheduled_depth", depth=scheduled_depth(), callback=handle_target_depth))
    else:
        arm.tasks.put(Task(name="peak_phy_depth", depth=target_depth, callback=handle_target_depth))
    arm.start_next_task()


def handle_target_depth(move_result):
    send_ifcb_action()
    # Should be end of queue; start over
    build_task_queue()
    arm.start_next_task()


def handle_nowinch():
    try:
        rospy.logerr('Running ifcb')
        send_ifcb_action()
        arm.start_next_task()
    except:
        raise


# Initial task list
def build_task_queue():
    # Clear queue. Do this instead of creating a new Queue to maintain thread safety
    with arm.tasks.mutex:
        arm.tasks.queue.clear()
    if rospy.get_param('winch/enabled') == True:
        arm.tasks.put(Task("upcast", rospy.get_param('winch/range/min'), arm.start_next_task))
        arm.tasks.put(Task("downcast", rospy.get_param('winch/range/max'), handle_downcast))
    else:
        arm.tasks.put(Task("no_winch", None, handle_nowinch))


def main():
    global arm
    global set_state
    global ifcb_runner
    rospy.init_node('arm', anonymous=True, log_level=rospy.DEBUG)

    # Get winch path, setup service client, register
    winch_name = None
    if rospy.get_param('winch/enabled') == True:
        winch_name = rospy.get_namespace() + 'winch/move_to_depth'
    arm = ArmIFCB(winch_name)

    # Subscribe to profiler messages that follow each transit
    rospy.Subscriber('/profiler', DepthProfile, on_profile_msg)

    # Setup action client for running IFCB
    ifcb_runner = actionlib.SimpleActionClient('ifcb_runner/sample', RunIFCBAction)
    rospy.loginfo(f'Arm {rospy.get_name()} awaiting IFCB-run action server')
    ifcb_runner.wait_for_server()
    rospy.loginfo(f'Arm {rospy.get_name()} IFCB-run action server acquired')

    # Set a fake timestamp for having run beads and catridge debubble, so that
    # we don't run it every startup and potentially waste time or bead supply.
    arm.last_cart_debub_time = rospy.Time.now()
    arm.last_bead_time = rospy.Time.now()

    # Build initial task queue
    build_task_queue()

    arm.loop()

if __name__ == '__main__':
    main()
