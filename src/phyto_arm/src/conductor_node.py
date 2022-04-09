#!/usr/bin/env python3
import sys
import threading

import actionlib
import numpy as np
import rospy

from ifcbclient.protocol import parse_response as parse_ifcb_msg
from ifcb.instrumentation import parse_marker as parse_ifcb_marker

from ds_core_msgs.msg import RawData

from ifcb.srv import RunRoutine
from phyto_arm.msg import DepthProfile, MoveToDepthAction, MoveToDepthGoal


# Global references to services/actions provided by other nodes
ifcb_run_routine = None
move_to_depth = None


# N.B.: This is an abuse of Python syntax, don't do as I do!
class state:
    ifcb_activity = threading.Event()
    latest_ifcb_msg = None

    profile_activity = threading.Event()
    latest_profile = None

    ifcb_is_busy = False
    ifcb_busy_condition = threading.Condition()

    ifcb_is_instrumented = False

    position_hold = True
    position_hold_condition = threading.Condition()

    scheduled_depths = []
    last_scheduled_time = None
    next_scheduled_index = 0


def on_ifcb_msg(msg):
    # Parse the message and see if it is a marker
    marker = None
    parsed = parse_ifcb_msg(msg.data.decode())
    if len(parsed) == 2 and parsed[0] == 'reportevent':
        marker = parse_marker(parsed[1])
    if marker is None:
        return

    # Alert waiting threads that there was some activity
    state.ifcb_activity.set()
    state.ifcb_activity.clear()
    
    # If we got a marker, we know the IFCB is running an instrumented routine,
    # and we can wait for it to finish
    state.is_ifcb_instrumented = True

    # Detect when we can release our position hold 
    if marker['routine'] == 'runsample' and \
       marker['kind'] == 'before' and \
       marker['value'].get('StepType') == 'SwitchTriggering' and \
       marker['value'].get('Arguments', []) == [False]:

        with state.position_hold_condition:
            state.position_hold = False
            state.position_hold_condition.notify()

    # Detect when a routine finishes
    if marker['kind'] == 'exit' and marker['path'] == []:
        with state.ifcb_busy_condition:
            state.ifcb_is_busy = False
            state.ifcb_busy_condition.notify()


def on_profile_msg(msg):
    # Alert waiting threads to the new profile message
    state.latest_profile = msg
    state.profile_activity.set()
    state.profile_activity.clear()


def loop():
    global ifcb_run_routine, move_to_depth

    # Go to minimum depth
    rospy.loginfo('Transiting to minimum depth')
    move_to_depth.send_goal(
        MoveToDepthGoal(depth=rospy.get_param('~range/min')))
    move_to_depth.wait_for_result()
    assert move_to_depth.get_state() == actionlib.GoalStatus.SUCCEEDED

    # Perform a downcast
    rospy.loginfo('Performing downcast')
    move_to_depth.send_goal(
        MoveToDepthGoal(depth=rospy.get_param('~range/max')))
    move_to_depth.wait_for_result()
    assert move_to_depth.get_state() == actionlib.GoalStatus.SUCCEEDED

    # Wait for the profile data for this cast
    result = move_to_depth.get_result()
    while state.latest_profile is None or \
          state.latest_profile.goal_uuid != result.uuid:
        rospy.loginfo('Waiting for next profile?')
        state.profile_activity.wait()

    # Find the maximal value in the profile
    argmax = max(range(len(state.latest_profile.values)),
                 key=lambda i: state.latest_profile.values[i])
    target_value = state.latest_profile.values[argmax]
    target_depth = state.latest_profile.depths[argmax]

    rospy.loginfo(f'Profile peak is {target_value:.2f} at {target_depth:.2f} m')

    # Decide if we want to use the scheduled depth
    scheduled_interval = rospy.Duration(60*rospy.get_param('~schedule/every'))

    if scheduled_interval == 0:  # disabled
        run_schedule = False
    elif state.last_scheduled_time is None:
        run_schedule = True
    elif rospy.Time.now() - state.last_scheduled_time > scheduled_interval:
        run_schedule = True
    elif target_value < rospy.get_param('~threshold'):
        run_schedule = True
    else:
        run_schedule = False

    if run_schedule:
        scheduled_depths = np.linspace(
            rospy.get_param('~schedule/range/first'),
            rospy.get_param('~schedule/range/last'),
            rospy.get_param('~schedule/range/count'),
        )

        target_depth = scheduled_depths[state.next_scheduled_index \
                                        % scheduled_depths.shape[0]]

        rospy.loginfo(f'Using scheduled depth of {target_depth:.2f} m')

        state.next_scheduled_index += 1
        state.last_scheduled_time = rospy.Time.now()
    
    # Safety check: Do not exceed depth bounds
    if not (rospy.get_param('~range/min') <= target_depth <= \
            rospy.get_param('~range/max')):
        rospy.logerr('Refusing to exceed depth bounds')
        return

    # Go to target depth
    move_to_depth.send_goal(MoveToDepthGoal(depth=target_depth))
    move_to_depth.wait_for_result()
    assert move_to_depth.get_state() == actionlib.GoalStatus.SUCCEEDED

    # Activate position hold, which will be released at a certain point during
    # sampling.
    with state.position_hold_condition:
        state.position_hold = True

    rospy.loginfo('Stopping before IFCB stuff')
    import time
    time.sleep(30)
    rospy.loginfo('Continuing after delay')
    return

    # Wait for current IFCB activity to finish
    with state.ifcb_busy_condition:
        state.ifcb_busy_condition.wait_for(lambda: not state.ifcb_is_busy)

    # Debubble
    with state.ifcb_busy_condition:
        state.ifcb_is_busy = True
    result = ifcb_run_routine(routine='debubble', instrument=True)
    assert result.success

    # Wait for debubble to finish
    with state.ifcb_busy_condition:
        state.ifcb_busy_condition.wait_for(lambda: not state.ifcb_is_busy)

    # Sample
    with state.ifcb_busy_condition:
        state.ifcb_is_busy = True
    result = ifcb_run_routine(routine='runsample', instrument=True)
    assert result.success

    # Wait for position hold release
    with state.position_hold_condition:
        state.position_hold_condition.wait_for(lambda: not state.position_hold)



def main():
    global ifcb_run_routine, move_to_depth

    rospy.init_node('conductor', anonymous=True)

    # Subscribe to IFCB messages to track routine progress
    rospy.Subscriber('/ifcb/in', RawData, on_ifcb_msg)

    # Subscribe to profiler messages that follow each transit
    rospy.Subscriber('/profiler', DepthProfile, on_profile_msg)

    # Initialize service proxy for sending routines to the IFCB
    rospy.wait_for_service('/ifcb/routine')
    ifcb_run_routine = rospy.ServiceProxy('/ifcb/routine', RunRoutine)

    # Initialize action client for controlling the winch
    move_to_depth = actionlib.SimpleActionClient(
        '/winch/move_to_depth',
        MoveToDepthAction
    )
    move_to_depth.wait_for_server()

    # Run the main loop forever
    while not rospy.is_shutdown():
        loop()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
