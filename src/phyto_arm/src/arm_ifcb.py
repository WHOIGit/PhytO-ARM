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


from phyto_arm.msg import ConductorState, ConductorStates, DepthProfile

from arm_base import ArmBase, Task


# Global references to service provided by other node
ifcb_run_routine = None


# Convenience function to publish state updates for debugging
set_state = None
def set_pub_state(pub, s):
    m = ConductorState()
    m.header.stamp = rospy.Time.now()
    m.state = s
    pub.publish(m)


class ArmIFCB(ArmBase):
    profile_activity = Event()
    latest_profile = None

    ifcb_is_idle = Event()

    position_hold = False
    position_hold_condition = Condition()

    scheduled_depths = []
    last_scheduled_time = None
    next_scheduled_index = 0

    last_cart_debub_time = None
    last_bead_time = None


state = None


def on_ifcb_msg(msg):
    # Parse the message and see if it is a marker
    marker = None
    parsed = parse_ifcb_msg(msg.data.decode())
    if len(parsed) == 2 and parsed[0] == 'reportevent':
        marker = parse_ifcb_marker(parsed[1])

    # For routines sent with 'interactive:start', the IFCB will tell us when the
    # routine has started and finished. This is the most reliable way to detect
    # this, rather than using the markers.
    #
    # XXX
    # The ifcb_is_idle Event should be clear()'d _before_ sending a routine, to
    # avoid a race condition where a thread sends a routine and then immediately
    # awaits the Event before we have been told the routine started.
    if parsed == ['valuechanged', 'interactive', 'stopped']:
        rospy.loginfo('IFCB routine not running')
        state.ifcb_is_idle.set()
        return

    # Remaining behaviors depend on having a parsed marker
    if marker is None:
        return

    # Detect when we can release our position hold
    if marker['routine'] == 'runsample' and \
       marker['kind'] == 'before' and \
       marker['value'].get('StepType') == 'ChangeSpeed' and \
       marker['value'].get('Arguments', []) == ['{#SamplingSpeed}']:

        rospy.loginfo('Should release position hold now')

        with state.position_hold_condition:
            state.position_hold = False
            state.position_hold_condition.notify()


def on_profile_msg(msg):
    # Alert waiting threads to the new profile message
    state.latest_profile = msg
    state.profile_activity.set()
    state.profile_activity.clear()


def is_scheduled_interval(peak_value):
    # Decide if we want to use the scheduled depth
    scheduled_interval = rospy.Duration(60*rospy.get_param('tasks/scheduled_depth/every'))

    if math.isclose(scheduled_interval.to_sec(), 0.0):  # disabled
        run_schedule = False
    elif state.last_scheduled_time is None:
        run_schedule = True
    elif rospy.Time.now() - state.last_scheduled_time > scheduled_interval:
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

    target_depth = scheduled_depths[state.next_scheduled_index % scheduled_depths.shape[0]]

    rospy.loginfo(f'Using scheduled depth of {target_depth:.2f} m')

    state.next_scheduled_index += 1
    state.last_scheduled_time = rospy.Time.now()
    return target_depth


def run_ifcb():
    # Activate position hold, which will be released at a certain point during sampling.
    rospy.loginfo('Activating position hold')
    with state.position_hold_condition:
        state.position_hold = True

    # Wait for current IFCB activity to finish
    set_state(ConductorStates.WAIT_FOR_IFCB)
    state.ifcb_is_idle.wait()

    # Build up a playlist of IFCB routines that we need to run
    playlist = []

    # Determine if it's time to run cartridge debubble
    cart_debub_interval = rospy.Duration(60*rospy.get_param('ifcb_maintenance/cartridge_debubble_interval'))
    run_cart_debub = not math.isclose(cart_debub_interval.to_sec(), 0.0)  # disabled
    if run_cart_debub and rospy.Time.now() - state.last_cart_debub_time > cart_debub_interval:
        rospy.loginfo('Will run cartridege debubble this round')
        playlist.append((ConductorStates.IFCB_CARTRIDGE_DEBUBBLE, 'cartridgedebubble'))
        state.last_cart_debub_time = rospy.Time.now()

    # Determine if it's time to run beads
    bead_interval = rospy.Duration(60*rospy.get_param('ifcb_maintenance/bead_interval'))
    run_beads = not math.isclose(bead_interval.to_sec(), 0.0)  # disabled
    if run_beads and rospy.Time.now() - state.last_bead_time > bead_interval:
        rospy.loginfo('Will run beads this round')
        playlist.append((ConductorStates.IFCB_DEBUBBLE, 'debubble'))
        playlist.append((ConductorStates.IFCB_BEADS,    'beads'))
        playlist.append((ConductorStates.IFCB_BIOCIDE,  'biocide'))
        playlist.append((ConductorStates.IFCB_BLEACH,   'bleach'))
        state.last_bead_time = rospy.Time.now()

    # Always run a debubble and sample
    playlist.append((ConductorStates.IFCB_DEBUBBLE,  'debubble'))
    playlist.append((ConductorStates.IFCB_RUNSAMPLE, 'runsample'))

    # Run IFCB steps in sequence
    for state_const, routine in playlist:
        # Wait for previous routine to finish before we submit a new one. The
        # loop exits after we start the 'runsample' routine, then we'll wait
        # for the position hold release.
        state.ifcb_is_idle.wait()

        rospy.loginfo(f'Starting {routine} routine')
        set_state(state_const)
        state.ifcb_is_idle.clear()
        result = ifcb_run_routine(routine=routine, instrument=True)
        assert result.success

    # Wait for position hold release
    rospy.loginfo('Routines finished - awaiting position hold release')
    with state.position_hold_condition:
        state.position_hold_condition.wait_for(lambda: not state.position_hold)
    rospy.loginfo('Position hold released')


def handle_downcast(move_result):
    # Wait for the profile data for this cast
    while state.latest_profile is None or \
        state.latest_profile.goal_uuid != move_result.uuid:
        notified = state.profile_activity.wait(60)
        # For short downcasts profiling will likely fail; switch to scheduled depth
        if not notified:
            rospy.logwarn('No profile data received, switching to scheduled depth')
            state.tasks.put(Task(name="scheduled_depth", depth=scheduled_depth(), callback=handle_target_depth))
            state.task_complete()
            return

    # Find the maximal value in the profile
    argmax = max(range(len(state.latest_profile.values)), key=lambda i: state.latest_profile.values[i])
    target_value = state.latest_profile.values[argmax]
    target_depth = state.latest_profile.depths[argmax]

    rospy.loginfo(f'Profile peak is {target_value:.2f} at {target_depth:.2f} m')

    # Next task dependent on whether we should run a scheduled interval now
    if is_scheduled_interval(target_value):
        state.tasks.put(Task(name="scheduled_depth", depth=scheduled_depth(), callback=handle_target_depth))
    else:
        state.tasks.put(Task(name="peak_phy_depth", depth=target_depth, callback=handle_target_depth))
    state.task_complete()


def handle_target_depth(move_result):
    run_ifcb()
    # Should be end of queue; start over
    build_task_queue()
    state.task_complete()


def handle_nowinch():
    try:
        rospy.logerr('Running ifcb')
        run_ifcb()
        state.task_complete()
    except:
        raise


# Initial task list
def build_task_queue():
    # Clear queue. Do this instead of creating a new Queue to maintain thread safety
    with state.tasks.mutex:
        state.tasks.queue.clear()
    if rospy.get_param('winch/enabled') == True:
        state.tasks.put(Task("upcast", rospy.get_param('winch/range/min'), state.task_complete))
        state.tasks.put(Task("downcast", rospy.get_param('winch/range/max'), handle_downcast))
    else:
        state.tasks.put(Task("no_winch", None, handle_nowinch))


def main():
    global state
    global set_state
    global ifcb_run_routine
    rospy.init_node('arm', anonymous=True, log_level=rospy.DEBUG)

    # Build initial task queue
    build_task_queue()

    # Publish state messages useful for debugging
    set_state = functools.partial(set_pub_state,
        rospy.Publisher('~state', ConductorState, queue_size=1, latch=True))

    # Subscribe to IFCB messages to track routine progress
    rospy.Subscriber('/ifcb/in', RawData, on_ifcb_msg)

    # Subscribe to profiler messages that follow each transit
    rospy.Subscriber('/profiler', DepthProfile, on_profile_msg)

    # Initialize service proxy for sending routines to the IFCB
    ifcb_run_routine = rospy.ServiceProxy('/ifcb/routine', RunRoutine)
    ifcb_run_routine.wait_for_service()

    # Get winch path, setup service client, register
    winch_name = None
    if rospy.get_param('winch/enabled') == True:
        winch_name = rospy.get_namespace() + 'winch/move_to_depth'
    state = ArmIFCB('arm', winch_name)

    # Set a fake timestamp for having run beads and catridge debubble, so that
    # we don't run it every startup and potentially waste time or bead supply.
    state.last_cart_debub_time = rospy.Time.now()
    state.last_bead_time = rospy.Time.now()

    state.loop()

if __name__ == '__main__':
    main()
