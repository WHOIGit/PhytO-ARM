#!/usr/bin/env python3
import functools
import math
import threading
import sys

import actionlib
import numpy as np
import rospy

from ifcbclient.protocol import parse_response as parse_ifcb_msg
from ifcb.instrumentation import parse_marker as parse_ifcb_marker

from ds_core_msgs.msg import RawData

from ifcb.srv import RunRoutine

from phyto_arm.srv import ArmRegistration, ArmTask, ArmTaskResponse

from phyto_arm.msg import ConductorState, ConductorStates, DepthProfile, \
                          InstrumentAction, InstrumentResult


# Global references to service provided by other node
ifcb_run_routine = None
instrument_server = None

lock = threading.RLock()

# Convenience function to publish state updates for debugging
set_state = None
def set_pub_state(pub, s):
    m = ConductorState()
    m.header.stamp = rospy.Time.now()
    m.state = s
    pub.publish(m)


# N.B.: This is an abuse of Python syntax, don't do as I do!
class state:
    profile_activity = threading.Event()
    latest_profile = None

    ifcb_is_idle = threading.Event()

    position_hold = False
    position_hold_condition = threading.Condition()

    scheduled_depths = []
    last_scheduled_time = None
    next_scheduled_index = 0

    last_cart_debub_time = None
    last_bead_time = None

    tasks = []


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
    scheduled_interval = rospy.Duration(60*rospy.get_param('tasks/schedule/every'))

    if math.isclose(scheduled_interval.to_sec(), 0.0):  # disabled
        run_schedule = False
    elif state.last_scheduled_time is None:
        run_schedule = True
    elif rospy.Time.now() - state.last_scheduled_time > scheduled_interval:
        run_schedule = True
    elif peak_value < rospy.get_param('tasks/threshold'):
        run_schedule = True
    else:
        run_schedule = False
    return run_schedule


def scheduled_depth():
    scheduled_depths = np.linspace(
        rospy.get_param('tasks/schedule/range/first'),
        rospy.get_param('tasks/schedule/range/last'),
        rospy.get_param('tasks/schedule/range/count'),
    )

    target_depth = scheduled_depths[state.next_scheduled_index \
                                    % scheduled_depths.shape[0]]

    rospy.loginfo(f'Using scheduled depth of {target_depth:.2f} m')

    state.next_scheduled_index += 1
    state.last_scheduled_time = rospy.Time.now()


# Initial task list
def build_task_queue():
    tasks = []
    if rospy.get_param('winch/enabled') == True:
        tasks.append(ArmTaskResponse(depth=rospy.get_param('winch/range/min'), instrument_arg="upcast"))
        tasks.append(ArmTaskResponse(depth=rospy.get_param('winch/range/max'), instrument_arg="downcast"))
    else:
        tasks.append(ArmTaskResponse(depth=0, instrument_arg="no_winch"))
    return tasks


# Responds to conductor with the next task
def get_task_handler(request):
    with lock:
        return state.tasks[0]


def run_ifcb():
    # Activate position hold, which will be released at a certain point during
    # sampling.
    rospy.loginfo('Activating position hold')
    with state.position_hold_condition:
        state.position_hold = True

    # Wait for current IFCB activity to finish
    set_state(ConductorStates.WAIT_FOR_IFCB)
    state.ifcb_is_idle.wait()

    # Build up a playlist of IFCB routines that we need to run
    playlist = []

    # Determine if it's time to run cartridge debubble
    cart_debub_interval = rospy.Duration(60*rospy.get_param('tasks/cartridge_debubble_interval'))
    run_cart_debub = not math.isclose(cart_debub_interval.to_sec(), 0.0)  # disabled
    if run_cart_debub and rospy.Time.now() - state.last_cart_debub_time > cart_debub_interval:
        rospy.loginfo('Will run cartridege debubble this round')
        playlist.append((ConductorStates.IFCB_CARTRIDGE_DEBUBBLE, 'cartridgedebubble'))
        state.last_cart_debub_time = rospy.Time.now()

    # Determine if it's time to run beads
    bead_interval = rospy.Duration(60*rospy.get_param('tasks/bead_interval'))
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


# Handles action calls from the conductor when a given task depth is reached
def instrument_handler(goal):
    arg = goal.arg
    move_result = goal.move_result

    if arg == "upcast":
        rospy.logerr('Upcast complete, popping task')
        with lock:
            state.tasks.pop(0)
        instrument_server.set_succeeded(InstrumentResult())

    elif arg == "downcast":
        # Wait for the profile data for this cast
        while state.latest_profile is None or \
            state.latest_profile.goal_uuid != move_result.uuid:
            state.profile_activity.wait()

        # Find the maximal value in the profile
        argmax = max(range(len(state.latest_profile.values)),
                    key=lambda i: state.latest_profile.values[i])
        target_value = state.latest_profile.values[argmax]
        target_depth = state.latest_profile.depths[argmax]
        
        rospy.loginfo(f'Profile peak is {target_value:.2f} at {target_depth:.2f} m')

        with lock:
            rospy.logerr('Downcast complete, popping task')
            state.tasks.pop(0)
            # Next task dependent on whether we should run a scheduled interval now
            if is_scheduled_interval(target_value):
                state.tasks.append(ArmTaskResponse(depth=scheduled_depth(), arg="scheduled_interval"))
            else:
                state.tasks.append(ArmTaskResponse(depth=target_depth, arg="peak_phy_depth"))
        
        instrument_server.set_succeeded(InstrumentResult())

    elif arg == "peak_phy_depth" or arg == "scheduled_depth":
        rospy.logerr('Phy/scheduled depth complete, popping task')
        run_ifcb()
        with lock:
            state.tasks = build_task_queue()
        instrument_server.set_succeeded(InstrumentResult())

    elif arg == "no_winch":
        try:
            rospy.logerr('Running ifcb')
            run_ifcb()
            instrument_server.set_succeeded(InstrumentResult())
        except:
            instrument_server.set_aborted(text='An exception occurred')
            raise
    else:
        raise ValueError(f'Unrecognized argument {arg} in instrument_handler')


def main():
    global set_state
    global instrument_server
    global ifcb_run_routine
    rospy.init_node('arm', anonymous=True, log_level=rospy.DEBUG)

    # Build initial task queue
    with lock:
        state.tasks = build_task_queue()

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

    # Setup service for fetching tasks
    service_name = rospy.get_namespace() + 'arm/get_task'
    rospy.Service(service_name, ArmTask, get_task_handler)

    # Setup action server for running IFCB
    instrument_name = rospy.get_namespace() + 'arm/run_instrument'
    instrument_server = actionlib.SimpleActionServer(instrument_name, InstrumentAction, instrument_handler, False)
    instrument_server.start()

    # Get winch path, setup service client, register
    winch_name = ''
    if rospy.get_param('winch/enabled') == True:
        winch_name = rospy.get_namespace() + 'winch/move_to_depth'
    register = rospy.ServiceProxy('/conductor/register_arm', ArmRegistration)
    register.wait_for_service()
    register(rospy.get_namespace(), winch_name, instrument_name, service_name)

    # Set a fake timestamp for having run beads and catridge debubble, so that
    # we don't run it every startup and potentially waste time or bead supply.
    state.last_cart_debub_time = rospy.Time.now()
    state.last_bead_time = rospy.Time.now()

    rospy.spin()


if __name__ == '__main__':
    main()
