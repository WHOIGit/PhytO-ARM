#!/usr/bin/env python3

from dataclasses import dataclass
import functools
import math
from threading import Condition, Event, Lock

import actionlib
import rospy

from ds_core_msgs.msg import RawData
from ifcbclient.protocol import parse_response as parse_ifcb_msg
from ifcb.instrumentation import parse_marker as parse_ifcb_marker
from ifcb.srv import RunRoutine
from std_msgs.msg import Bool

from phyto_arm.msg import ConductorState, ConductorStates, RunIFCBAction, RunIFCBResult

# Global references to service provided by other node
ifcb_run_routine = None
ifcb_action_server = None


# Convenience function to publish state updates for debugging
current_state = None
publish_lock = Lock()
set_state = None
state_timer = None


def set_pub_state(pub, new_state=None):
    global current_state, state_timer

    # Restart the heartbeat on each transition
    if new_state is not None:
        current_state = new_state
        if state_timer is not None:
            state_timer.shutdown()
        state_timer = rospy.Timer(rospy.Duration(1),
                                  lambda _: set_pub_state(pub), reset=True)

    # Keep an older heartbeat from being latched after a transition
    with publish_lock:
        m = ConductorState()
        m.header.stamp = rospy.Time.now()
        m.state = current_state
        pub.publish(m)


@dataclass
class IFCBState:
    ifcb_is_idle = Event()

    position_hold = False
    position_hold_condition = Condition()

    last_cart_debub_time = None
    last_bead_time = None
    last_clean_time = None

state = IFCBState()


def on_routine_running(msg):
    if msg.data:
        state.ifcb_is_idle.clear()
    else:
        state.ifcb_is_idle.set()


def on_ifcb_msg(msg):
    # Parse the message and see if it is a marker
    marker = None
    parsed = parse_ifcb_msg(msg.data.decode())
    if len(parsed) == 2 and parsed[0] == 'reportevent':
        marker = parse_ifcb_marker(parsed[1])

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



def run_sample_routines(goal):
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
        state.last_bead_time = rospy.Time.now()

    # Determine if it's time to run a clean
    clean_interval = rospy.Duration(60*rospy.get_param('ifcb_maintenance/clean_interval'))
    run_clean = not math.isclose(clean_interval.to_sec(), 0.0)  # disabled
    if run_clean and rospy.Time.now() - state.last_clean_time > clean_interval:
        rospy.loginfo('Will run biocide and bleach this round')
        playlist.append((ConductorStates.IFCB_BIOCIDE,  'biocide'))
        playlist.append((ConductorStates.IFCB_BLEACH,   'bleach'))
        state.last_clean_time = rospy.Time.now()

    # Always run a debubble and sample
    playlist.append((ConductorStates.IFCB_DEBUBBLE,  'debubble'))
    playlist.append((ConductorStates.IFCB_RUNSAMPLE, 'runsample'))

    # Determine if we should flush the sample tube after each sample
    flush_after_sample = rospy.get_param('ifcb_maintenance/flush_after_sample')
    if flush_after_sample:
        playlist.append((ConductorStates.IFCB_FLUSH, 'flushsampletube'))

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
        ifcb_action_server.set_succeeded(RunIFCBResult())
    rospy.loginfo('Position hold released')



def main():
    global set_state
    global ifcb_run_routine
    global ifcb_action_server
    rospy.init_node('ifcb_simple', anonymous=True, log_level=rospy.DEBUG)

    # Publish state messages useful for debugging
    set_state = functools.partial(set_pub_state,
        rospy.Publisher('~state', ConductorState, queue_size=1, latch=True))

    # Subscribe to IFCB messages to track routine progress
    rospy.Subscriber('/ifcb/in', RawData, on_ifcb_msg)
    rospy.Subscriber('/ifcb/is_routine_running', Bool, on_routine_running)

    # Initialize service proxy for sending routines to the IFCB
    ifcb_run_routine = rospy.ServiceProxy('/ifcb/routine', RunRoutine)
    ifcb_run_routine.wait_for_service()

    # Setup action server for running IFCB
    ifcb_action_server = actionlib.SimpleActionServer('~sample', RunIFCBAction, run_sample_routines, False)
    ifcb_action_server.start()


    # Set a fake timestamp for having run beads and catridge debubble, so that
    # we don't run it every startup and potentially waste time or bead supply.
    state.last_cart_debub_time = rospy.Time.now()
    state.last_bead_time = rospy.Time.now()
    state.last_clean_time = rospy.Time.now()

    rospy.spin()


if __name__ == '__main__':
    main()
