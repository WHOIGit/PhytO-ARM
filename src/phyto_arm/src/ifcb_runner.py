#!/usr/bin/env python3

from dataclasses import dataclass
import functools
import json
import math
from threading import Condition, Event

import actionlib
import rospy

from ds_core_msgs.msg import RawData
from ifcbclient.protocol import parse_response as parse_ifcb_msg
from ifcb.instrumentation import parse_marker as parse_ifcb_marker
from ifcb.srv import RunRoutine

from phyto_arm.msg import ConductorState, ConductorStates, RunIFCBAction, RunIFCBResult

# Global references to service provided by other node
ifcb_run_routine = None
ifcb_action_server = None


# The sample has been drawn when the IFCB switches to its sampling speed
SAMPLING_SPEED = {
    'StepType': 'ChangeSpeed',
    'Arguments': ['{#SamplingSpeed}'],
}


# Convenience function to publish state updates for debugging
set_state = None
def set_pub_state(pub, s):
    m = ConductorState()
    m.header.stamp = rospy.Time.now()
    m.state = s
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
    # The position hold waits on a separate condition. Set the idle event
    # and notify under that condition's lock so a stop wakes the hold waiter
    # even if the IFCB skipped the sampling marker.
    if parsed == ['valuechanged', 'interactive', 'stopped']:
        rospy.loginfo('IFCB routine not running')
        with state.position_hold_condition:
            state.ifcb_is_idle.set()
            state.position_hold_condition.notify()
        return

    # Remaining behaviors depend on having a parsed marker
    if marker is None:
        return

    # Detect when we can release our position hold
    if (marker['routine'] == 'runsample' and marker['kind'] == 'before'
            and all(marker['value'].get(k) == v
                    for k, v in SAMPLING_SPEED.items())):
        with state.position_hold_condition:
            if not state.ifcb_is_idle.is_set():
                state.position_hold = False
                state.position_hold_condition.notify()



def run_routine(routine):
    # Arm the hold before the Host can report the sampling step
    before = []
    if routine == 'runsample':
        before = [json.dumps(SAMPLING_SPEED)]
        with state.position_hold_condition:
            state.position_hold = True

    # Clear the idle event before sending the routine because the Host can
    # report completion before the service call returns.
    state.ifcb_is_idle.clear()
    result = ifcb_run_routine(routine=routine, instrument_before=before)
    if not result.success:
        state.ifcb_is_idle.set()
        raise ValueError('routine request failed')

    # Wait for sample intake to finish before allowing the winch to move
    if before:
        with state.position_hold_condition:
            state.position_hold_condition.wait_for(
                lambda: not state.position_hold or state.ifcb_is_idle.is_set())
            if state.position_hold:
                raise ValueError('runsample stopped before the sampling step')


def run_sample_routines(goal):
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
        state.ifcb_is_idle.wait()
        rospy.loginfo(f'Starting {routine} routine')
        set_state(state_const)
        try:
            run_routine(routine)
        except (rospy.ServiceException, ValueError) as exc:
            message = f'Could not run {routine}: {exc}'
            rospy.logerr(message)
            ifcb_action_server.set_aborted(text=message)
            return

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
