#!/usr/bin/env python3
import functools
import math
import os
import random

import rclpy

from dataclasses import dataclass
from threading import Condition, Event

from action_msgs.msg import GoalStatus
from ifcb.instrumentation import parse_marker as parse_ifcb_marker
from ifcb_msgs.srv import RunRoutine
from ifcbclient.protocol import parse_response as parse_ifcb_msg
from phyto_arm.queued_action import QueuedActionServer
from phyto_arm_msgs.action import RunIFCB
from phyto_arm_msgs.msg import ConductorState, ConductorStates
from phyto_sim_utils.parameters import declare_number
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.logging import LoggingSeverity
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool
from wr2_msgs.msg import RawData


# Convenience function to publish state updates for debugging
set_state = None
def set_pub_state(node, pub, s):
    m = ConductorState()
    m.header.stamp = node.get_clock().now().to_msg()
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


def on_routine_running(msg):
    if msg.data:
        state.ifcb_is_idle.clear()
    else:
        state.ifcb_is_idle.set()


def on_ifcb_msg(node, msg):
    # Parse the message and see if it is a marker
    marker = None
    parsed = parse_ifcb_msg(bytes(msg.data).decode())
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

        node.get_logger().info('Should release position hold now')

        with state.position_hold_condition:
            state.position_hold = False
            state.position_hold_condition.notify()



def run_sample_routines(node, ifcb_run_routine, goal):
    # Activate position hold, which will be released at a certain point during sampling.
    node.get_logger().info('Activating position hold')
    with state.position_hold_condition:
        state.position_hold = True

    # Wait for current IFCB activity to finish
    set_state(ConductorStates.WAIT_FOR_IFCB)
    state.ifcb_is_idle.wait()

    # Build up a playlist of IFCB routines that we need to run
    playlist = []

    # Determine if it's time to run cartridge debubble
    cart_debub_interval = Duration(seconds=60*node.get_parameter('ifcb_maintenance.cartridge_debubble_interval').value)
    run_cart_debub = not math.isclose(cart_debub_interval.nanoseconds / 1e9, 0.0)  # disabled
    if run_cart_debub and node.get_clock().now() - state.last_cart_debub_time > cart_debub_interval:
        node.get_logger().info('Will run cartridege debubble this round')
        playlist.append((ConductorStates.IFCB_CARTRIDGE_DEBUBBLE, 'cartridgedebubble'))
        state.last_cart_debub_time = node.get_clock().now()

    # Determine if it's time to run beads
    bead_interval = Duration(seconds=60*node.get_parameter('ifcb_maintenance.bead_interval').value)
    run_beads = not math.isclose(bead_interval.nanoseconds / 1e9, 0.0)  # disabled
    if run_beads and node.get_clock().now() - state.last_bead_time > bead_interval:
        node.get_logger().info('Will run beads this round')
        playlist.append((ConductorStates.IFCB_DEBUBBLE, 'debubble'))
        playlist.append((ConductorStates.IFCB_BEADS,    'beads'))
        state.last_bead_time = node.get_clock().now()

    # Determine if it's time to run a clean
    clean_interval = Duration(seconds=60*node.get_parameter('ifcb_maintenance.clean_interval').value)
    run_clean = not math.isclose(clean_interval.nanoseconds / 1e9, 0.0)  # disabled
    if run_clean and node.get_clock().now() - state.last_clean_time > clean_interval:
        node.get_logger().info('Will run biocide and bleach this round')
        playlist.append((ConductorStates.IFCB_BIOCIDE,  'biocide'))
        playlist.append((ConductorStates.IFCB_BLEACH,   'bleach'))
        state.last_clean_time = node.get_clock().now()

    # Always run a debubble and sample
    playlist.append((ConductorStates.IFCB_DEBUBBLE,  'debubble'))
    playlist.append((ConductorStates.IFCB_RUNSAMPLE, 'runsample'))

    # Determine if we should flush the sample tube after each sample
    flush_after_sample = node.get_parameter('ifcb_maintenance.flush_after_sample').value
    if flush_after_sample:
        playlist.append((ConductorStates.IFCB_FLUSH, 'flushsampletube'))

    # Run IFCB steps in sequence
    for state_const, routine in playlist:
        # Wait for previous routine to finish before we submit a new one. The
        # loop exits after we start the 'runsample' routine, then we'll wait
        # for the position hold release.
        state.ifcb_is_idle.wait()

        node.get_logger().info(f'Starting {routine} routine')
        set_state(state_const)
        state.ifcb_is_idle.clear()
        request = RunRoutine.Request(routine=routine, instrument=True)
        result = ifcb_run_routine.call_async(request)
        ready = Event()
        result.add_done_callback(lambda _: ready.set())
        if not ready.wait(10):
            ifcb_run_routine.remove_pending_request(result)
            raise TimeoutError('IFCB routine service timed out')
        assert result.result().success

    # Wait for position hold release
    node.get_logger().info('Routines finished - awaiting position hold release')
    with state.position_hold_condition:
        state.position_hold_condition.wait_for(lambda: not state.position_hold)
        goal.finish(GoalStatus.STATUS_SUCCEEDED, RunIFCB.Result())
    node.get_logger().info('Position hold released')



def create_action_server(node: Node):
    global set_state
    node.declare_parameter('ifcb_maintenance.flush_after_sample', Parameter.Type.BOOL)

    for name in (
        'ifcb_maintenance.bead_interval', 'ifcb_maintenance.cartridge_debubble_interval',
        'ifcb_maintenance.clean_interval'
    ):
        declare_number(node, name)

    latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)

    # Publish state messages useful for debugging
    set_state = functools.partial(set_pub_state, node,
        node.create_publisher(ConductorState, '~/state', latched))

    # Subscribe to IFCB messages to track routine progress
    node.create_subscription(RawData, '/ifcb/in', functools.partial(on_ifcb_msg, node), 10)
    node.create_subscription(Bool, '/ifcb/is_routine_running', on_routine_running, latched)

    # Initialize service proxy for sending routines to the IFCB
    ifcb_run_routine = node.create_client(RunRoutine, '/ifcb/routine')
    if not ifcb_run_routine.wait_for_service():
        return None

    # Setup action server for running IFCB
    server = QueuedActionServer(node, RunIFCB, '~/sample',
                                functools.partial(run_sample_routines, node, ifcb_run_routine))

    # Set a fake timestamp for having run beads and catridge debubble, so that
    # we don't run it every startup and potentially waste time or bead supply.
    state.last_cart_debub_time = node.get_clock().now()
    state.last_bead_time = node.get_clock().now()
    state.last_clean_time = node.get_clock().now()

    return server


def main():
    rclpy.init()
    node = rclpy.create_node(f'ifcb_simple_{os.getpid()}_{random.getrandbits(32):08x}')
    node.get_logger().set_level(LoggingSeverity.DEBUG)
    executor = MultiThreadedExecutor()
    server = None
    try:
        server = create_action_server(node)
        if server is not None:
            rclpy.spin(node, executor=executor)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        if server is not None:
            server.server.destroy()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
