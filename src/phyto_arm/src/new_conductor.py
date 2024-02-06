#!/usr/bin/env python3
from collections import namedtuple
from dataclasses import dataclass
from functools import partial
import threading

import actionlib
import rospy

from phyto_arm.msg import MoveToDepthAction, MoveToDepthGoal, \
                        InstrumentAction, InstrumentGoal
from phyto_arm.srv import ArmRegistration, ArmTask

@dataclass
class Arm:
    namespace: str
    winch: actionlib.SimpleActionClient
    instrument: actionlib.SimpleActionClient
    get_task: rospy.ServiceProxy
    busy: threading.Event = threading.Event()

arms = []
winches_moving = 0
lock = threading.RLock()

def err_arm(arm, msg):
    rospy.logerr(f"[{arm.namespace}]: {msg}")


def debug_arm(arm, msg):
    rospy.logwarn(f"[{arm.namespace}]: {msg}")


def handle_registration(req):
    rospy.logwarn(f"Registering arm {req.arm_namespace} with {req.winch_name} and {req.instrument_name}")
    # If winch is used, setup client
    winch_client = None
    if req.winch_name != "":
        winch_client = actionlib.SimpleActionClient(
            req.winch_name,
            MoveToDepthAction
        )
        rospy.logwarn(f"Awaiting winch server for {req.winch_name}")
        winch_client.wait_for_server()

    # Setup action client for running instrument
    instrument_client = actionlib.SimpleActionClient(
        req.instrument_name,
        InstrumentAction
    )
    instrument_client.wait_for_server()

    # Set up service client for getting tasks
    task_client = rospy.ServiceProxy(req.task_server, ArmTask)
    rospy.logwarn(f"Awaiting task server for {req.winch_name}")
    task_client.wait_for_service()

    arm = Arm(namespace=req.arm_namespace, \
                    winch=winch_client, \
                    instrument=instrument_client, \
                    get_task=task_client)
    arms.append(arm)
    rospy.logwarn(f'Arm {req.arm_namespace} registered successfully')
    return True, "Arm registered successfully"


def is_busy(action):
    state = action.get_state()
    return state == actionlib.GoalStatus.PENDING or state == actionlib.GoalStatus.ACTIVE


# Callback executed after an instrument action completes
def instrument_done(arm, state, result):
    assert state == actionlib.GoalStatus.SUCCEEDED
    arm.busy.clear()


# Callback executed after a winch move action completes
def move_done(arm, instrument_arg, state, result):
    global winches_moving
    # Ensure winch move was successful and instrument ready
    assert state == actionlib.GoalStatus.SUCCEEDED
    assert not is_busy(arm.instrument)
    # Tell instrument requesting the move that it's time to run
    arm.instrument.send_goal(InstrumentGoal(arg=instrument_arg, move_result=result), done_cb=partial(instrument_done, arm))
    # Let loop know another winch can move now
    with lock:
        winches_moving -= 1


def send_winch_move(arm, depth, instrument_arg):
    global winches_moving
    debug_arm(arm, f"Sending winch move to {depth}")
    # Ensure winch is enabled
    if rospy.get_param(f'{arm.namespace}/winch/enabled') != True:
        err_arm(arm, f'Move aborted: winch is disabled in config')
        return
    # Safety check: Do not exceed depth bounds
    if depth < rospy.get_param(f'{arm.namespace}/winch/range/min'):
        err_arm(arm, f'Move aborted: depth {depth} is below min {rospy.get_param("~range/min")}')
        return
    elif depth > rospy.get_param(f'{arm.namespace}/winch/range/max'):
        err_arm(arm, f'Move aborted: depth {depth} is above max {rospy.get_param("~range/max")}')
        return
    assert not is_busy(arm.winch)
    with lock:
        winches_moving += 1
    debug_arm(arm, f"Checks passed; Moving winch to {depth}")
    arm.winch.send_goal(MoveToDepthGoal(depth=depth), done_cb=partial(move_done, arm, instrument_arg))



def loop():
    while not rospy.is_shutdown():
        # Rate limit to 20Hz since all actions are async
        rospy.sleep(0.05)
        for arm in arms:
            # See if instrument is ready for more work
            if arm.busy.is_set(): continue
            # Otherwise, get a task from the server
            task = arm.get_task()
            with lock:
                arm.busy.set()
                # If this instrument has no winch, just run task directly
                if arm.winch is None or task.hold:
                    debug_arm(arm, f"Running task:\n{task}")
                    arm.instrument.send_goal(InstrumentGoal(arg=task.instrument_arg, move_result=None), done_cb=partial(instrument_done, arm))
                # Otherwise, move winch and run instrument
                elif winches_moving < rospy.get_param('~max_moving_winches'):
                    send_winch_move(arm, task.depth, task.instrument_arg)
                # Or try again later
                else: 
                    # debug_arm(arm, f"Max winches already moving: {winches_moving}/{rospy.get_param('~max_moving_winches')}")
                    arm.busy.clear()
                


def main():
    global move_to_depth
    rospy.init_node('conductor', log_level=rospy.DEBUG)

    # Start the loop in a separate thread
    loop_thread = threading.Thread(target=loop)
    loop_thread.start()

    # Setup service for registering new winch/instrument pairs
    rospy.Service('~register_arm', ArmRegistration, handle_registration)

    rospy.loginfo("Service path: /register_arm")

    # Keep your node from exiting until it has been shutdown
    rospy.spin()


if __name__ == "__main__":
    main()
