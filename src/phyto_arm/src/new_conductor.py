#!/usr/bin/env python3
from collections import namedtuple
from functools import partial
from threading import RLock, Thread

import actionlib
import rospy

from phyto_arm.msg import MoveToDepthAction, MoveToDepthGoal, \
                        InstrumentAction, InstrumentGoal
from phyto_arm.srv import ArmRegistration, ArmTask

ArmTupple = namedtuple("Arms", ["namespace", "winch", "instrument", "get_task"])
arms = []
winches_moving = 0
lock = RLock()

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

    arm = ArmTupple(namespace=req.arm_namespace, \
                    winch=winch_client, \
                    instrument=instrument_client, \
                    get_task=task_client)
    arms.append(arm)
    rospy.logwarn(f'Arm {req.arm_namespace} registered successfully')
    return True, "Arm registered successfully"

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
    with lock:
        winches_moving += 1
    arm.winch.send_goal(MoveToDepthGoal(depth=depth), done_cb=partial(move_done, arm, instrument_arg))


def move_done(arm, instrument_arg, state, result):
    global winches_moving
    debug_arm(arm, f"Winch move done with state {state}")
    assert state == actionlib.GoalStatus.SUCCEEDED
    with lock:
        winches_moving -= 1
    # Tell instrument that requested the move it's time to run
    arm.instrument.send_goal(InstrumentAction(arg=instrument_arg, move_result=result))


def is_busy(action):
    state = action.get_state()
    return state == actionlib.GoalStatus.PENDING or state == actionlib.GoalStatus.ACTIVE


def loop():
    while not rospy.is_shutdown():
        # Rate limit to 20Hz since all actions are async
        rospy.sleep(0.05)
        for arm in arms:
            debug_arm(arm, f"Checking for new task")
            # See if instrument is ready for more work
            if is_busy(arm.instrument): 
                debug_arm(arm, f"Instrument is busy")
                continue
            task = arm.get_task()
            # If this instrument has no winch, just run task directly
            if arm.winch is None or task.hold:
                debug_arm(arm, f"Running task {task}")
                arm.instrument.send_goal(InstrumentGoal(arg=task.instrument_arg, move_result=None))
                continue
            # Ensure winch is free and movement allowed
            with lock:
                debug_arm(arm, f"Checking if winch is free")
                if is_busy(arm.winch) or winches_moving >= rospy.get_param('~max_moving_winches'):
                    continue
                # Otherwise, move the winch and run instrument
                send_winch_move(arm, task.depth, task.instrument_arg)


def main():
    global move_to_depth
    rospy.init_node('conductor', log_level=rospy.DEBUG)

    # Start the loop in a separate thread
    loop_thread = Thread(target=loop)
    loop_thread.start()

    # Setup service for registering new winch/instrument pairs
    rospy.Service('~register_arm', ArmRegistration, handle_registration)

    rospy.loginfo("Service path: /register_arm")

    # Keep your node from exiting until it has been shutdown
    rospy.spin()


if __name__ == "__main__":
    main()