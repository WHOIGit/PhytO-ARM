#!/usr/bin/env python3
from collections import namedtuple
from functools import partial
import math
from threading import RLock

import actionlib
import rospy

from phyto_arm.srv import ArmRegistration, ArmTask

from phyto_arm.msg import MoveToDepthAction, MoveToDepthGoal, \
                        InstrumentAction


ArmTupple = namedtuple("Arms", ["winch", "instrument", "get_task"])
arms = []
winches_moving = 0
lock = RLock()

def handle_registration(req):
    # If winch is used, setup client
    winch_client = None
    if req.winch_name != "":
        winch_client = actionlib.SimpleActionClient(
            req.winch_name,
            MoveToDepthAction
        )
        winch_client.wait_for_server()

    # Setup action client for running instrument
    instrument_client = actionlib.SimpleActionClient(
        req.instrument_name,
        InstrumentAction
    )
    instrument_client.wait_for_server()

    # Set up service client for getting tasks
    task_client = rospy.ServiceProxy(req.task_server, ArmTask)
    task_client.wait_for_server()

    arm = ArmTupple(winch=winch_client, instrument=instrument_client, get_task=task_client)
    arms.append(arm)
    return True, "Arm registered successfully"


def run_move_task(winch, depth, instrument, instrument_arg):
    # Safety check: Do not exceed depth bounds
    if not (rospy.get_param('~range/min') <= depth <= rospy.get_param('~range/max')):
        rospy.logerr('Refusing to exceed depth bounds')
        return
    with lock:
        winches_moving += 1
    winch.send_goal(MoveToDepthGoal(depth=depth), done_cb=partial(move_done, instrument, instrument_arg))


def move_done(instrument, instrument_arg, state, result):
    assert state == actionlib.GoalStatus.SUCCEEDED
    # Tell instrument that requested the move it's time to run
    instrument.send_goal(InstrumentAction(arg=instrument_arg, result=result))
    with lock:
        number_moving -= 1


def is_busy(action):
    state = action.get_state()
    return state == actionlib.GoalStatus.PENDING or state == actionlib.GoalStatus.ACTIVE


def loop():
    for arm in arms:
        # See if instrument is ready for more work
        if is_busy(arm.instrument): continue
        task = arm.get_task()
        # If this instrument has no winch, just run task directly
        if arm.winch is None or task.hold:
            arm.instrument.send_goal(InstrumentAction(arg=task.instrument_arg))
            continue
        # Ensure winch is free and movement allowed
        with lock:
            if is_busy(arm.winch) or winches_moving >= rospy.param('~max_moving_winches'):
                continue
            # Move the winch and run instrument
            run_move_task(arm.winch, task.depth, arm.instrument, task.instrument_arg)


def main():
    global move_to_depth

    rospy.init_node('conductor')

    # Setup service for registering new winch/instrument pairs
    rospy.Service('register_arm', ArmRegistration, handle_registration)


    while not rospy.is_shutdown():
        loop()
        # Rate limit to 20Hz since all actions are async
        rospy.sleep(0.05)


if __name__ == "__main__":
    main()