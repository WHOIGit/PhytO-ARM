#!/usr/bin/env python3
from collections import namedtuple
from dataclasses import dataclass
from functools import partial
from threading import BoundedSemaphore, Event, Thread

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
    ready: Event = Event()


# Global list of registered arms
arms = []
# Thread safe counter for limiting concurrent winch movements
movement_semaphore = None


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

    # Default to ready
    arm.ready.set()
    # Start loop for this arm in a new thread
    Thread(target=partial(loop, arm)).start()

    return True, "Arm registered successfully"


def is_busy(action):
    state = action.get_state()
    return state == actionlib.GoalStatus.PENDING or state == actionlib.GoalStatus.ACTIVE


def send_instrument_goal(arm, task, callback, move_result=None):
    # Ensure instrument is ready
    assert not is_busy(arm.instrument)

    # Callback executed after an instrument action completes
    def instrument_done(state, result):
        assert state == actionlib.GoalStatus.SUCCEEDED
        callback()

    goal = InstrumentGoal(name=task.name, args=task.args, move_result=move_result)
    arm.instrument.send_goal(goal, done_cb=instrument_done)



def send_winch_goal(arm, depth, callback):
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
    # Safety check: Do not exceed max concurrent winch movements
    movement_semaphore.acquire()
    debug_arm(arm, f"All checks passed; Moving winch to {depth}")

    # Callback executed after a winch move action completes
    def winch_done(state, result):
        global winches_moving
        # Ensure winch move was successful
        assert state == actionlib.GoalStatus.SUCCEEDED
        # Free up semaphore for another winch movement
        movement_semaphore.release()
        callback(result)

    arm.winch.send_goal(MoveToDepthGoal(depth=depth), done_cb=winch_done)



def loop(arm):
    while not rospy.is_shutdown():
        # Wait until instrument is ready for more work
        arm.ready.wait()
        arm.ready.clear()
        task = arm.get_task()
        rospy.loginfo(f'[{arm.namespace}]: Received task {task.name}')
        # Setup callback for when task is complete
        def task_complete():
            rospy.loginfo(f'[{arm.namespace}]: Task {task.name} complete')
            arm.ready.set()

        # If no movement is required
        if arm.winch is None:
            # Then run instrument
            send_instrument_goal(arm, task, task_complete)
        # Otherwise, move winch
        else:
            send_winch_goal(arm, task.depth, \
                            # Then run instrument
                            lambda result: 
                                send_instrument_goal(arm, task, task_complete, move_result=result))


def main():
    global move_to_depth
    global movement_semaphore
    rospy.init_node('conductor', log_level=rospy.DEBUG)

    # Setup semaphore for limiting concurrent winch movements
    movement_semaphore = BoundedSemaphore(rospy.get_param('~max_moving_winches'))

    # Setup service for registering new winch/instrument pairs
    rospy.Service('~register_arm', ArmRegistration, handle_registration)

    # Keep your node from exiting until it has been shutdown
    rospy.spin()


if __name__ == "__main__":
    main()
