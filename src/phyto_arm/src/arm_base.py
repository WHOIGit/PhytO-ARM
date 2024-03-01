#!/usr/bin/env python3
from dataclasses import dataclass
from threading import Lock

import actionlib
import rospy

from phyto_arm.msg import MoveToDepthAction, MoveToDepthGoal
from phyto_arm.srv import LockCheck, LockCheckRequest, LockOperation, LockOperationRequest


@dataclass
class Task:
    '''A task to be executed by an arm.
    - name: a string identifying the task
    - callback: a function to call when the task is complete
    - depth: the depth to move to (optional, won't move if not provided)
    - speed: the speed to move at (optional, will use config max if not provided)
    '''
    name: str
    callback: callable
    depth: float = None
    speed: float = None


# This is the base class for all arms. It should be subclassed and the get_next_task
# method should be overridden to provide the arm's specific task logic.
# Do not use this class directly.
class ArmBase:

    def __init__(self, arm_name, winch_name=None):
        self.arm_name = arm_name
        self.task_lock = Lock()

        if rospy.get_param('winch/enabled'):
            if winch_name is None:
                raise ValueError('Winch name must be provided if winch is enabled')

            # Setup service clients for acquiring permission to move winch. Prevents too many
            # winches from moving simultaneously
            acquire_move_clearance = rospy.ServiceProxy('/lock_manager/acquire', LockOperation)
            release_move_clearance = rospy.ServiceProxy('/lock_manager/release', LockOperation)
            lock_op = LockOperationRequest(arm_name)

            # Bind lock operations to instance
            self.request_clearance = lambda: acquire_move_clearance(lock_op).success
            self.release_clearance = lambda: release_move_clearance(lock_op).success

            # Await check lock
            check_lock = rospy.ServiceProxy('/lock_manager/check', LockCheck)
            rospy.loginfo('Awaiting lock manager check...')
            check_lock.wait_for_service()
            rospy.loginfo('Server acquired for lock check.')

            # Check whether we have a dangling acquisition to release
            if check_lock(LockCheckRequest(arm_name)).has_lock:
                rospy.logwarn('Releasing dangling lock acquisition')
                assert self.release_clearance(), 'Failed to release dangling lock'

            self.winch_client = actionlib.SimpleActionClient(
                winch_name,
                MoveToDepthAction
            )
            rospy.loginfo(f'Awaiting winch server for {winch_name}')
            self.winch_client.wait_for_server()
            rospy.loginfo(f'Server acquired for {winch_name}')


    def winch_busy(self):
        state = self.winch_client.get_state()
        return state == actionlib.GoalStatus.PENDING or state == actionlib.GoalStatus.ACTIVE


    def send_winch_goal(self, depth, speed, callback):
        global winches_moving
        rospy.loginfo(f'Sending winch move to {depth}')

        # Ensure winch is enabled
        if rospy.get_param('winch/enabled') is not True:
            raise ValueError('Move aborted: winch is disabled in config')

        # Safety check: Do not exceed depth bounds
        if depth < rospy.get_param('winch/range/min'):
            raise ValueError(f'Move aborted: depth {depth} is below min {rospy.get_param("winch/range/max")}')
        elif depth > rospy.get_param('winch/range/max'):
            raise ValueError(f'Move aborted: depth {depth} is above max {rospy.get_param("winch/range/max")}')

        # Set max speed if not specified
        if speed is None:
            speed = rospy.get_param('winch/max_speed')

        # Safety check: Speed cannot exceed max speed
        elif speed > rospy.get_param('winch/max_speed'):
            raise ValueError(f'Move aborted: speed {speed} is above max {rospy.get_param("winch/max_speed")}')
        if self.winch_busy():
            raise RuntimeError('Move aborted: winch in unexpected busy state')

        rospy.loginfo(f'All checks passed; Moving winch to {depth}')

        # Intermediate callback for ensuring move was a success and to release lock
        # before calling the task's callback
        def winch_done(state, move_result):
            try:
                # Ensure winch move was successful
                assert state == actionlib.GoalStatus.SUCCEEDED

                # Free up semaphore for another winch movement
                assert self.release_clearance()
                callback(move_result)
            except Exception as e:
                rospy.logerr(f'Unexpected error: {e}')
                rospy.signal_shutdown(f'Shutting down due to unexpected error: {e}')
                raise e

        self.winch_client.send_goal(MoveToDepthGoal(depth=depth, velocity=speed), done_cb=winch_done)


    # Logic for determining arm tasks goes heres, all implementations should override this
    def get_next_task(self, last_task):
        return None


    # Callback for when a task is complete. Unused result argument is required
    # for cases where this method is used as a task callback
    def start_next_task(self, move_result=None):
        self.task_lock.release()


    # Primary loop that should be the same for all arms, override should not be needed
    def loop(self):
        task = None
        while not rospy.is_shutdown():
            # Don't block so that we can keeping checking for shutdowns
            if self.task_lock.acquire(blocking=False):
                # If no movement is required
                if self.winch_client is None:
                    task = self.get_next_task(task)
                    rospy.logwarn(f'No winch; running {task.name}')
                    task.callback()

                # Otherwise, move winch
                else:
                    rospy.logwarn('Arm waiting for clearance')

                    # TODO: Consider replacing this with a queueing mechanism.  Requires setting up
                    # callbacks via ROS service calls. Unnecessarily complex for a 2 winch system,
                    # but might be the only way to achieve round-robin for many winches.

                    while not rospy.is_shutdown() and not self.request_clearance():
                        # Wait until central semaphore clears us to move
                        rospy.sleep(1)

                    # TODO: Optimize task evaluation. Currently we are blocking on the assumption that
                    # movement will be needed; this is not always the case. Not a problem for 1 or 2
                    # winches, but could get slow if the number of winches greatly exceeds the
                    # max_moving_winches limit.

                    task = self.get_next_task(task)
                    rospy.logwarn(f'Goal depth {task.depth} for task {task.name}')
                    self.send_winch_goal(task.depth, task.speed, task.callback)
