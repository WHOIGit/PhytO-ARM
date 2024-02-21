#!/usr/bin/env python3
from dataclasses import dataclass
import functools
import math
from queue import Queue
from threading import Lock

import actionlib
import numpy as np
import rospy

from phyto_arm.msg import MoveToDepthAction, MoveToDepthGoal

from std_srvs.srv import Trigger, TriggerResponse



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
    task_lock = Lock()

    def __init__(self, winch_name=None):
        # Get winch path, setup action clinet
        winch_name = ''
        if rospy.get_param('winch/enabled') == True:
            winch_name = rospy.get_namespace() + 'winch/move_to_depth'
        
        # Setup service clients for acquiring permission to move winch. Prevents too many
        # winches from moving simultaneously
        self.acquire_move_clearance = rospy.ServiceProxy('/winch_semaphore/acquire', Trigger)
        self.release_move_clearance = rospy.ServiceProxy('/winch_semaphore/release', Trigger)

        if winch_name is not None:
            self.winch_client = actionlib.SimpleActionClient(
                winch_name,
                MoveToDepthAction
            )
            rospy.loginfo(f"Awaiting winch server for {winch_name}")
            self.winch_client.wait_for_server()
            rospy.loginfo(f"Server acquired for {winch_name}")


    def winch_busy(self):
        state = self.winch_client.get_state()
        return state == actionlib.GoalStatus.PENDING or state == actionlib.GoalStatus.ACTIVE


    def send_winch_goal(self, depth, speed, callback):
        global winches_moving
        rospy.loginfo(f"Sending winch move to {depth}")
        # Ensure winch is enabled
        if rospy.get_param(f'winch/enabled') != True:
            rospy.logerr(f'Move aborted: winch is disabled in config')
            return
        # Safety check: Do not exceed depth bounds
        if depth < rospy.get_param(f'winch/range/min'):
            rospy.logerr(f'Move aborted: depth {depth} is below min {rospy.get_param("winch/range/max")}')
            return
        elif depth > rospy.get_param(f'winch/range/max'):
            rospy.logerr(f'Move aborted: depth {depth} is above max {rospy.get_param("winch/range/max")}')
            return
        # Set max speed if not specified
        if speed is None:
            speed = rospy.get_param('winch/max_speed')
        # Safety check: Speed cannot exceed max speed
        elif speed > rospy.get_param(f'winch/max_speed'):
            rospy.logerr(f'Move aborted: speed {speed} is above max {rospy.get_param("winch/max_speed")}')
            return
        assert not self.winch_busy()
        rospy.loginfo(f"All checks passed; Moving winch to {depth}")


        # Intermediate callback for ensuring move was a success and to release semaphore 
        # before calling the task's callback
        def winch_done(state, move_result):
            try:
                # Ensure winch move was successful
                assert state == actionlib.GoalStatus.SUCCEEDED
                # Free up semaphore for another winch movement
                assert self.release_move_clearance()
                callback(move_result)
            except Exception as e:
                rospy.logerr(f"Unexpected error: {e}")
                rospy.signal_shutdown(f"Shutting down due to unexpected error: {e}")
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
                    # Wait until central semaphore clears us to move
                    while not rospy.is_shutdown() and not self.acquire_move_clearance():
                        rospy.sleep(1)
                    # TODO: Optimize task evaluation. Currently we are blocking on the assumption that
                    # movement will be needed; this is not always the case. Not a problem for 1 or 2
                    # winches, but could get slow if the number of winches greatly exceeds the
                    # max_moving_winches limit.
                    task = self.get_next_task(task)
                    rospy.logwarn(f'Sending winch goal for {task.name}')
                    self.send_winch_goal(task.depth, task.speed, task.callback)

