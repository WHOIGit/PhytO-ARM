#!/usr/bin/env python3
from dataclasses import dataclass
import functools
import math
from queue import Queue
from threading import Event, Condition

import actionlib
import numpy as np
import rospy

from phyto_arm.msg import MoveToDepthAction, MoveToDepthGoal, \
                        PayloadAction, PayloadGoal

from std_srvs.srv import Trigger, TriggerResponse

@dataclass
class Task:
    name: str
    depth: float
    callback: callable

class ArmBase:
    tasks = Queue()
    ready = Event()

    def __init__(self, name, winch_name=None):
        rospy.init_node(name, anonymous=True, log_level=rospy.DEBUG)

        # Build initial task queue
        self.build_task_queue()

        # Get winch path, setup action clinet
        winch_name = ''
        if rospy.get_param('winch/enabled') == True:
            winch_name = rospy.get_namespace() + 'winch/move_to_depth'
        
        # Setup service clients for acquiring permission to move winch
        acquire_winch = rospy.ServiceProxy('/winch_semaphore/acquire', Trigger)
        release_winch = rospy.ServiceProxy('/winch_semaphore/release', Trigger)

        if winch_name is not None:
            winch_client = actionlib.SimpleActionClient(
                winch_name,
                MoveToDepthAction
            )
            rospy.logwarn(f"Awaiting winch server for {winch_name}")
            winch_client.wait_for_server()


    def is_busy(self, action):
        state = action.get_state()
        return state == actionlib.GoalStatus.PENDING or state == actionlib.GoalStatus.ACTIVE


    def send_winch_goal(self, depth, callback):
        global winches_moving
        rospy.loginfo(f"Sending winch move to {depth}")
        # Ensure winch is enabled
        if rospy.get_param(f'winch/enabled') != True:
            rospy.logerr(f'Move aborted: winch is disabled in config')
            return
        # Safety check: Do not exceed depth bounds
        if depth < rospy.get_param(f'winch/range/min'):
            rospy.logerr(f'Move aborted: depth {depth} is below min {rospy.get_param("~range/min")}')
            return
        elif depth > rospy.get_param(f'winch/range/max'):
            rospy.logerr(f'Move aborted: depth {depth} is above max {rospy.get_param("~range/max")}')
            return
        assert not self.is_busy(self.winch_client)
        # Safety check: Do not exceed max concurrent winch movements
        if not self.acquire_winch():
            rospy.logerr(f'Move aborted: too many concurrent winch movements')
            return False

        rospy.loginfo(f"All checks passed; Moving winch to {depth}")

        # Callback executed after a winch move action completes
        def winch_done(state, result):
            # Ensure winch move was successful
            assert state == actionlib.GoalStatus.SUCCEEDED
            # Free up semaphore for another winch movement
            assert self.release_winch()
            callback(result)

        self.winch_client.send_goal(MoveToDepthGoal(depth=depth), done_cb=winch_done)


    # Initial task list
    def build_task_queue(self):
        # Clear queue. Do this instead of creating a new Queue to maintain thread safety
        with self.tasks.mutex:
            self.tasks.queue.clear()

    # Setup callback for when a task is complete
    def task_complete(self, result):
        self.ready.set()

    def loop(self, arm):
        while not rospy.is_shutdown():
            # Wait until payload is ready for more work
            self.ready.wait()
            self.ready.clear()
            task = self.tasks.get()
            rospy.loginfo(f'Received task {task.name}')

            # If no movement is required
            if self.winch_client is None:
                task.callback()
            # Otherwise, move winch
            else:
                self.send_winch_goal(arm, task.depth, task.callback)
        


