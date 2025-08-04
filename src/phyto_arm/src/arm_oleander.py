#!/usr/bin/env python3
import math

from datetime import datetime, timedelta
from threading import Event

import actionlib
import numpy as np
import rospy
from std_msgs.msg import Bool

from arm_base import ArmBase, Task
from phyto_arm.msg import RunIFCBGoal, RunIFCBAction


class ArmOleander(ArmBase):
    last_cart_debub_time = None
    last_bead_time = None

    # Primary method for determining the next task to execute
    # Each Task object takes:
    #  - name: a string identifying the task
    #  - callback: a function to call when the task is complete
    #  - depth: the depth to move to (optional, won't move if not provided)
    #  - speed: the speed to move at (optional, will use config max if not provided)
    def get_next_task(self, last_task):
        # Always check if we're in the geofence
        if self.geofence_block():
            if last_task is None or last_task.name == 'shutdown_sampling':
                return Task('await_geofence', await_geofence)
            # If not, shut down sampling
            return Task('shutdown_sampling', shutdown_sampling)

        if last_task is None or last_task.name == 'await_geofence':
            # Restart ifcb and pump
            return Task('restart_sampling', restart_sampling)

        # Keep sampling until we are outside the geofence
        if last_task.name in ['restart_sampling', 'ifcb_sample']:
            return Task('ifcb_sample', ifcb_sample)

        raise ValueError(f'Unhandled next-task state where last task={last_task.name}')


# Global reference to action provided by other node
ifcb_runner = None

# Global reference to arm state
arm = None

# DL outlet publishers
dl_pump_pub = None
dl_ifcb_pub = None

def send_ifcb_action():
    goal = RunIFCBGoal()
    ifcb_runner.send_goal(goal)
    ifcb_runner.wait_for_result()

def await_geofence():
    rospy.loginfo('Waiting for geofence to be cleared')
    geofence_check_interval = rospy.get_param('tasks/geofence_check_interval')
    while arm.geofence_block():
        rospy.sleep(geofence_check_interval)
    rospy.loginfo('Geofence cleared, continuing')
    arm.start_next_task()

def ifcb_sample():
    send_ifcb_action()
    arm.start_next_task()

def shutdown_sampling():
    dl_pump_pub.publish(Bool(data=False))

    # Wait for the IFCB to finish its current sample
    shutdown_duration = rospy.get_param('tasks/shutdown_wait_duration')
    rospy.logwarn(f'Shutting down sampling for {shutdown_duration} seconds')
    rospy.sleep(shutdown_duration)
    dl_ifcb_pub.publish(Bool(data=False))
    arm.start_next_task()

def restart_sampling():
    dl_pump_pub.publish(Bool(data=True))
    dl_ifcb_pub.publish(Bool(data=True))

    # Wait for IFCB to start up
    restart_duration = rospy.get_param('tasks/restart_wait_duration')
    rospy.logwarn(f'Waiting for IFCB to start up for {restart_duration} seconds')
    rospy.sleep(restart_duration)
    arm.start_next_task()

def main():
    global arm
    global ifcb_runner
    global dl_pump_pub
    global dl_ifcb_pub

    rospy.init_node('arm_oleander', log_level=rospy.DEBUG)

    arm = ArmOleander(rospy.get_name())

    # Setup publisher for DL outlet
    dl_pump_pub = rospy.Publisher('/digital_logger/outlet/pump/status', Bool, queue_size=10)
    dl_ifcb_pub = rospy.Publisher('/digital_logger/outlet/ifcb/status', Bool, queue_size=10)

    # Setup action client for running IFCB
    ifcb_runner = actionlib.SimpleActionClient('ifcb_runner/sample', RunIFCBAction)
    rospy.loginfo(f'Arm {rospy.get_name()} awaiting IFCB-run action server')
    ifcb_runner.wait_for_server()
    rospy.loginfo(f'Arm {rospy.get_name()} IFCB-run action server acquired')

    # Set a fake timestamp for having run beads and catridge debubble, so that
    # we don't run it every startup and potentially waste time or bead supply.
    arm.last_cart_debub_time = rospy.Time.now()
    arm.last_bead_time = rospy.Time.now()

    arm.loop()

if __name__ == '__main__':
    main()
