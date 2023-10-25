#!/usr/bin/env python3
import functools
import math
import threading
import sys

import actionlib
import numpy as np
import rospy

from ifcbclient.protocol import parse_response as parse_ifcb_msg
from ifcb.instrumentation import parse_marker as parse_ifcb_marker

from ds_core_msgs.msg import RawData
from std_srvs.srv import Empty

from ifcb.srv import RunRoutine

from phyto_arm.srv import ArmRegistration, ArmTaskResponse
from phyto_arm.msg import ConductorState, DepthProfile, \
                          InstrumentAction


# Global references to service provided by other node
ifcb_run_routine = None


# Convenience function to publish state updates for debugging
set_state
def set_pub_state(pub, s):
    m = ConductorState()
    m.header.stamp = rospy.Time.now()
    m.state = s
    pub.publish(m)


# N.B.: This is an abuse of Python syntax, don't do as I do!
class state:
    profile_activity = threading.Event()
    latest_profile = None
    tasks = []


def on_profile_msg(msg):
    # Alert waiting threads to the new profile message
    state.latest_profile = msg
    state.profile_activity.set()
    state.profile_activity.clear()


# Responds to conductor with the next task
def get_task_handler():
    if not state.tasks:
        return ArmTaskResponse(hold=True)
    return state.tasks[0]


# Handles action callback from the conductor
def instrument_handler(arg, result):
    # If we got here via a task, remove it
    if (arg == "peak_depth"): state.tasks.pop(0)
    
    # Wait for the profile data for this cast
    while state.latest_profile is None or \
        state.latest_profile.goal_uuid != result.uuid:
        state.profile_activity.wait()

    # Find the maximal value in the profile
    argmax = max(range(len(state.latest_profile.values)),
                key=lambda i: state.latest_profile.values[i])
    target_depth = state.latest_profile.depths[argmax]
    
    # Move ESP to the peak depth
    state.tasks.append(ArmTaskResponse(depth=target_depth, arg="peak_depth"))


def main():
    global set_state
    rospy.init_node('arm', anonymous=True, log_level=rospy.DEBUG)

    # Publish state messages useful for debugging
    set_state = functools.partial(set_pub_state,
        rospy.Publisher('~state', ConductorState, queue_size=1, latch=True))

    # Subscribe to profiler messages from the AML arm
    rospy.Subscriber(rospy.get_param('profiler_topic'), DepthProfile, on_profile_msg)

    # Setup service for fetching tasks
    service_name = rospy.get_namespace() + 'arm/get_task'
    rospy.Service(service_name, Empty, get_task_handler)

    # Setup action server for waiting while ESP samples
    instrument_name = rospy.get_namespace() + 'arm/run_instrument'
    server = actionlib.SimpleActionServer(instrument_name, InstrumentAction, instrument_handler, False)
    server.start()

    # Get winch path
    winch_name = rospy.get_namespace() + 'winch/move_to_depth'

    # Register with conductor
    register = rospy.ServiceProxy('/conductor/register_arm', ArmRegistration)
    register.wait_for_service()
    registration = ArmRegistration()
    registration.winch_name = winch_name
    registration.instrument_name = instrument_name
    registration.task_server = service_name
    register(registration)


if __name__ == '__main__':
    main()
