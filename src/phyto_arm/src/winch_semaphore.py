#!/usr/bin/env python3
import rospy
from std_srvs.srv import Trigger, TriggerResponse
from threading import BoundedSemaphore

# This semaphore is used to limit the number of winches that can move at the same time.
# Necessary because any number of winches is supported and likely to be using the same
# power supply, so this allows us to limit concurrent movements to an arbitrary number.
movement_semaphore = None

def handle_acquire(req):
    acquired = movement_semaphore.acquire(blocking=False)
    return TriggerResponse(success=acquired)

def handle_release(req):
    try:
        movement_semaphore.release()
        return TriggerResponse(success=True)
    except ValueError:
        # Handle case where release is called more times than acquire
        rospy.logwarn("Semaphore release called without a corresponding acquire")
        return TriggerResponse(success=False)

if __name__ == "__main__":
    rospy.init_node('winch_semaphore')
    movement_semaphore = BoundedSemaphore(value=rospy.get_param('~max_moving_winches'))
    rospy.Service('~acquire', Trigger, handle_acquire)
    rospy.Service('~release', Trigger, handle_release)
    rospy.spin()
