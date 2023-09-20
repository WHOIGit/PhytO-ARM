
from collections import namedtuple
import math
from threading import Lock

import actionlib
import rospy
from std_srvs.srv import Empty

from phyto_arm.msg import RegisterCallback, MoveToDepthAction, MoveToDepthGoal, \
                        RunInstrumentAction


InstrumentObj = namedtuple("Instruments", "status", "goal_depth", "run", "winch")
instruments = []
winches_moving = 0
move_to_depth = None
lock = Lock()

def handle_registration(req):
    # Setup service clients
    status_client = rospy.ServiceProxy(req.status_callback, Empty)
    goal_depth_client = rospy.ServiceProxy(req.goal_depth_callback, Empty)

    # Setup action client for controlling the winch
    winch_client = actionlib.SimpleActionClient(
        req.move_winch_callback,
        MoveToDepthAction
    )
    winch_client.wait_for_server()

    # Setup action client for running instrument
    run_client = actionlib.SimpleActionClient(
        req.run_instrument_callback,
        RunInstrumentAction
    )
    run_client.wait_for_server()

    # Add clients to instruments registry
    instrument = InstrumentObj(status=status_client, \
                    goal_depth=goal_depth_client, \
                    run=run_client, \
                    winch=winch_client)
    instruments.append(instrument)

    return True, "Instrument registered successfully"


def move_to_depth(winch, depth):
    with lock:
        winch.send_goal(MoveToDepthGoal(depth=depth), done_cb=move_callback)
        winches_moving += 1


def move_callback(self, state, result):
    with self.lock:
        self.number_moving -= 1


def is_busy(action):
    state = action.get_state()
    return state == actionlib.GoalStatus.PENDING or state == actionlib.GoalStatus.ACTIVE


def loop():
    for instrument in instruments:
        try:
            # See if instrument is ready for more work
            if is_busy(instrument.winch) or is_busy(instrument.run): continue
            goal_depth = instrument.goal_depth()
            if goal_depth.satisfied:
                # Do the science
                instrument.run.send_goal(RunInstrumentAction())
            else:
                # Check concurrent movement
                with lock:
                    if winches_moving >= rospy.param('~max_moving_winches'):
                        continue
                # Move the winch
                move_to_depth(instrument.winch, goal_depth.depth)

        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {str(e)}")


def main():
    global move_to_depth

    rospy.init_node('conductor')

    # Setup service for registering new winch/instrument pairs
    rospy.Service('register_instrument', RegisterCallback, handle_registration)

    while not rospy.is_shutdown():
        # Check for new callbacks
        rospy.spin_once()
        loop()
        # Rate limit to 20Hz since all actions are async
        rospy.sleep(0.05)


if __name__ == "__main__":
    main()