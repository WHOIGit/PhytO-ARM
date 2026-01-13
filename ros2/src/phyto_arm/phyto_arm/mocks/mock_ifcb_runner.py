#!/usr/bin/env python3
import actionlib
import rospy

from phyto_arm.msg import ConductorState, ConductorStates, RunIFCBAction, RunIFCBResult

class MockIFCBActionServer:
    def __init__(self, name):
        self._action_server = actionlib.SimpleActionServer(
            name,
            RunIFCBAction,
            execute_cb=self.execute_cb,
            auto_start=False
        )
        self.state_publisher = rospy.Publisher('~state', ConductorState, queue_size=1, latch=True)
        self._action_server.start()

    def set_pub_state(self, s):
        m = ConductorState()
        m.header.stamp = rospy.Time.now()
        m.state = s
        self.state_publisher.publish(m)

    def execute_cb(self, goal):
        rospy.loginfo("Mock IFCB action server received a goal")

        # Simulate various IFCB routines
        routines = [
            ConductorStates.IFCB_DEBUBBLE,
            ConductorStates.IFCB_BEADS,
            ConductorStates.IFCB_BIOCIDE,
            ConductorStates.IFCB_BLEACH,
            ConductorStates.IFCB_RUNSAMPLE
        ]

        for routine in routines:
            self.set_pub_state(routine)
            rospy.loginfo(f"Mock IFCB running routine: {routine}")
            rospy.sleep(1)  # Simulate time taken for each routine

        # Simulate successful completion
        rospy.loginfo("Mock IFCB action succeeded")
        self._action_server.set_succeeded(RunIFCBResult())

def main():
    rospy.init_node('mock_ifcb_action_server')
    MockIFCBActionServer('~sample')
    rospy.spin()

if __name__ == '__main__':
    main()
