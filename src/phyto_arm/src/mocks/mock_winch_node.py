#!/usr/bin/env python3
import rospy
import actionlib
from phyto_arm.msg import MoveToDepthAction, MoveToDepthFeedback, MoveToDepthResult
from std_msgs.msg import Float64  # Assuming depth is published as a simple float

class MockActionServer:
    def __init__(self, name, action_spec):
        self._action_server = actionlib.SimpleActionServer(
            name,
            action_spec,
            execute_cb=self.execute_cb,
            auto_start=False
        )
        self.depth_publisher = rospy.Publisher('mock_depth', Float64, queue_size=10)
        self._action_server.start()

    def execute_cb(self, goal):
        rospy.loginfo(f"Starting mocked move to depth {goal.depth}")

        # Simulate progress
        for i in range(10):
            rospy.sleep(2)  # Simulate time passing
            current_depth = goal.depth * i / 10  # Simulate depth change
            feedback = MoveToDepthFeedback()
            feedback.depth = current_depth
            self._action_server.publish_feedback(feedback)
            rospy.logdebug(f"Mocked depth progress: {feedback.depth}")

            # Publish the current mock depth
            self.depth_publisher.publish(Float64(current_depth))

        # Simulate successful completion
        if self._action_server.is_preempt_requested():
            rospy.loginfo("Preempted")
            self._action_server.set_preempted()
        else:
            result = MoveToDepthResult()
            result.time_elapsed.data = rospy.Duration.from_sec(10)  # Simulated duration
            rospy.loginfo("Mocked action succeeded")
            self._action_server.set_succeeded(result)

def main():
    rospy.init_node('mock_winch')
    rospy.logdebug(f'Starting mock winch node {rospy.get_name()}')
    server = MockActionServer('winch/move_to_depth', MoveToDepthAction)
    rospy.spin()

if __name__ == '__main__':
    main()
