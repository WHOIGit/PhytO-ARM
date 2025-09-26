#!/usr/bin/env python3
"""
Simple arm implementation for basic movement commands only.
This arm moves to requested depths within configured bounds without any scientific payload operations.
"""
import rospy
from arm_base import ArmBase, Task


class ArmSimpleMove(ArmBase):
    def __init__(self, arm_name, winch_name=None):
        super().__init__(arm_name, winch_name)
        self.target_depth = None
        self.movement_complete = True

    def get_next_task(self, last_task):
        """
        Simple task logic: move to target depth if one is set, otherwise stay at current position.
        """
        if not rospy.get_param('winch_enabled'):
            return Task('no_winch', self.handle_no_winch)

        # Check for new target depth from parameter
        self.check_for_target_updates()

        # If we have a target depth to move to
        if self.target_depth is not None and self.movement_complete:
            target = self.target_depth
            self.target_depth = None  # Clear target after using it
            self.movement_complete = False
            return Task('move_to_target', self.handle_movement_complete, target)

        # Default behavior: stay at min depth
        if last_task is None:
            return Task('move_to_min', self.handle_movement_complete, rospy.get_param('winch/range/min'))

        # If no specific target, just wait and check again
        return Task('idle', self.handle_idle)

    def set_target_depth(self, depth):
        """
        Set a target depth for the arm to move to.
        Depth will be validated against winch range limits.
        """
        min_depth = rospy.get_param('winch/range/min')
        max_depth = rospy.get_param('winch/range/max')

        if depth < min_depth:
            rospy.logwarn(f'Requested depth {depth}m below minimum {min_depth}m, using minimum')
            depth = min_depth
        elif depth > max_depth:
            rospy.logwarn(f'Requested depth {depth}m above maximum {max_depth}m, using maximum')
            depth = max_depth

        self.target_depth = depth
        rospy.loginfo(f'Target depth set to {depth}m')

    def handle_movement_complete(self, move_result):
        """Handle completion of a movement task."""
        self.movement_complete = True
        rospy.loginfo('Movement completed successfully')
        self.start_next_task()

    def handle_no_winch(self):
        """Handle case where winch is disabled."""
        rospy.loginfo('Winch disabled, no movement possible')
        rospy.sleep(5)  # Wait 5 seconds before next task
        self.start_next_task()

    def handle_idle(self):
        """Handle idle state - wait and check for new targets."""
        rospy.sleep(1)  # Wait 1 second before checking for new tasks
        self.start_next_task()

    def check_for_target_updates(self):
        """Check for target depth updates from ROS parameters."""
        try:
            target_param = rospy.get_param('tasks/target_depth', None)
            if target_param is not None and target_param != self.target_depth:
                self.set_target_depth(target_param)
                # Clear the parameter after reading to avoid repeated moves to same depth
                rospy.delete_param('tasks/target_depth')
        except (KeyError, TypeError) as e:
            rospy.logdebug('Error checking target parameter: %s', e)


def main():
    rospy.init_node('arm_simple', log_level=rospy.DEBUG)

    winch_name = None
    if rospy.get_param('winch_enabled') is True:
        winch_name = rospy.get_namespace() + 'winch/move_to_depth'

    arm = ArmSimpleMove(rospy.get_name(), winch_name)

    rospy.loginfo('Simple arm %s ready for movement commands', rospy.get_name())
    rospy.loginfo('Set target depth with: rosparam set arm_simple_move/tasks/target_depth <depth_in_meters>')

    arm.loop()


if __name__ == '__main__':
    main()
