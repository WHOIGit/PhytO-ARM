#!/usr/bin/env python3
import numpy as np
import rospy

from arm_base import ArmBase, Task


class ArmChanos(ArmBase):
    stage_index = 0

    # Primary method for determining the next task to execute
    # Each Task object takes:
    #  - name: a string identifying the task
    #  - callback: a function to call when the task is complete
    #  - depth: the depth to move to (optional, won't move if not provided)
    #  - speed: the speed to move at (optional, will use config max if not provided)
    def get_next_task(self, last_task):
        assert rospy.get_param('winch/enabled'), 'Winch is not enabled'

        if last_task is None:
            return Task("upcast", self.start_next_task, rospy.get_param('winch/range/min'))

        if last_task.name == "upcast":
            downcast_depth = rospy.get_param('winch/range/max')
            downcast_speed = rospy.get_param('tasks/downcast/speed')
            return Task("downcast", self.start_next_task, downcast_depth, downcast_speed)

        if last_task.name in ["downcast", "upcast_stage"]:
            next_depth = stage_depth(self.stage_index)
            self.stage_index += 1
            if self.stage_index == len(rospy.get_param('tasks/upcast/stages')):
                self.stage_index = 0
            return Task("upcast_stage", await_stage, next_depth)

        raise ValueError(f"Unhandled next-task state where last task={last_task.name}")

# Global reference to arm state
arm = None


def await_stage(move_result):
    duration = rospy.get_param('tasks/upcast/stage_duration')
    rospy.loginfo(f"Waiting {duration} seconds for DC sensor to complete.")
    rospy.sleep(duration)
    rospy.loginfo('Done waiting for DC sensor to complete.')
    arm.start_next_task()


def stage_depth(index):
    stages = rospy.get_param('tasks/upcast/stages')
    return stages[index]


def main():
    global arm
    rospy.init_node('arm', anonymous=True, log_level=rospy.DEBUG)

    winch_name = None
    if rospy.get_param('winch/enabled') == True:
        winch_name = rospy.get_namespace() + 'winch/move_to_depth'
    arm = ArmChanos(rospy.get_name(), winch_name)

    arm.loop()

if __name__ == '__main__':
    main()
