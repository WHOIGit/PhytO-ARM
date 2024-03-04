#!/usr/bin/env python3
import numpy as np
import rospy

from arm_base import ArmBase, Task
from phyto_arm.msg import DepthProfile

class ArmChanos(ArmBase):
    stage_index = 0
    latest_profile = None
    profiler_peak_value = None
    profiler_peak_depth = None
    profiler_peak_time = None

    # Primary method for determining the next task to execute
    # Each Task object takes:
    #  - name: a string identifying the task
    #  - callback: a function to call when the task is complete
    #  - depth: the depth to move to (optional, won't move if not provided)
    #  - speed: the speed to move at (optional, will use config max if not provided)
    def get_next_task(self, last_task):
        assert rospy.get_param('winch/enabled'), 'Winch is not enabled'

        # If the profiler peak task is enabled, prioritize first
        if rospy.get_param('tasks/profiler_peak/enabled'):
            profiler_peak_depth = get_profiler_peak()
            if profiler_peak_depth is not None:
                return Task('seek_profiler_peak', await_sensor, profiler_peak_depth)

        # If the last task was the staged cast, move into position for a slowcast
        if last_task is None or last_task.name in ['last_stage', 'seek_profiler_peak']:
            if rospy.get_param('tasks/slowcast/downcast'):
                goal_depth = rospy.get_param('winch/range/min')
            else:
                goal_depth = rospy.get_param('winch/range/max')
            return Task('slowcast_position', self.start_next_task, goal_depth)

        # If in the slowcast position, execute the slowcast
        if last_task.name == 'slowcast_position':
            if rospy.get_param('tasks/slowcast/downcast'):
                goal_depth = rospy.get_param('winch/range/max')
            else:
                goal_depth = rospy.get_param('winch/range/min')
            speed = rospy.get_param('tasks/slowcast/speed')
            return Task('slowcast', self.start_next_task, goal_depth, speed)
        
        # After slowcast, execute every stage in staged cast
        if last_task.name in ['slowcast', 'stage']:
            next_depth = stage_depth(self.stage_index)
            self.stage_index += 1

            # If at the last stage, reset counter and identify as last stage
            if self.stage_index == len(rospy.get_param('tasks/staged_cast/stages')):
                self.stage_index = 0
                return Task('last_stage', await_sensor, next_depth)
            
            # Otherwise, just move to the next stage
            return Task('stage', await_sensor, next_depth)
        raise ValueError(f'Unhandled next-task state where last task={last_task.name}')

# Global reference to arm state
arm = None


def get_profiler_peak():
    if arm.latest_profile is None:
        rospy.logwarn('No profiler peak available')
        return None
    
    if arm.profiler_peak_value < rospy.get_param('tasks/profiler_peak/threshold'):
        rospy.logwarn(f'Profiler peak value {arm.profiler_peak_value} is below threshold')
        return None
    
    # Check that last peak hasn't passed expiration window
    expiration_window = rospy.Duration(rospy.get_param('tasks/profiler_peak/peak_expiration'))
    expiration_time = arm.latest_profile.header.stamp + expiration_window
    if expiration_time < rospy.Time.now():
        rospy.logwarn(f'Profiler peak expired at {expiration_time}')
        return None

    return arm.profiler_peak_depth


def on_profile_msg(msg):
    arm.latest_profile = msg
    # Find the maximal value in the profile
    argmax = max(range(len(arm.latest_profile.values)), key=lambda i: arm.latest_profile.values[i])
    arm.profiler_peak_value = arm.latest_profile.values[argmax]
    arm.profiler_peak_depth = arm.latest_profile.depths[argmax]


def await_sensor(move_result):
    duration = rospy.get_param('tasks/staged_cast/stage_duration')
    rospy.loginfo(f"Waiting {duration} seconds for DC sensor to complete.")
    rospy.sleep(duration)
    rospy.loginfo('Done waiting for DC sensor to complete.')
    arm.start_next_task()


def stage_depth(index):
    stages = rospy.get_param('tasks/staged_cast/stages')
    return stages[index]


def main():
    global arm
    rospy.init_node('arm', anonymous=True, log_level=rospy.DEBUG)

    winch_name = None
    if rospy.get_param('winch/enabled') == True:
        winch_name = rospy.get_namespace() + 'winch/move_to_depth'
    arm = ArmChanos(rospy.get_name(), winch_name)

    # Subscribe to profiler messages that follow each transit
    rospy.Subscriber('/arm_ifcb/profiler', DepthProfile, on_profile_msg)

    arm.loop()

if __name__ == '__main__':
    main()
