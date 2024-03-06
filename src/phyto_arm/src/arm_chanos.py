#!/usr/bin/env python3
import numpy as np
import rospy
from threading import Lock

from arm_base import ArmBase, Task
from phyto_arm.msg import DepthProfile


class ArmChanos(ArmBase):
    step_index = 0
    latest_profile = None
    profiler_peak_value = None
    profiler_peak_depth = None
    profiler_peak_time = None

    profiler_step_index = 0


    # Primary method for determining the next task to execute
    # Each Task object takes:
    #  - name: a string identifying the task
    #  - callback: a function to call when the task is complete
    #  - depth: the depth to move to (optional, won't move if not provided)
    #  - speed: the speed to move at (optional, will use config max if not provided)
    def get_next_task(self, last_task):
        assert rospy.get_param('winch/enabled'), 'Winch is not enabled'

        # If the profiler peak task is enabled, prioritize first
        if rospy.get_param('tasks/profiler_peak/enabled') and profiler_peak_ready():
            if last_task is None or last_task.name == 'profile_step':
                next_depth = next_profiler_depth(self.profiler_step_index)
                self.profiler_step_index += 1
                if self.step_index == len(rospy.get_param('tasks/profiler_peak/offset_steps')):
                    self.step_index = 0
                    return Task('profile_last_step', await_sensor, next_depth)
                return Task('profile_step', await_sensor, next_depth)

        # For continuous profile, figure out the start and goal depths
        direction = rospy.get_param('tasks/continuous_profile/direction')
        if direction == 'up':
            continuous_start = rospy.get_param('winch/range/max')
            continuous_goal = rospy.get_param('winch/range/min')
        elif direction == 'down':
            continuous_start = rospy.get_param('winch/range/min')
            continuous_goal = rospy.get_param('winch/range/max')
        else:
            raise ValueError(f'Unexpected continuous profile direction: {direction}')

        # If the last task was the step cast, move into position for a continuous cast
        if last_task is None or last_task.name in ['last_step', 'seek_profiler_peak']:
            return Task('continuous_position', self.start_next_task, continuous_start)

        # If in the continuous position, execute the continuous profile
        if last_task.name == 'continuous_position':
            speed = rospy.get_param('tasks/continuous_profile/speed')
            return Task('continuous_profile', self.start_next_task, continuous_goal, speed)
        
        # After continuous, execute every step in stepped cast
        if last_task.name in ['continuous_profile', 'step']:
            next_depth = step_depth(self.step_index)
            self.step_index += 1

            # If at the last step, reset counter and identify as last step
            if self.step_index == len(rospy.get_param('tasks/step_profile/steps')):
                self.step_index = 0
                return Task('last_step', await_sensor, next_depth)
            
            # Otherwise, just move to the next step
            return Task('step', await_sensor, next_depth)
        raise ValueError(f'Unhandled next-task state where last task={last_task.name}')

# Global reference to arm state
arm = None


def profiler_peak_ready():
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



def next_profiler_depth(index):
    if arm.latest_profile is None:
        raise ValueError('No profiler peak available')
    
    steps = rospy.get_param('tasks/profiler_peak/offset_steps')
    offset = steps[index]

    next_depth = arm.profiler_peak_depth + offset

    if next_depth > rospy.get_param('winch/range/max'):
        rospy.logwarn('Next peak profile depth exceeds max depth, using max depth')
        next_depth = rospy.get_param('winch/range/max')
    if next_depth < rospy.get_param('winch/range/min'):
        rospy.logwarn('Next peak profile depth is less than min depth, using min depth')
        next_depth = rospy.get_param('winch/range/min')

    return next_depth


def on_profile_msg(msg):
    arm.latest_profile = msg
    # Find the maximal value in the profile
    argmax = max(range(len(arm.latest_profile.values)), key=lambda i: arm.latest_profile.values[i])
    arm.profiler_peak_value = arm.latest_profile.values[argmax]
    arm.profiler_peak_depth = arm.latest_profile.depths[argmax]


def await_sensor(move_result):
    duration = rospy.get_param('tasks/dwell_time')
    rospy.loginfo(f'Waiting {duration} seconds for DC sensor to complete.')
    rospy.sleep(duration)
    rospy.loginfo('Done waiting for DC sensor to complete.')
    arm.start_next_task()


def step_depth(index):
    steps = rospy.get_param('tasks/step_profile/steps')
    return steps[index]


def main():
    global arm
    rospy.init_node('arm_chanos', log_level=rospy.DEBUG)

    winch_name = None
    if rospy.get_param('winch/enabled') == True:
        winch_name = rospy.get_namespace() + 'winch/move_to_depth'
    arm = ArmChanos(rospy.get_name(), winch_name)

    # Subscribe to profiler messages that follow each transit
    rospy.Subscriber('/arm_ifcb/profiler', DepthProfile, on_profile_msg)

    arm.loop()

if __name__ == '__main__':
    main()
