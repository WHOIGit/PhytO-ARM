#!/usr/bin/env python3
import numpy as np
import rospy
from threading import Lock

from arm_base import ArmBase, Task
from phyto_arm.msg import DepthProfile


class ArmChanos(ArmBase):
    latest_profile = None
    profiler_peak_value = None
    profiler_peak_depth = None
    profiler_peak_time = None

    profiler_steps = []
    absolute_steps = []

    timer = None

    # Primary method for determining the next task to execute
    # Each Task object takes:
    #  - name: a string identifying the task
    #  - callback: a function to call when the task is complete
    #  - depth: the depth to move to (optional, won't move if not provided)
    #  - speed: the speed to move at (optional, will use config max if not provided)
    def get_next_task(self, last_task):
        assert rospy.get_param('winch/enabled'), 'Winch is not enabled'

        # If the profiler peak task is available, keep looping through profiler steps
        if rospy.get_param('tasks/profiler_peak/enabled'):
            if not self.profiler_steps and profiler_peak_ready():
                self.profiler_steps = compute_profiler_steps()
            if self.profiler_steps:
                next_depth = self.profiler_steps.pop(0)
                return Task('profiler_step', await_sensor, next_depth)

        # For continuous profile, figure out the start and goal depths
        continuous_start, continuous_goal = continuous_direction()

        # If the last task was the step cast, move into position for a continuous cast
        if last_task is None or last_task.name in ['profiler_step', 'last_step']:
            return Task('continuous_position', self.start_next_task, continuous_start)

        # If in the continuous position, execute the continuous profile
        if last_task.name == 'continuous_position':
            speed = rospy.get_param('tasks/continuous_profile/speed')
            return Task('continuous_profile', self.start_next_task, continuous_goal, speed)

        # After continuous, execute every step in stepped cast
        if last_task.name in ['continuous_profile', 'absolute_step']:

            # Steps are always regenerated after a continuous profile
            if last_task.name == 'continuous_profile':
                self.absolute_steps = rospy.get_param('tasks/step_profile/steps')
            next_depth = self.absolute_steps.pop(0)

            # Detect the last step to end the loop
            if not self.absolute_steps:
                return Task('last_step', await_sensor, next_depth)

            return Task('absolute_step', await_sensor, next_depth)
        raise ValueError(f'Unhandled next-task state where last task={last_task.name}')

# Global reference to arm state
arm = None


def profiler_peak_ready():
    if arm.latest_profile is None:
        rospy.logwarn('No profiler peak available')
        return False

    if arm.profiler_peak_value < rospy.get_param('tasks/profiler_peak/threshold'):
        rospy.logwarn(f'Profiler peak value {arm.profiler_peak_value} is below threshold')
        return False

    # Check that last peak hasn't passed expiration window
    expiration_window = rospy.Duration(rospy.get_param('tasks/profiler_peak/peak_expiration'))
    expiration_time = arm.latest_profile.header.stamp + expiration_window
    if expiration_time < rospy.Time.now():
        rospy.logwarn(f'Profiler peak expired at {expiration_time}')
        return False

    return True


def compute_profiler_steps():
    if arm.latest_profile is None:
        raise ValueError('No profiler peak available')

    depths = []
    offsets = rospy.get_param('tasks/profiler_peak/offset_steps')

    DEPTH_MAX = rospy.get_param('winch/range/max')
    DEPTH_MIN = rospy.get_param('winch/range/min')

    for offset in offsets:
        next_depth = arm.profiler_peak_depth + offset

        # Ensure none of the steps exceeds max or min bounds
        if next_depth > DEPTH_MAX:
            rospy.logwarn('Profiler peak step exceeds max depth, clamping to max')
            next_depth = DEPTH_MAX
        if next_depth < DEPTH_MIN:
            rospy.logwarn('Profiler peak step exceeds min depth, clamping to min')
            next_depth = DEPTH_MIN
        depths.append(next_depth)

    return depths


def on_profile_msg(msg):
    arm.latest_profile = msg
    # Find the maximal value in the profile
    argmax = max(range(len(arm.latest_profile.values)), key=lambda i: arm.latest_profile.values[i])
    arm.profiler_peak_value = arm.latest_profile.values[argmax]
    arm.profiler_peak_depth = arm.latest_profile.depths[argmax]


def continuous_direction():
    direction = rospy.get_param('tasks/continuous_profile/direction')
    if direction == 'up':
        continuous_start = rospy.get_param('winch/range/max')
        continuous_goal = rospy.get_param('winch/range/min')
    elif direction == 'down':
        continuous_start = rospy.get_param('winch/range/min')
        continuous_goal = rospy.get_param('winch/range/max')
    else:
        raise ValueError(f'Unexpected continuous profile direction: {direction}')
    return continuous_start, continuous_goal


def await_sensor(move_result):
    duration = rospy.get_param('tasks/dwell_time')
    rospy.loginfo(f'Waiting {duration} seconds for DC sensor to complete.')

    def timer_callback(event):
        if rospy.is_shutdown():
            rospy.logwarn('Node shutdown detected. Aborting DC sensor wait.')
            return

        rospy.loginfo('Done waiting for DC sensor to complete.')
        arm.start_next_task()

    arm.timer = rospy.Timer(rospy.Duration(duration), timer_callback, oneshot=True)


def main():
    global arm
    rospy.init_node('arm_chanos', log_level=rospy.DEBUG)

    winch_name = None
    if rospy.get_param('winch/enabled') == True:
        winch_name = rospy.get_namespace() + 'winch/move_to_depth'
    arm = ArmChanos(rospy.get_name(), winch_name)

    # Subscribe to profiler messages that follow each transit
    rospy.Subscriber('/arm_ifcb/profiler', DepthProfile, on_profile_msg)

    # Ensure the sensor timer isn't still running in shutdown event
    def shutdown_hook(reason):
        if arm.timer:
            rospy.loginfo('Node shutdown requested. Canceling DC sensor wait timer.')
            arm.timer.shutdown()
    rospy.core.add_preshutdown_hook(shutdown_hook)

    arm.loop()

if __name__ == '__main__':
    main()
