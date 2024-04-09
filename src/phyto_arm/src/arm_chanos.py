#!/usr/bin/env python3

import rospy

from arm_base import ArmBase, Task
from phyto_arm.msg import DepthProfile


class ArmChanos(ArmBase):
    latest_profile = None
    profiler_peak_value = None
    profiler_peak_depth = None
    profiler_peak_time = None

    steps = []

    timer = None

    is_downcasting = True

    # Primary method for determining the next task to execute
    # Each Task object takes:
    #  - name: a string identifying the task
    #  - callback: a function to call when the task is complete
    #  - depth: the depth to move to (optional, won't move if not provided)
    #  - speed: the speed to move at (optional, will use config max if not provided)
    def get_next_task(self, last_task):
        assert rospy.get_param('winch/enabled'), 'Winch is not enabled'

        # Always upcast to start position first
        if last_task is None:
            self.is_downcasting = False
            return Task('upcast_continuous', self.start_next_task, rospy.get_param('winch/range/min'))

        # If in the middle of stepped movement, continue to next step
        if self.steps:
            next_depth = self.steps.pop(0)
            return Task(last_task.name, await_sensor, next_depth)

        # Finish out the upcast when upcast steps are used
        if last_task.name in ['upcast_step', 'upcast_profile_step']:
            return Task('upcast_continuous', self.start_next_task, rospy.get_param('winch/range/min'))

        # Finish out the downcast when downcast steps are used
        elif last_task.name in ['downcast_step', 'downcast_profile_step']:
            return Task('downcast_continuous', self.start_next_task, rospy.get_param('winch/range/max'))
    
        # Finally if a cast has completed, switch direction and start the next cast
        elif last_task.name in ['upcast_continuous', 'downcast_continuous']: 
            self.is_downcasting = not self.is_downcasting

        # No other state should be possible
        else: 
            raise ValueError(f'Unexpected last task name: {last_task.name}')

        # Handle continuous movement cases
        speed = rospy.get_param('tasks/continuous_speed')
        if self.is_downcasting and rospy.get_param('tasks/downcast_type') == 'continuous':
            return Task('downcast_continuous', self.start_next_task, rospy.get_param('winch/range/max'), speed)
        elif not self.is_downcasting and rospy.get_param('tasks/upcast_type') == 'continuous':
            return Task('upcast_continuous', self.start_next_task, rospy.get_param('winch/range/min'), speed)

        # If profiler peak is enabled and ready, execute new set of profile steps
        if rospy.get_param('tasks/profiler_peak/enabled') and profiler_peak_ready():
            self.steps = compute_profiler_steps(self.is_downcasting)
            task_name = 'downcast_profile_step' if self.is_downcasting else 'upcast_profile_step'
            next_depth = self.steps.pop(0)
            return Task(task_name, await_sensor, next_depth)

        # If no profiler peak, execute default profile
        if self.is_downcasting:
            self.steps = compute_absolute_steps(self.is_downcasting)
            next_depth = self.steps.pop(0)
            return Task('downcast_step', await_sensor, next_depth)
        else:
            self.steps = compute_absolute_steps(self.is_downcasting)
            next_depth = self.steps.pop(0)
            return Task('upcast_step', await_sensor, next_depth)


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


def clamp_steps(steps):
    DEPTH_MAX = rospy.get_param('winch/range/max')
    DEPTH_MIN = rospy.get_param('winch/range/min')
    clamped_steps = steps.copy()

    for i in range(len(steps)):
        if steps[i] > DEPTH_MAX:
            rospy.logwarn(f'Step {steps[i]} exceeds max depth, clamping to {DEPTH_MAX}')
            clamped_steps[i] = DEPTH_MAX
        if steps[i] < DEPTH_MIN:
            rospy.logwarn(f'Step {steps[i]} exceeds min depth, clamping to {DEPTH_MIN}')
            clamped_steps[i] = DEPTH_MIN

    return clamped_steps


def compute_absolute_steps(is_downcast):
    steps = rospy.get_param('tasks/default_steps')
    steps = clamp_steps(steps)

    # Sort according to direction.
    # If downcasting, use default ascending order.
    # Otherwise, reverse to get descending order.
    # Remember that in this context "ascending order" means "increasing depth".
    steps.sort(reverse=not is_downcast)
    return steps


def compute_profiler_steps(is_downcast):
    if arm.latest_profile is None:
        raise ValueError('No profiler peak available')

    depths = []
    offsets = rospy.get_param('tasks/profiler_peak/offset_steps')

    # Add each offset to peak depth to get list of depths,
    # then clamp within min/max range
    for offset in offsets:
        next_depth = arm.profiler_peak_depth + offset
        depths.append(next_depth)
    depths = clamp_steps(depths)

    # Sort according to direction
    if is_downcast:
        depths.sort()
    else:
        depths.sort(reverse=True)

    return depths


def on_profile_msg(msg):
    arm.latest_profile = msg

    # Find the maximal value in the profile
    argmax = max(range(len(arm.latest_profile.values)), key=lambda i: arm.latest_profile.values[i])
    arm.profiler_peak_value = arm.latest_profile.values[argmax]
    arm.profiler_peak_depth = arm.latest_profile.depths[argmax]


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
    if rospy.get_param('winch/enabled') is True:
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
