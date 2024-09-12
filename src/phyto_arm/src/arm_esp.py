#!/usr/bin/env python3

import rospy

from arm_base import ArmBase, Task
from phyto_arm.msg import DepthProfile


class ArmESP(ArmBase):
    latest_profile = None
    profiler_peak_value = None
    profiler_peak_depth = None

    steps = []

    timer = None

    # Primary method for determining the next task to execute
    # Each Task object takes:
    #  - name: a string identifying the task
    #  - callback: a function to call when the task is complete
    #  - depth: the depth to move to (optional, won't move if not provided)
    #  - speed: the speed to move at (optional, will use config max if not provided)
    def get_next_task(self, last_task):
        assert rospy.get_param('winch_enabled'), 'Winch is not enabled'

        # If profiler peak is enabled and ready, execute new set of profile steps
        if rospy.get_param('tasks/profiler_peak/enabled') and profiler_peak_ready():
            peak_depth = clamped_profiler_peak()
            esp_update_freq = rospy.get_param('tasks/profiler_peak/update_frequency')
            return Task("profiler_peak", wait_seconds(esp_update_freq), peak_depth)

        # By default hold and wait a second
        return Task('default_position_wait', wait_seconds(5), rospy.get_param('tasks/default_depth'))

# Global reference to arm state
arm = None


def profiler_peak_ready():
    if arm.latest_profile is None:
        rospy.logwarn('No profiler peak available')
        return False

    if arm.profiler_peak_value < rospy.get_param('tasks/profiler_peak/threshold'):
        rospy.logwarn(f'Profiler peak value {arm.profiler_peak_value} is below threshold')
        return False

    return True


def clamped_profiler_peak():
    DEPTH_MAX = rospy.get_param('winch/range/max')
    DEPTH_MIN = rospy.get_param('winch/range/min')
    depth = arm.profiler_peak_depth

    if depth > DEPTH_MAX:
        rospy.logwarn(f'Peak depth {depth} exceeds max depth, clamping to {DEPTH_MAX}')
        return DEPTH_MAX
    if depth < DEPTH_MIN:
        rospy.logwarn(f'Peak depth {depth} exceeds min depth, clamping to {DEPTH_MIN}')
        return DEPTH_MIN

    return depth


def on_profile_msg(msg):
    arm.latest_profile = msg

    # Find the maximal value in the profile
    argmax = max(range(len(arm.latest_profile.values)), key=lambda i: arm.latest_profile.values[i])
    arm.profiler_peak_value = arm.latest_profile.values[argmax]
    arm.profiler_peak_depth = arm.latest_profile.depths[argmax]


def wait_seconds(duration):
    def awaiter(move_result):
        def timer_callback(event):
            if rospy.is_shutdown():
                return
            arm.start_next_task()

        arm.timer = rospy.Timer(rospy.Duration(duration), timer_callback, oneshot=True)
    return awaiter


def main():
    global arm
    rospy.init_node('arm_esp', log_level=rospy.DEBUG)

    winch_name = None
    if rospy.get_param('winch_enabled') is True:
        winch_name = rospy.get_namespace() + 'winch/move_to_depth'
    arm = ArmESP(rospy.get_name(), winch_name)

    # Subscribe to profiler messages that follow each transit
    rospy.Subscriber('/arm_ifcb/profiler', DepthProfile, on_profile_msg)

    # Ensure the sensor timer isn't still running in shutdown event
    def shutdown_hook(reason):
        if arm.timer:
            arm.timer.shutdown()
    rospy.core.add_preshutdown_hook(shutdown_hook)

    arm.loop()

if __name__ == '__main__':
    main()
