#!/usr/bin/env python3

import rclpy

from phyto_arm.arm_base import ArmBase, Task, arm_node
from phyto_arm_msgs.msg import DepthProfile
from phyto_sim_utils.parameters import declare_number
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.time import Time


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
        assert self.node.get_parameter('winch_enabled').value, 'Winch is not enabled'

        # Always upcast to start position first
        if last_task is None:
            self.is_downcasting = False
            return Task('upcast_continuous', self.start_next_task, self.node.get_parameter('winch.range.min').value)

        # If in the middle of stepped movement, continue to next step
        if self.steps:
            next_depth = self.steps.pop(0)
            return Task(last_task.name, await_sensor, next_depth)

        # Finish out the upcast when upcast steps are used
        if last_task.name in ['upcast_step', 'upcast_profile_step']:
            return Task('upcast_continuous', self.start_next_task, self.node.get_parameter('winch.range.min').value)

        # Finish out the downcast when downcast steps are used
        elif last_task.name in ['downcast_step', 'downcast_profile_step']:
            return Task('downcast_continuous', self.start_next_task, self.node.get_parameter('winch.range.max').value)
    
        # Finally if a cast has completed, switch direction and start the next cast
        elif last_task.name in ['upcast_continuous', 'downcast_continuous']: 
            self.is_downcasting = not self.is_downcasting

        # No other state should be possible
        else: 
            raise ValueError(f'Unexpected last task name: {last_task.name}')

        # Handle continuous movement cases
        speed = self.node.get_parameter('tasks.continuous_speed').value
        if self.is_downcasting and self.node.get_parameter('tasks.downcast_type').value == 'continuous':
            return Task('downcast_continuous', self.start_next_task, self.node.get_parameter('winch.range.max').value, speed)
        elif not self.is_downcasting and self.node.get_parameter('tasks.upcast_type').value == 'continuous':
            return Task('upcast_continuous', self.start_next_task, self.node.get_parameter('winch.range.min').value, speed)

        # If profiler peak is enabled and ready, execute new set of profile steps
        if self.node.get_parameter('tasks.profiler_peak.enabled').value and profiler_peak_ready():
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
        arm.node.get_logger().warning('No profiler peak available')
        return False

    if arm.profiler_peak_value < arm.node.get_parameter('tasks.profiler_peak.threshold').value:
        arm.node.get_logger().warning(f'Profiler peak value {arm.profiler_peak_value} is below threshold')
        return False

    # Check that last peak hasn't passed expiration window
    expiration_window = Duration(seconds=arm.node.get_parameter('tasks.profiler_peak.peak_expiration').value)
    expiration_time = Time.from_msg(arm.latest_profile.header.stamp) + expiration_window
    if expiration_time < arm.node.get_clock().now():
        arm.node.get_logger().warning(f'Profiler peak expired at {expiration_time}')
        return False

    return True


def clamp_steps(steps):
    DEPTH_MAX = arm.node.get_parameter('winch.range.max').value
    DEPTH_MIN = arm.node.get_parameter('winch.range.min').value
    clamped_steps = steps.copy()

    for i in range(len(steps)):
        if steps[i] > DEPTH_MAX:
            arm.node.get_logger().warning(f'Step {steps[i]} exceeds max depth, clamping to {DEPTH_MAX}')
            clamped_steps[i] = DEPTH_MAX
        if steps[i] < DEPTH_MIN:
            arm.node.get_logger().warning(f'Step {steps[i]} exceeds min depth, clamping to {DEPTH_MIN}')
            clamped_steps[i] = DEPTH_MIN

    return clamped_steps


def compute_absolute_steps(is_downcast):
    steps = arm.node.get_parameter('tasks.default_steps').value
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
    offsets = arm.node.get_parameter('tasks.profiler_peak.offset_steps').value

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
    duration = arm.node.get_parameter('tasks.dwell_time').value
    arm.node.get_logger().info(f'Waiting {duration} seconds for DC sensor to complete.')

    def timer_callback():
        if not rclpy.ok(context=arm.node.context):
            arm.node.get_logger().warning('Node shutdown detected. Aborting DC sensor wait.')
            return

        arm.node.destroy_timer(arm.timer)
        arm.timer = None
        arm.node.get_logger().info('Done waiting for DC sensor to complete.')
        arm.start_next_task()

    arm.timer = arm.node.create_timer(duration, timer_callback)


def main():
    global arm
    with arm_node('arm_chanos') as node:
        node.declare_parameters('', [
            ('tasks.downcast_type', Parameter.Type.STRING),
            ('tasks.profiler_peak.enabled', Parameter.Type.BOOL),
            ('tasks.upcast_type', Parameter.Type.STRING),
        ])

        for name in (
            'tasks.continuous_speed', 'tasks.dwell_time',
            'tasks.profiler_peak.peak_expiration', 'tasks.profiler_peak.threshold'
        ):
            declare_number(node, name)

        for name in ('tasks.default_steps', 'tasks.profiler_peak.offset_steps'):
            node.declare_parameter(name, descriptor=ParameterDescriptor(dynamic_typing=True))

        winch_name = None
        if node.get_parameter('winch_enabled').value is True:
            winch_name = 'winch/move_to_depth'
        arm = ArmChanos(node, winch_name)

        # Subscribe to profiler messages that follow each transit
        node.create_subscription(DepthProfile, '/arm_ifcb/profiler', on_profile_msg, 10)

        # Ensure the sensor timer isn't still running in shutdown event
        try:
            arm.loop()
        finally:
            if arm.timer:
                node.get_logger().info('Node shutdown requested. Canceling DC sensor wait timer.')
                node.destroy_timer(arm.timer)


if __name__ == '__main__':
    main()
