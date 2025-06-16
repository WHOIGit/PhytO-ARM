#!/usr/bin/env python3

import actionlib
import rospy

from arm_base import ArmBase, Task
from phyto_arm.msg import RunIFCBGoal, RunIFCBAction, OutletStatus


class ArmSipper(ArmBase):
    sample_index = None

    last_cart_debub_time = None
    last_bead_time = None

    # Primary method for determining the next task to execute
    # Each Task object takes:
    #  - name: a string identifying the task
    #  - callback: a function to call when the task is complete
    #  - depth: the depth to move to (optional, won't move if not provided)
    #  - speed: the speed to move at (optional, will use config max if not provided)
    def get_next_task(self, last_task):
        sequence = rospy.get_param('sequence')
        if self.sample_index is None:
            self.sample_index = 0
        else:
            self.sample_index = (self.sample_index + 1) % len(sequence)
        next_sample = sequence[self.sample_index]

        # Othrwise, start off at min
        if last_task is None:
            return Task('presample_pump', pump_seawater_for(rospy.get_param('task_durations/presample_pump')))

        if last_task.name in ['presample_pump', 'postsample_pump']:
            return Task('presample_drain', drain_for(rospy.get_param('task_durations/presample_drain')))

        if last_task.name in ['presample_drain', 'postsample_drain']:
            return Task('sample_pump', pump_sample_for(next_sample, rospy.get_param('task_durations/sample_pump')))

        if last_task.name is 'presample_pump':
            return Task('ifcb_valve_open', open_valve_and_run_ifcb_for(rospy.get_param('task_durations/ifcb_valve_open')))

        if last_task.name is 'ifcb_valve_open':
            return Task ('postsample_drain', drain_for(rospy.get_param('task_durations/postsample_drain')))

        if last_task.name in ['postsample_drain']:
            return Task('postsample_pump', pump_seawater_for(rospy.get_param('task_durations/postsample_pump')))

        raise ValueError(f'Unhandled next-task state where last task={last_task.name}')


# Global reference to action provided by other node
ifcb_runner = None
# Global reference to digital logger publisher
digital_logger_pub = None
# Global reference to arm state
arm = None


def timed_toggle(outlet, duration):
    def outlet_toggle_callback():
        digital_logger_pub.publish(OutletStatus(outlet, True))
        rospy.sleep(duration)
        digital_logger_pub.publish(OutletStatus(outlet, False))
    return outlet_toggle_callback

def pump_seawater_for(duration):
    outlet = rospy.get_param('seawater_pump_outlet')
    return timed_toggle(outlet, duration)


def drain_for(duration):
    outlet = rospy.get_param('drain_outlet')
    return timed_toggle(outlet, duration)


def pump_sample_for(next_sample, duration):
    sample_config = rospy.get_param(f'samples/{next_sample}')
    outlet = sample_config['outlet']
    return timed_toggle(outlet, duration)


def open_valve_and_run_ifcb_for(duration):
    def open_valve_and_run_ifcb_callback():
        outlet = rospy.get_param('ifcb_valve_outlet')
        digital_logger_pub.publish(OutletStatus(outlet, True))
        goal = RunIFCBGoal()
        ifcb_runner.send_goal(goal)
        ifcb_runner.wait_for_result()
        digital_logger_pub.publish(OutletStatus(outlet, False))
    return open_valve_and_run_ifcb_callback


def main():
    global arm
    global ifcb_runner
    global digital_logger_pub

    rospy.init_node('arm_sipper', log_level=rospy.DEBUG)

    arm = ArmSipper(rospy.get_name())


    # Setup action client for running IFCB
    ifcb_runner = actionlib.SimpleActionClient('ifcb_runner/sample', RunIFCBAction)
    rospy.loginfo(f'Arm {rospy.get_name()} awaiting IFCB-run action server')
    ifcb_runner.wait_for_server()
    rospy.loginfo(f'Arm {rospy.get_name()} IFCB-run action server acquired')

    # Set a fake timestamp for having run beads and catridge debubble, so that
    # we don't run it every startup and potentially waste time or bead supply.
    arm.last_cart_debub_time = rospy.Time.now()
    arm.last_bead_time = rospy.Time.now()

    # Setup publisher for digital logger control
    digital_logger_pub = rospy.Publisher('/digital_logger/control', OutletStatus, queue_size=10)

    arm.loop()

if __name__ == '__main__':
    main()
