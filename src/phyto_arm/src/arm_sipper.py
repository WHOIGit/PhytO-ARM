#!/usr/bin/env python3

import actionlib
from os import path
import rospy

from arm_base import ArmBase, Task
from phyto_arm.msg import RunIFCBGoal, RunIFCBAction, OutletStatus
from std_msgs.msg import String

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
        sequence = rospy.get_param('sample_sequence')
        if self.sample_index is None:
            self.sample_index = 0
        else:
            self.sample_index = (self.sample_index + 1) % len(sequence)
        next_sample = sequence[self.sample_index]

        # Othrwise, start off at min
        if last_task is None:
            rospy.logwarn(f'Sample {next_sample}: starting at presample_pump')
            return Task('presample_pump', pump_seawater_for(rospy.get_param('task_durations/presample_pump')))

        if last_task.name in ['presample_pump', 'postsample_pump']:
            rospy.logwarn(f'Sample {next_sample}: starting at presample_drain')
            return Task('presample_drain', drain_for(rospy.get_param('task_durations/presample_drain')))

        if last_task.name == 'presample_drain':
            rospy.logwarn(f'Sample {next_sample}: starting at sample_pump')
            return Task('sample_pump', pump_sample_for(next_sample, rospy.get_param('task_durations/sample_pump')))

        if last_task.name == 'sample_pump':
            rospy.logwarn(f'Sample {next_sample}: starting at ifcb_valve_open')
            return Task('ifcb_valve_open', open_valve_and_run_ifcb_for(next_sample, rospy.get_param('task_durations/ifcb_valve_open')))

        if last_task.name == 'ifcb_valve_open':
            rospy.logwarn(f'Sample {next_sample}: starting at postsample_drain')
            return Task ('postsample_drain', drain_for(rospy.get_param('task_durations/postsample_drain')))

        if last_task.name == 'postsample_drain':
            rospy.logwarn(f'Sample {next_sample}: starting at postsample_pump')
            return Task('postsample_pump', pump_seawater_for(rospy.get_param('task_durations/postsample_pump')))

        raise ValueError(f'Unhandled next-task state where last task={last_task.name}')


# Global reference to action provided by other node
ifcb_runner = None
ifcb_data_dir = None
# Global reference to publishers
digital_logger_1_pub = None
digital_logger_2_pub = None
sample_location_pub = None
sample_depth_pub = None
sample_time_pub = None
sample_method_pub = None
# Global reference to arm state
arm = None


def timed_toggle(outlet, duration, digital_logger_pub):
    def outlet_toggle_callback():
        digital_logger_pub.publish(OutletStatus(str(outlet), True))
        rospy.sleep(duration)
        digital_logger_pub.publish(OutletStatus(str(outlet), False))
        arm.start_next_task()
    return outlet_toggle_callback

def pump_seawater_for(duration):
    outlet = rospy.get_param('seawater_pump_outlet')
    return timed_toggle(outlet, duration, digital_logger_1_pub)

def drain_for(duration):
    outlet = rospy.get_param('drain_outlet')
    return timed_toggle(outlet, duration, digital_logger_2_pub)

def pump_sample_for(next_sample, duration):
    sample_config = rospy.get_param(f'sample_metadata/{next_sample}')
    outlet = sample_config['outlet']
    return timed_toggle(outlet, duration, digital_logger_1_pub)

def publish_sample_metadata(sample_id):
    sample_metadata = rospy.get_param(f'sample_metadata/{sample_id}')
    sample_location_pub.publish(str(sample_metadata['location']))
    sample_depth_pub.publish(str(sample_metadata['depth']))
    sample_time_pub.publish(str(sample_metadata['time']))
    sample_method_pub.publish(str(sample_metadata['method']))

def open_valve_and_run_ifcb_for(next_sample, duration):
    def open_valve_and_run_ifcb_callback():

        # Set the data directory for IFCB to the sample subdirectory
        ifcb_subdir = rospy.get_param(f'sample_metadata/{next_sample}/ifcb_subdir')
        rospy.set_param('/ifcb/data_dir', path.join(ifcb_data_dir, ifcb_subdir))

        # Open the IFCB valve
        outlet = rospy.get_param('ifcb_valve_outlet')
        digital_logger_2_pub.publish(OutletStatus(str(outlet), True))

        # Run the IFCB and publish sample metadata
        goal = RunIFCBGoal()
        ifcb_runner.send_goal(goal)
        publish_sample_metadata(next_sample)
        # ifcb_runner.wait_for_result() # TODO: Should probably prefer this over sleep
        rospy.sleep(duration)

        # Close the IFCB valve and reset the data directory
        digital_logger_2_pub.publish(OutletStatus(str(outlet), False))
        rospy.set_param('/ifcb/data_dir', ifcb_data_dir)
        arm.start_next_task()
    return open_valve_and_run_ifcb_callback


def main():
    global arm
    global ifcb_runner
    global ifcb_data_dir
    global digital_logger_1_pub
    global digital_logger_2_pub
    global sample_location_pub
    global sample_depth_pub
    global sample_time_pub
    global sample_method_pub

    rospy.init_node('arm_sipper', log_level=rospy.DEBUG)

    arm = ArmSipper(rospy.get_name())

    # Save the data directory for IFCB
    ifcb_data_dir = rospy.get_param('/ifcb/data_dir')

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
    digital_logger_1_pub = rospy.Publisher('digital_logger_1/control', OutletStatus, queue_size=10)
    digital_logger_2_pub = rospy.Publisher('digital_logger_2/control', OutletStatus, queue_size=10)

    # Setup publisher for sample metadata. A string msg for each field
    sample_location_pub = rospy.Publisher('sample_metadata/location', String, queue_size=10)
    sample_depth_pub = rospy.Publisher('sample_metadata/depth', String, queue_size=10)
    sample_time_pub = rospy.Publisher('sample_metadata/time', String, queue_size=10)
    sample_method_pub = rospy.Publisher('sample_metadata/method', String, queue_size=10)

    arm.loop()

if __name__ == '__main__':
    main()
