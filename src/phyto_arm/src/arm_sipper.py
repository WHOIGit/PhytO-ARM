#!/usr/bin/env python3

from os import path
import rospy

import actionlib
from arm_base import ArmBase, Task
from std_msgs.msg import String
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
        try:
            samples = rospy.get_param('~samples')
        except KeyError:
            rospy.logerr("Missing required parameter '~samples'. Shutting down node.")
            rospy.signal_shutdown("Missing configuration")
            return None

        if self.sample_index is None:
            self.sample_index = 0
        else:
            self.sample_index = (self.sample_index + 1) % len(samples)

        next_sample = samples[self.sample_index]
        next_sample_name = next_sample.get('name', f'sample_{self.sample_index}')

        try:
            task_durations = rospy.get_param('~task_durations')
        except KeyError:
            rospy.logerr("Missing required parameter '~task_durations'. Shutting down node.")
            rospy.signal_shutdown("Missing configuration")
            return None

        # Start at presample_pump if no previous task
        if last_task is None:
            rospy.logwarn(f'Sample {next_sample_name}: starting at presample_pump')
            return Task('presample_pump', pump_seawater_for(task_durations['presample_pump']))

        if last_task.name in ['presample_pump', 'postsample_pump']:
            rospy.logwarn(f'Sample {next_sample_name}: starting at presample_drain')
            return Task('presample_drain', drain_for(task_durations['presample_drain']))

        if last_task.name == 'presample_drain':
            rospy.logwarn(f'Sample {next_sample_name}: starting at sample_pump')
            return Task('sample_pump', pump_sample_for(next_sample, task_durations['sample_pump']))

        if last_task.name == 'sample_pump':
            rospy.logwarn(f'Sample {next_sample_name}: starting at ifcb_valve_open')
            return Task('ifcb_valve_open', open_valve_and_run_ifcb_for(next_sample, task_durations['ifcb_valve_open']))

        if last_task.name == 'ifcb_valve_open':
            rospy.logwarn(f'Sample {next_sample_name}: starting at postsample_drain')
            return Task('postsample_drain', drain_for(task_durations['postsample_drain']))

        if last_task.name == 'postsample_drain':
            rospy.logwarn(f'Sample {next_sample_name}: starting at postsample_pump')
            return Task('postsample_pump', pump_seawater_for(task_durations['postsample_pump']))

        raise ValueError(f'Unhandled next-task state where last task={last_task.name}')


# Global references
ifcb_runner = None
ifcb_data_dir = None
digital_logger_publishers = {}  # Maps digital logger names to publishers
metadata_publishers = {}  # Maps metadata keys to publishers
arm = None


def validate_config():
    """Validate that all required configuration parameters exist and are correct."""
    required_params = [
        '~samples',
        '~task_durations', 
        '~non_sample_power',
        '~digital_loggers'
    ]

    for param in required_params:
        if not rospy.has_param(param):
            rospy.logerr(f"Missing required parameter '{param}'. Shutting down node.")
            rospy.signal_shutdown("Missing configuration")
            return False

    # Validate samples structure
    try:
        samples = rospy.get_param('~samples')
        if not isinstance(samples, list) or len(samples) == 0:
            rospy.logerr("Parameter '~samples' must be a non-empty list. Shutting down node.")
            rospy.signal_shutdown("Invalid configuration")
            return False

        for i, sample in enumerate(samples):
            if not isinstance(sample, dict):
                rospy.logerr(f"Sample {i} must be a dictionary. Shutting down node.")
                rospy.signal_shutdown("Invalid configuration")
                return False

            required_sample_keys = ['ifcb_subdir', 'power', 'metadata']
            for key in required_sample_keys:
                if key not in sample:
                    rospy.logerr(f"Sample {i} missing required key '{key}'. Shutting down node.")
                    rospy.signal_shutdown("Invalid configuration")
                    return False

            # Validate power structure
            power = sample['power']
            if 'digital_logger' not in power or 'outlet' not in power:
                rospy.logerr(f"Sample {i} power section must have 'digital_logger' and 'outlet'. Shutting down node.")
                rospy.signal_shutdown("Invalid configuration")
                return False

    except Exception as e:
        rospy.logerr(f"Error validating samples configuration: {e}. Shutting down node.")
        rospy.signal_shutdown("Invalid configuration")
        return False

    # Validate non_sample_power structure
    try:
        non_sample_power = rospy.get_param('~non_sample_power')
        required_power_items = ['seatwater_pump', 'drain_valve', 'ifcb_valve']
        for item in required_power_items:
            if item not in non_sample_power:
                rospy.logerr(f"Missing required non_sample_power item '{item}'. Shutting down node.")
                rospy.signal_shutdown("Invalid configuration")
                return False

            power_config = non_sample_power[item]
            if 'digital_logger' not in power_config or 'outlet' not in power_config:
                rospy.logerr(f"non_sample_power item '{item}' must have 'digital_logger' and 'outlet'. Shutting down node.")
                rospy.signal_shutdown("Invalid configuration")
                return False

    except Exception as e:
        rospy.logerr(f"Error validating non_sample_power configuration: {e}. Shutting down node.")
        rospy.signal_shutdown("Invalid configuration")
        return False

    return True


def setup_digital_logger_publishers():
    """Create publishers for each digital logger based on configuration."""
    try:
        digital_loggers = rospy.get_param('~digital_loggers')
        for dl_name in digital_loggers.keys():
            topic_name = f'{dl_name}/control'
            digital_logger_publishers[dl_name] = rospy.Publisher(topic_name, OutletStatus, queue_size=10)
            rospy.loginfo(f"Created publisher for digital logger '{dl_name}' on topic '{topic_name}'")
    except Exception as e:
        rospy.logerr(f"Error setting up digital logger publishers: {e}. Shutting down node.")
        rospy.signal_shutdown("Configuration error")


def get_digital_logger_publisher(digital_logger_name):
    """Get the publisher for a given digital logger name."""
    if digital_logger_name not in digital_logger_publishers:
        rospy.logerr(f"Digital logger '{digital_logger_name}' not found in configuration. Shutting down node.")
        rospy.signal_shutdown("Configuration error")
        return None
    return digital_logger_publishers[digital_logger_name]


def get_metadata_publisher(metadata_key):
    """Get or create a publisher for a given metadata key."""
    if metadata_key not in metadata_publishers:
        topic_name = f'sample_metadata/{metadata_key}'
        metadata_publishers[metadata_key] = rospy.Publisher(topic_name, String, queue_size=10)
        rospy.loginfo(f"Created publisher for metadata key '{metadata_key}' on topic '{topic_name}'")
    return metadata_publishers[metadata_key]


def timed_toggle(outlet, duration, digital_logger_pub):
    def outlet_toggle_callback():
        digital_logger_pub.publish(OutletStatus(str(outlet), True))
        rospy.sleep(duration)
        digital_logger_pub.publish(OutletStatus(str(outlet), False))
        arm.start_next_task()
    return outlet_toggle_callback


def pump_seawater_for(duration):
    try:
        power_config = rospy.get_param('~non_sample_power/seatwater_pump')
        digital_logger_name = power_config['digital_logger']
        outlet = power_config['outlet']

        publisher = get_digital_logger_publisher(digital_logger_name)
        if publisher is None:
            return None

        return timed_toggle(outlet, duration, publisher)
    except KeyError as e:
        rospy.logerr(f"Missing configuration for seawater pump: {e}. Shutting down node.")
        rospy.signal_shutdown("Configuration error")
        return None


def drain_for(duration):
    try:
        power_config = rospy.get_param('~non_sample_power/drain_valve')
        digital_logger_name = power_config['digital_logger']
        outlet = power_config['outlet']

        publisher = get_digital_logger_publisher(digital_logger_name)
        if publisher is None:
            return None

        return timed_toggle(outlet, duration, publisher)
    except KeyError as e:
        rospy.logerr(f"Missing configuration for drain valve: {e}. Shutting down node.")
        rospy.signal_shutdown("Configuration error")
        return None


def pump_sample_for(sample_config, duration):
    try:
        power_config = sample_config['power']
        digital_logger_name = power_config['digital_logger']
        outlet = power_config['outlet']

        publisher = get_digital_logger_publisher(digital_logger_name)
        if publisher is None:
            return None

        return timed_toggle(outlet, duration, publisher)
    except KeyError as e:
        rospy.logerr(f"Missing power configuration for sample: {e}. Shutting down node.")
        rospy.signal_shutdown("Configuration error")
        return None


def publish_sample_metadata(sample_config):
    """Dynamically publish all metadata fields for a sample."""
    try:
        metadata = sample_config['metadata']
        for key, value in metadata.items():
            publisher = get_metadata_publisher(key)
            publisher.publish(str(value))
            rospy.logdebug(f"Published metadata {key}: {value}")
    except KeyError as e:
        rospy.logerr(f"Missing metadata in sample configuration: {e}")
    except Exception as e:
        rospy.logerr(f"Error publishing sample metadata: {e}")


def open_valve_and_run_ifcb_for(sample_config, duration):
    def open_valve_and_run_ifcb_callback():
        try:
            # Set the data directory for IFCB to the sample subdirectory
            ifcb_subdir = sample_config['ifcb_subdir']
            rospy.set_param('/ifcb/data_dir', path.join(ifcb_data_dir, ifcb_subdir))

            # Open the IFCB valve
            power_config = rospy.get_param('~non_sample_power/ifcb_valve')
            digital_logger_name = power_config['digital_logger']
            outlet = power_config['outlet']

            publisher = get_digital_logger_publisher(digital_logger_name)
            if publisher is None:
                return

            publisher.publish(OutletStatus(str(outlet), True))

            # Run the IFCB and publish sample metadata
            goal = RunIFCBGoal()
            ifcb_runner.send_goal(goal)
            publish_sample_metadata(sample_config)
            # ifcb_runner.wait_for_result() # TODO: Should probably prefer this over sleep
            rospy.sleep(duration)

            # Close the IFCB valve and reset the data directory
            publisher.publish(OutletStatus(str(outlet), False))
            rospy.set_param('/ifcb/data_dir', ifcb_data_dir)
            arm.start_next_task()
        except Exception as e:
            rospy.logerr(f"Error in open_valve_and_run_ifcb_for: {e}. Shutting down node.")
            rospy.signal_shutdown("Runtime error")
    return open_valve_and_run_ifcb_callback


def main():
    global arm
    global ifcb_runner
    global ifcb_data_dir

    rospy.init_node('arm_sipper', log_level=rospy.DEBUG)

    # Validate configuration first
    if not validate_config():
        return

    arm = ArmSipper(rospy.get_name())

    # Setup digital logger publishers
    setup_digital_logger_publishers()

    # Save the data directory for IFCB
    try:
        ifcb_data_dir = rospy.get_param('/ifcb/data_dir')
    except KeyError:
        rospy.logerr("Missing required parameter '/ifcb/data_dir'. Shutting down node.")
        rospy.signal_shutdown("Missing configuration")
        return

    # Setup action client for running IFCB
    ifcb_runner = actionlib.SimpleActionClient('ifcb_runner/sample', RunIFCBAction)
    rospy.loginfo(f'Arm {rospy.get_name()} awaiting IFCB-run action server')
    ifcb_runner.wait_for_server()
    rospy.loginfo(f'Arm {rospy.get_name()} IFCB-run action server acquired')

    # Set a fake timestamp for having run beads and cartridge debubble, so that
    # we don't run it every startup and potentially waste time or bead supply.
    arm.last_cart_debub_time = rospy.Time.now()
    arm.last_bead_time = rospy.Time.now()

    rospy.loginfo("Arm sipper node initialized successfully")
    arm.loop()


if __name__ == '__main__':
    main()
