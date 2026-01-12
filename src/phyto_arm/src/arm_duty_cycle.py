#!/usr/bin/env python3
"""Duty cycle arm implementation for PhytO-ARM.

This module implements a duty cycle arm that turns the IFCB on and off based on
a configurable schedule while performing profiling and sampling tasks.
"""
import subprocess

from datetime import datetime, timedelta
from threading import Event

import actionlib
import rospy

from std_msgs.msg import Bool
from ds_sensor_msgs.msg import DepthPressure

from arm_base import ArmBase, Task
from phyto_arm.msg import DepthProfile, RunIFCBGoal, RunIFCBAction
import ifcb.srv


class ArmDutyCycle(ArmBase):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

        # Profile tracking
        self.profile_activity = Event()
        self.latest_profile = None

        # Profiler peak tracking
        self.profiler_peak_depth = None
        self.profiler_peak_value = None

        # Maintenance tracking
        self.last_cart_debub_time = None
        self.last_bead_time = None

        # IFCB connection tracking
        self.ifcb_connected = False
        self.ifcb_connection_event = Event()

        # CTD depth tracking
        self.ctd_depth_received = Event()
        self.latest_ctd_depth = None

        # Duty cycle state tracking
        self.is_sampling_active = False
        self.last_duty_check_time = None
        self.next_duty_transition_time = None

        # Sample session tracking
        self.samples_in_current_session = 0
        self.last_session_start_time = None
        self.target_samples_per_session = 0
        self.reference_start_time = None  # Fixed reference time for interval calculations

    def get_next_task(self, last_task):
        # Check duty cycle schedule
        if self.should_transition_duty_state():
            if self.is_sampling_active:
                # Time to shut down
                return Task('shutdown_sampling', shutdown_sampling)
            else:
                # Time to start up
                return Task('startup_sampling', startup_sampling)
        elif last_task is None:
            send_ifcb_auxpower_on()

        # If we're not supposed to be sampling, just wait
        if not self.is_sampling_active:
            return Task('await_duty_cycle', await_duty_cycle)

        if not rospy.get_param('winch_enabled'):
            return Task('no_winch', handle_nowinch)

        # Start off at min depth for upcast
        preupcast_tasks = ['profiler_peak_depth', 'default_depth', 'await_ifcb_connection',
                          'startup_sampling', 'await_duty_cycle']
        if last_task is None or last_task.name in preupcast_tasks:
            return Task('upcast', self.start_next_task, rospy.get_param('winch/range/min'))

        # Then perform a downcast to get a full profile
        if last_task.name == 'upcast':
            return Task('downcast', handle_downcast, rospy.get_param('winch/range/max'))

        # Then go to peak depth if found and above threshold, otherwise use default depth
        if last_task.name == 'downcast':
            threshold = rospy.get_param('tasks/profiler_peak/threshold', 0.0)
            if self.profiler_peak_depth is not None and self.profiler_peak_value >= threshold:
                rospy.loginfo(f'Using profiler peak depth: {self.profiler_peak_depth:.2f} m')
                return Task('profiler_peak_depth', handle_target_depth, self.profiler_peak_depth)

            # Peak not found or below threshold, use default depth
            default_depth = rospy.get_param('tasks/default_depth', 1.0)
            rospy.loginfo(f'Using default depth: {default_depth:.2f} m')
            return Task('default_depth', handle_target_depth, default_depth)

        raise ValueError(f'Unhandled next-task state where last task={last_task.name}')

    def should_transition_duty_state(self):
        """Check if it's time to transition between sampling and non-sampling states"""
        current_time = rospy.Time.now()

        # Initialize duty cycle tracking if needed
        if self.last_duty_check_time is None:
            self.last_duty_check_time = current_time
            self.update_duty_cycle_state()
            return False

        if self.is_sampling_active:
            # Check if we've reached the target sample count
            if self.samples_in_current_session >= self.target_samples_per_session:
                rospy.loginfo(f'Reached target sample count ({self.target_samples_per_session}), shutting down')
                return True
        else:
            # Check if it's time to start a new sampling session
            if self.next_duty_transition_time and current_time >= self.next_duty_transition_time:
                rospy.loginfo('Interval elapsed, starting new sampling session')
                return True

        return False

    def update_duty_cycle_state(self):
        """Update the duty cycle state based on interval configuration"""
        # Load configuration
        interval_minutes = rospy.get_param('tasks/duty_cycle/interval', 0)
        self.target_samples_per_session = rospy.get_param('tasks/duty_cycle/number_of_samples', 0)

        if interval_minutes == 0 or self.target_samples_per_session == 0:
            # No interval configured, default to always sampling
            rospy.logwarn('Duty cycle interval or number_of_samples not configured, defaulting to continuous sampling')
            self.is_sampling_active = True
            self.next_duty_transition_time = None
            return

        # Initialize reference start time if needed
        if self.reference_start_time is None:
            start_time_str = rospy.get_param('tasks/duty_cycle/start_time', None)
            if start_time_str:
                try:
                    # Parse UTC time string
                    reference_datetime = datetime.strptime(start_time_str, '%Y-%m-%d %H:%M')
                    self.reference_start_time = rospy.Time.from_sec(reference_datetime.timestamp())
                    rospy.loginfo(f'Using configured reference start time: {start_time_str} UTC')
                except ValueError:
                    rospy.logwarn(f'Invalid start_time format: {start_time_str}, using current time')
                    self.reference_start_time = rospy.Time.now()
            else:
                # No start time configured, use current time
                self.reference_start_time = rospy.Time.now()
                rospy.loginfo('No start_time configured, using current time as reference')

        # Calculate next session start time based on fixed intervals from reference time
        current_time = rospy.Time.now()

        # Calculate how many intervals have elapsed since reference time
        elapsed = (current_time - self.reference_start_time).to_sec()
        if elapsed < 0:
            # Reference time is in the future, use it as the next session
            intervals_elapsed = 0
        else:
            # Round up to get the next interval slot
            intervals_elapsed = int(elapsed / (interval_minutes * 60)) + 1

        # Calculate next session time
        self.next_duty_transition_time = self.reference_start_time + rospy.Duration(intervals_elapsed * interval_minutes * 60)

        rospy.loginfo(f'Duty cycle state: sampling_active={self.is_sampling_active}, '
                     f'interval={interval_minutes} min, '
                     f'target_samples={self.target_samples_per_session}, '
                     f'next_transition={self.next_duty_transition_time}')


# Global reference to action provided by other node
ifcb_runner = None

# Global reference to arm state
arm = None

# DL outlet publishers
dl_ifcb_pub = None

# IFCB command service client
ifcb_command_client = None


def on_profile_msg(msg):
    """Handle incoming profile messages.

    Args:
        msg: The DepthProfile message
    """
    # Alert waiting threads to the new profile message
    arm.latest_profile = msg
    arm.profile_activity.set()
    arm.profile_activity.clear()


def on_ifcb_connection_status(msg):
    """Handle IFCB connection status updates.

    Args:
        msg: Bool message indicating connection status
    """
    arm.ifcb_connected = msg.data
    if arm.ifcb_connected:
        rospy.loginfo('IFCB connection established')
        arm.ifcb_connection_event.set()
        arm.ifcb_connection_event.clear()
    else:
        rospy.logwarn('IFCB connection lost')


def on_ctd_depth(msg):
    arm.latest_ctd_depth = msg.depth
    arm.ctd_depth_received.set()
    arm.ctd_depth_received.clear()


def await_ifcb_connection():
    """Wait for IFCB connection to be established."""
    while not arm.ifcb_connected and not rospy.is_shutdown():
        rospy.loginfo('Waiting for IFCB connection...')
        arm.ifcb_connection_event.wait(30)

    if arm.ifcb_connected:
        rospy.loginfo('IFCB connection restored, resuming tasks')
        arm.start_next_task()


def await_duty_cycle():
    """Wait for the next duty cycle transition"""
    rospy.loginfo('Outside sampling window, waiting for next duty cycle')

    # Check duty cycle every minute
    check_interval = rospy.get_param('tasks/duty_cycle/check_interval', 60)

    while not arm.should_transition_duty_state() and not rospy.is_shutdown():
        rospy.sleep(check_interval)

    rospy.loginfo('Duty cycle transition time reached')
    arm.start_next_task()


def startup_sampling():
    """Start IFCB sampling by turning on the digital logger outlet"""
    rospy.loginfo('Starting IFCB sampling')
    power_on_ifcb()

    # Turn on IFCB auxiliary power
    send_ifcb_auxpower_on()

    # Wait to get a depth message
    rospy.loginfo('Waiting for CTD depth message...')
    timeout = 60  # Wait up to 60 seconds for depth message
    if arm.ctd_depth_received.wait(timeout=timeout):
        rospy.loginfo(f'CTD depth received: {arm.latest_ctd_depth:.2f} m')
    else:
        rospy.logerr('No CTD depth message received within timeout')
        raise RuntimeError('No depth messages received after startup wait period')

    # Reset session tracking
    arm.samples_in_current_session = 0
    arm.last_session_start_time = rospy.Time.now()

    # Update duty cycle state
    arm.is_sampling_active = True
    arm.update_duty_cycle_state()

    rospy.loginfo(f'Starting new sampling session: target {arm.target_samples_per_session} samples')
    arm.start_next_task()


def shutdown_sampling():
    """Stop IFCB sampling by turning off the digital logger outlet"""
    rospy.loginfo('Stopping IFCB sampling')

    # Wait for current sample to finish
    shutdown_duration = rospy.get_param('tasks/duty_cycle/shutdown_wait_duration')
    rospy.logwarn(f'Waiting for IFCB to finish current sample for {shutdown_duration} seconds')
    rospy.sleep(shutdown_duration)

    # Sync files from IFCB to Pi before shutdown
    sync_ifcb_files()

    power_off_ifcb()

    # Update duty cycle state
    arm.is_sampling_active = False
    arm.update_duty_cycle_state()

    arm.start_next_task()


def power_on_ifcb():
    # If already connected, proceed immediately
    if arm.ifcb_connected:
        rospy.loginfo('IFCB already connected, proceeding')
        return

    rospy.loginfo('Turning on IFCB power')
    dl_ifcb_pub.publish(Bool(data=True))

    # Wait for IFCB to connect
    restart_duration = rospy.get_param('tasks/duty_cycle/restart_wait_duration', 120)
    rospy.loginfo(f'Waiting for IFCB to connect (timeout: {restart_duration} seconds)')


    # Wait for connection with timeout
    if arm.ifcb_connection_event.wait(timeout=restart_duration):
        rospy.loginfo('IFCB connected successfully')
    else:
        rospy.logerr('IFCB failed to connect within timeout')


def power_off_ifcb():
    send_ifcb_host_shutdown()
    arm.ifcb_connected = False
    rospy.loginfo('Waiting 60 seconds for IFCB to shutdown')
    rospy.sleep(60) # Wait 1 minute for IFCB to shutdown completely

    # Switch off power
    rospy.loginfo('Turning off IFCB power')
    dl_ifcb_pub.publish(Bool(data=False))


def send_ifcb_auxpower_on():
    """Send command to turn on IFCB auxiliary power port 1"""
    rospy.loginfo('Sending IFCB auxiliary power ON command')
    req = ifcb.srv.CommandRequest()
    req.command = 'daq:switchauxpower1:1'
    response = ifcb_command_client(req)
    if response.success:
        rospy.loginfo('IFCB auxiliary power ON command sent successfully')
    else:
        rospy.logwarn('IFCB auxiliary power ON command failed')
    return response.success


def send_ifcb_host_shutdown():
    rospy.loginfo('Sending shutdown command to IFCB host via SSH')

    # Get IFCB host address from ROS parameters
    ifcb_host = rospy.get_param('/ifcb/address')

    # Execute shutdown command via SSH
    # Assumes SSH key is already set up for passwordless authentication
    result = subprocess.run([
        'ssh', '-o', 'ConnectTimeout=10',
        '-o', 'StrictHostKeyChecking=no',
        f'ifcb@{ifcb_host}',
        'sudo shutdown now'
    ], capture_output=True, text=True, timeout=15)

    # Accept return codes 0 (clean exit) and 255 (SSH connection dropped due to shutdown)
    # When the remote host shuts down immediately, SSH returns 255 as the connection is lost
    if result.returncode in [0, 255]:
        rospy.loginfo('IFCB host shutdown command sent successfully')
        return True
    else:
        error_msg = f"Unable to send shutdown command to IFCB (code {result.returncode})"
        if result.stderr:
            error_msg += f"\nSTDERR: {result.stderr}"
        if result.stdout:
            error_msg += f"\nSTDOUT: {result.stdout}"
        error_msg += "\nDid you remember to add passwordless shutdown to the IFCB via visudo?"
        rospy.logerr(error_msg)
        raise SystemError(error_msg)


def sync_ifcb_files():
    """Sync files from IFCB host to Pi using rsync.

    This function is called before IFCB shutdown to transfer data files
    from the IFCB host to the Raspberry Pi for backup/storage.
    """
    # Check if file sync is enabled
    sync_enabled = rospy.get_param('tasks/duty_cycle/file_sync/enabled', False)
    if not sync_enabled:
        rospy.loginfo('File sync disabled, skipping')
        return True

    # Get sync configuration
    ifcb_host = rospy.get_param('/ifcb/address')
    source_path = rospy.get_param('tasks/duty_cycle/file_sync/source_path', '/data/ifcbdata')
    destination_path = rospy.get_param('tasks/duty_cycle/file_sync/destination_path', '/mnt/data/ifcb_sync')
    rsync_timeout = rospy.get_param('tasks/duty_cycle/file_sync/rsync_timeout', 600)
    rsync_options = rospy.get_param('tasks/duty_cycle/file_sync/rsync_options', '-avz')

    rospy.loginfo(f'Starting file sync from IFCB: {source_path} -> {destination_path}')

    # Build rsync command
    # Format: rsync <options> ifcb@<host>:<source>/ <destination>/
    # Note: trailing slashes are important for rsync behavior
    rsync_cmd = [
        'rsync',
        *rsync_options.split(),  # Split options string into list
        '--timeout=60',  # Per-file timeout
        f'ifcb@{ifcb_host}:{source_path}/',
        f'{destination_path}/'
    ]

    try:
        rospy.loginfo(f'Running: {" ".join(rsync_cmd)}')
        result = subprocess.run(
            rsync_cmd,
            capture_output=True,
            text=True,
            timeout=rsync_timeout
        )

        if result.returncode == 0:
            rospy.loginfo('File sync completed successfully')
            if result.stdout:
                rospy.logdebug(f'Rsync output: {result.stdout}')
            return True
        else:
            rospy.logerr(f'File sync failed with return code {result.returncode}')
            if result.stderr:
                rospy.logerr(f'Rsync error: {result.stderr}')
            return False

    except subprocess.TimeoutExpired:
        rospy.logerr(f'File sync timed out after {rsync_timeout} seconds')
        return False
    except Exception as e:
        rospy.logerr(f'File sync failed with exception: {e}')
        return False


def send_ifcb_action():
    """Send an IFCB sampling action and wait for completion."""
    goal = RunIFCBGoal()
    ifcb_runner.send_goal(goal)
    ifcb_runner.wait_for_result()

    # Increment sample counter for duty cycle tracking
    arm.samples_in_current_session += 1
    rospy.loginfo(f'Completed sample {arm.samples_in_current_session}/{arm.target_samples_per_session}')


def handle_downcast(move_result):
    """Handle completion of a downcast movement and process profile data.

    Args:
        move_result: The result from the movement action
    """
    # Wait for the profile data for this cast
    while arm.latest_profile is None or \
        arm.latest_profile.goal_uuid != move_result.uuid:
        notified = arm.profile_activity.wait(60)

        # For short downcasts profiling will likely fail; switch to scheduled depth
        if not notified:
            rospy.logwarn('No profile data received')
            arm.profiler_peak_depth = None
            arm.profiler_peak_value = None
            arm.start_next_task()
            return

    # Find the maximal value in the profile
    argmax = max(range(len(arm.latest_profile.values)),
                 key=lambda i: arm.latest_profile.values[i])
    arm.profiler_peak_value = arm.latest_profile.values[argmax]
    arm.profiler_peak_depth = arm.latest_profile.depths[argmax]

    rospy.loginfo(f'Profile peak is {arm.profiler_peak_value:.2f} at '
                 f'{arm.profiler_peak_depth:.2f} m')
    arm.start_next_task()


def handle_target_depth(move_result):
    """Handle arrival at target depth and trigger IFCB sampling.

    Args:
        move_result: The result from the movement action
    """
    send_ifcb_action()
    arm.start_next_task()


def handle_nowinch():
    """Handle IFCB sampling when winch is disabled."""
    send_ifcb_action()
    arm.start_next_task()


def main():
    global arm
    global ifcb_runner
    global dl_ifcb_pub
    global ifcb_command_client

    rospy.init_node('arm_duty_cycle', log_level=rospy.DEBUG)

    winch_name = None
    if rospy.get_param('winch_enabled') is True:
        winch_name = rospy.get_namespace() + 'winch/move_to_depth'
    arm = ArmDutyCycle(rospy.get_name(), winch_name)

    # Set a fake timestamp for having run beads and cartridge debubble, so that
    # we don't run it every startup and potentially waste time or bead supply.
    arm.last_cart_debub_time = rospy.Time.now()
    arm.last_bead_time = rospy.Time.now()

    # Subscribe to profiler messages that follow each transit
    rospy.Subscriber('profiler', DepthProfile, on_profile_msg)

    # Subscribe to IFCB connection status
    rospy.Subscriber('/ifcb/connected', Bool, on_ifcb_connection_status)

    # Subscribe to CTD depth topic
    ctd_topic = rospy.get_param('ctd_topic')
    rospy.loginfo(f'Subscribing to CTD depth topic: {ctd_topic}')
    rospy.Subscriber(ctd_topic, DepthPressure, on_ctd_depth)

    # Setup publisher for DL outlet control
    dl_ifcb_pub = rospy.Publisher('/digital_logger/outlet/ifcb/control', Bool, queue_size=10)

    # Setup action client for running IFCB
    ifcb_runner = actionlib.SimpleActionClient('ifcb_runner/sample', RunIFCBAction)
    rospy.loginfo(f'Arm {rospy.get_name()} awaiting IFCB-run action server')
    ifcb_runner.wait_for_server()
    rospy.loginfo(f'Arm {rospy.get_name()} IFCB-run action server acquired')

    # Setup service client for IFCB commands
    rospy.loginfo('Waiting for IFCB command service...')
    ifcb_command_client = rospy.ServiceProxy('/ifcb/command', ifcb.srv.Command)
    rospy.loginfo('IFCB command service client acquired')

    # Initialize duty cycle state
    arm.update_duty_cycle_state()

    arm.loop()

if __name__ == '__main__':
    main()
