#!/usr/bin/env python3
import time
import traceback

import rclpy

from dataclasses import dataclass
from threading import Event, Lock, Thread

from action_msgs.msg import GoalStatus
from geopy.distance import distance as vincenty_distance
from phyto_arm_msgs.action import MoveToDepth
from phyto_arm_msgs.srv import LockCheck, LockOperation
from rclpy.action import ActionClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String


@dataclass
class Task:
    '''A task to be executed by an arm.
    - name: a string identifying the task
    - callback: a function to call when the task is complete
    - depth: the depth to move to (optional, won't move if not provided)
    - speed: the speed to move at (optional, will use config max if not provided)
    '''
    name: str
    callback: callable
    depth: float = None
    speed: float = None


# This is the base class for all arms. It should be subclassed and the get_next_task
# method should be overridden to provide the arm's specific task logic.
# Do not use this class directly.
class ArmBase:

    def __init__(self, node: Node, winch_name=None):
        self.node = node
        self.arm_name = node.get_fully_qualified_name()
        self.stopping = Event()
        node.context.on_shutdown(self.stopping.set)
        self.task_lock = Lock()
        self.winch_client = None
        self._winch_busy = False

        # GPS related variables
        self._gps_lock = Lock()
        self._last_gps_fix = None
        self._last_gps_fix_time = None

        # Subscribe to GPS data for geofence check
        node.create_subscription(NavSatFix, '/gps/fix', self.gps_callback, qos_profile_sensor_data)
        self.node.get_logger().info(f"Subscribed to /gps/fix for {self.arm_name}")

        # Publisher for current task status
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.task_pub = node.create_publisher(String, 'task', latched)

        if node.get_parameter('winch_enabled').value:
            if winch_name is None:
                raise ValueError('Winch name must be provided if winch is enabled')

            # Setup service clients for acquiring permission to move winch. Prevents too many
            # winches from moving simultaneously
            acquire_move_clearance = node.create_client(LockOperation, '/lock_manager/acquire')
            release_move_clearance = node.create_client(LockOperation, '/lock_manager/release')
            lock_op = LockOperation.Request(arm_name=self.arm_name)

            # Bind lock operations to instance
            self.request_clearance = lambda: self.call_lock(acquire_move_clearance, lock_op).success
            self.release_clearance = lambda: self.call_lock(release_move_clearance, lock_op).success

            # Await check lock
            check_lock = node.create_client(LockCheck, '/lock_manager/check')
            self.node.get_logger().info('Awaiting lock manager check...')
            check_lock.wait_for_service()
            self.node.get_logger().info('Server acquired for lock check.')

            # Check whether we have a dangling acquisition to release
            if self.call_lock(check_lock, LockCheck.Request(arm_name=self.arm_name)).has_lock:
                self.node.get_logger().warning('Releasing dangling lock acquisition')
                assert self.release_clearance(), 'Failed to release dangling lock'

            self.winch_client = ActionClient(
                node, MoveToDepth, winch_name, callback_group=MutuallyExclusiveCallbackGroup()
            )
            self.node.get_logger().info(f'Awaiting winch server for {winch_name}')
            self.winch_client.wait_for_server()
            self.node.get_logger().info(f'Server acquired for {winch_name}')

        # Publish initial idle state
        self.publish_current_task("idle")


    def call_lock(self, client, request):
        future = client.call_async(request)
        deadline = time.monotonic() + 10
        while not future.done():
            if self.stopping.wait(0.01):
                raise RuntimeError('Shutdown during lock service call')
            if time.monotonic() >= deadline:
                raise TimeoutError(f'Lock service {client.srv_name} timed out')
        return future.result()


    def gps_callback(self, msg):
        with self._gps_lock:
            self._last_gps_fix = msg
            self._last_gps_fix_time = self.node.get_clock().now().nanoseconds / 1e9

    def geofence_block(self):
        geofence_enabled = self.node.get_parameter_or('geofence.enabled').value

        if not geofence_enabled:
            return False

        current_time = self.node.get_clock().now().nanoseconds / 1e9

        with self._gps_lock:
            last_fix = self._last_gps_fix
            last_fix_time = self._last_gps_fix_time

        # Check that we've had a fix in the last 5 minutes
        no_recent_fix = last_fix is None or (current_time - (last_fix_time or 0) > 300)
        if no_recent_fix:
            self.node.get_logger().warning(f'{self.arm_name}: GPS fix is missing or expired.', throttle_duration_sec=60.0)
            if self.node.get_parameter_or('geofence.strict').value:
                self.node.get_logger().info(f'{self.arm_name}: Strict GPS lock enabled.', throttle_duration_sec=60.0)
                return True # Pause due to strict mode and missing/expired GPS
            else:
                self.node.get_logger().info(f'{self.arm_name}: Strict GPS lock not enabled, proceeding with task.', throttle_duration_sec=60.0)
                return False # Don't pause if not strict

        # Valid GPS fix, check deny zones
        deny_zones = {}
        for key, parameter in self.node.get_parameters_by_prefix('geofence.deny').items():
            index, field = key.split('.', 1)
            deny_zones.setdefault(int(index), {})[field] = parameter.value
        current_point = (last_fix.latitude, last_fix.longitude)

        for index in sorted(deny_zones):
            zone = deny_zones[index]
            zone_point = (zone.get('latitude'), zone.get('longitude'))
            zone_radius = zone.get('radius')

            if None in zone_point or zone_radius is None:
                raise ValueError(f'{self.arm_name}: Invalid deny zone definition: {zone}')

            distance = vincenty_distance(current_point, zone_point).m

            if distance < zone_radius:
                deny_warn = f'{self.arm_name}: Inside denied geofence zone ({distance:.1f}m < {zone_radius}m).'
                self.node.get_logger().warning(deny_warn, throttle_duration_sec=60.0)
                return True # Pause due to being in a denied zone

        return False


    def publish_current_task(self, task_name):
        self.task_pub.publish(String(data=task_name))

    def winch_busy(self):
        return self._winch_busy


    def send_winch_goal(self, depth, speed, callback):
        self.node.get_logger().info(f'Sending winch move to {depth}')

        # Ensure winch is enabled
        if self.node.get_parameter('winch_enabled').value is not True:
            raise ValueError('Move aborted: winch is disabled in config')

        # Safety check: Do not exceed depth bounds
        min_depth = self.node.get_parameter('winch.range.min').value
        max_depth = self.node.get_parameter('winch.range.max').value
        if depth < min_depth:
            raise ValueError(f'Move aborted: depth {depth} is below min {min_depth}')
        elif depth > max_depth:
            raise ValueError(f'Move aborted: depth {depth} is above max {max_depth}')

        # Set max speed if not specified
        max_speed = self.node.get_parameter('winch.max_speed').value
        if speed is None:
            speed = max_speed

        # Safety check: Speed cannot exceed max speed
        elif speed > max_speed:
            raise ValueError(f'Move aborted: speed {speed} is above max {max_speed}')
        if self.winch_busy():
            raise RuntimeError('Move aborted: winch in unexpected busy state')
        self.node.get_logger().info(f"All checks passed; Moving winch to {depth}")

        # Intermediate callback for ensuring move was a success and to release lock
        # before calling the task's callback
        def winch_done(future):
            try:
                # Ensure winch move was successful
                handle = future.result()
                assert handle.accepted
                response = handle.get_result()
                self._winch_busy = False
                assert response.status == GoalStatus.STATUS_SUCCEEDED

                # Free up semaphore for another winch movement
                assert self.release_clearance()
                callback(response.result)
            except Exception:
                self._winch_busy = False
                self.node.get_logger().error(f'Unexpected error during winch movement: {traceback.format_exc()}')
                rclpy.try_shutdown(context=self.node.context)
                raise

        self._winch_busy = True
        goal = MoveToDepth.Goal(depth=float(depth), velocity=float(speed))
        future = self.winch_client.send_goal_async(goal)
        # A completed move can wait for another action, service or profile.
        future.add_done_callback(lambda f: Thread(target=winch_done, args=(f,), daemon=True).start())
        self.publish_current_task("winch_movement")


    # Logic for determining arm tasks goes heres, all implementations should override this
    def get_next_task(self, last_task):
        raise NotImplementedError('Subclasses must implement this method')


    # Callback for when a task is complete. Unused result argument is required
    # for cases where this method is used as a task callback
    def start_next_task(self, move_result=None):
        self.publish_current_task("task_completed")
        self.task_lock.release()


    # Primary loop that should be the same for all arms, override should not be needed
    def loop(self):
        task = None
        while rclpy.ok(context=self.node.context):
            # Don't block so that we can keeping checking for shutdowns
            if self.task_lock.acquire(blocking=False):
                # If no movement is required
                if self.winch_client is None:
                    task = self.get_next_task(task)
                    self.node.get_logger().warning(f'No winch; running {task.name}')
                    self.publish_current_task(task.name)
                    task.callback()

                # Otherwise, move winch
                else:
                    self.node.get_logger().warning('Arm waiting for clearance')
                    self.publish_current_task("waiting_for_clearance")

                    # TODO: Consider replacing this with a queueing mechanism.  Requires setting up
                    # callbacks via ROS service calls. Unnecessarily complex for a 2 winch system,
                    # but might be the only way to achieve round-robin for many winches.

                    while rclpy.ok(context=self.node.context) and not self.request_clearance():
                        # Wait until central semaphore clears us to move
                        if self.stopping.wait(1):
                            return

                    # TODO: Optimize task evaluation. Currently we are blocking on the assumption that
                    # movement will be needed; this is not always the case. Not a problem for 1 or 2
                    # winches, but could get slow if the number of winches greatly exceeds the
                    # max_moving_winches limit.

                    task = self.get_next_task(task)
                    self.node.get_logger().warning(f'Goal depth {task.depth} for task {task.name}')
                    self.publish_current_task(task.name)
                    self.send_winch_goal(task.depth, task.speed, task.callback)
