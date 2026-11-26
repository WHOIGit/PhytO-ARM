#!/usr/bin/env python3

import rclpy

from threading import Lock, Semaphore

from phyto_arm_msgs.srv import LockCheck, LockOperation
from rclpy.executors import ExternalShutdownException
from rclpy.parameter import Parameter


class NamedLockManager:
    def __init__(self, max_concurrent_locks):
        self.max_concurrent_locks = max_concurrent_locks
        self.semaphore = Semaphore(max_concurrent_locks)
        self.lock = Lock()
        self.lock_owners = set()

    def acquire_lock(self, arm_name):
        with self.lock:
            if arm_name in self.lock_owners:
                return False  # Lock already acquired by this arm
            else:
                acquired = self.semaphore.acquire(blocking=False)
                if acquired:
                    self.lock_owners.add(arm_name)
                return acquired

    def release_lock(self, arm_name):
        with self.lock:
            if arm_name in self.lock_owners:
                self.lock_owners.remove(arm_name)
                self.semaphore.release()
                return True
            else:
                return False  # Lock not owned by this arm

    def check_lock(self, arm_name):
        with self.lock:
            return arm_name in self.lock_owners


if __name__ == "__main__":
    rclpy.init()
    node = rclpy.create_node('lock_manager')

    def handle_acquire(req, response):
        response.success = lock_manager.acquire_lock(req.arm_name)
        return response

    def handle_release(req, response):
        response.success = lock_manager.release_lock(req.arm_name)
        return response

    def handle_check(req, response):
        response.has_lock = lock_manager.check_lock(req.arm_name)
        return response

    try:
        node.declare_parameter('max_moving_winches', Parameter.Type.INTEGER)
        lock_manager = NamedLockManager(node.get_parameter('max_moving_winches').value)
        node.create_service(LockOperation, '~/acquire', handle_acquire)
        node.create_service(LockOperation, '~/release', handle_release)
        node.create_service(LockCheck, '~/check', handle_check)
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
