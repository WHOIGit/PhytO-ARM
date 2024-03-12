#!/usr/bin/env python3

from threading import Lock, Semaphore

import rospy
from phyto_arm.srv import LockOperation, LockCheck, LockOperationResponse, LockCheckResponse

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
    rospy.init_node('lock_manager')
    lock_manager = NamedLockManager(rospy.get_param('~max_moving_winches'))

    def handle_acquire(req):
        success = lock_manager.acquire_lock(req.arm_name)
        return LockOperationResponse(success=success)

    def handle_release(req):
        success = lock_manager.release_lock(req.arm_name)
        return LockOperationResponse(success=success)

    def handle_check(req):
        has_lock = lock_manager.check_lock(req.arm_name)
        return LockCheckResponse(has_lock=has_lock)

    rospy.Service('~acquire', LockOperation, handle_acquire)
    rospy.Service('~release', LockOperation, handle_release)
    rospy.Service('~check', LockCheck, handle_check)

    rospy.spin()
