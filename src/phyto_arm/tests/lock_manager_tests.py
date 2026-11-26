import unittest

from phyto_arm.lock_manager import NamedLockManager


class TestNamedLockManager(unittest.TestCase):
    def setUp(self):
        # Initialize NamedLockManager with a limit of 2 concurrent locks for testing
        self.lock_manager = NamedLockManager(max_concurrent_locks=2)

    def test_acquire_lock_success(self):
        # Test that a lock can be successfully acquired
        self.assertTrue(self.lock_manager.acquire_lock("arm1"))

    def test_acquire_lock_fail_due_to_limit(self):
        # Acquire locks up to the limit
        self.lock_manager.acquire_lock("arm1")
        self.lock_manager.acquire_lock("arm2")
        # Test that acquiring another lock fails due to the limit
        self.assertFalse(self.lock_manager.acquire_lock("arm3"))

    def test_release_lock_success(self):
        # Acquire a lock and then release it
        self.lock_manager.acquire_lock("arm1")
        self.assertTrue(self.lock_manager.release_lock("arm1"))

    def test_release_lock_fail_not_owned(self):
        # Test releasing a lock that was not acquired
        self.assertFalse(self.lock_manager.release_lock("arm1"))

    def test_check_lock_owned(self):
        # Acquire a lock and check it
        self.lock_manager.acquire_lock("arm1")
        self.assertTrue(self.lock_manager.check_lock("arm1"))

    def test_check_lock_not_owned(self):
        # Check a lock without acquiring it
        self.assertFalse(self.lock_manager.check_lock("arm1"))

    def test_reentrant_acquire_lock(self):
        # Test acquiring a lock twice by the same arm
        self.assertTrue(self.lock_manager.acquire_lock("arm1"))
        self.assertFalse(self.lock_manager.acquire_lock("arm1"))  # Should fail due to re-entrance

    def test_release_lock_not_affecting_others(self):
        # Test that releasing one lock does not affect others
        self.lock_manager.acquire_lock("arm1")
        self.lock_manager.acquire_lock("arm2")
        self.lock_manager.release_lock("arm1")
        self.assertTrue(self.lock_manager.check_lock("arm2"))
