import asyncio
import os
import sys
import threading
import unittest
from types import SimpleNamespace
from unittest.mock import Mock, call, patch

import rospy

sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'src'))

import winch_node as winch


class TestDepthMove(unittest.TestCase):
    def move(self, target=2.0, position=0):
        return winch.DepthMove(target, 0.01, 1.0, position, epsilon=0.01,
                         gear_ratio=1, half_speed_dist=0.05,
                         margin=0.125, spool_circumference=1.0)

    def test_upcast_and_downcast_bounds(self):
        for target, limits in ((0.0, (-9216, 1024)), (2.0, (-1024, 9216))):
            with self.subTest(target=target):
                move = self.move(target)
                self.assertEqual((move.lower, move.upper), limits)
                self.assertEqual(move.next_envelope(), (-1024, 1024))

    def test_envelope_needs_depth_and_encoder(self):
        move = self.move()
        move.next_envelope()
        move.observe_motor(winch.Motion.MODE_VELOCITY, 100)
        self.assertIsNone(move.next_envelope())
        move.observe_depth(1.0)
        self.assertEqual(move.next_envelope(), (-924, 1124))
        self.assertIsNone(move.next_envelope())

    def test_depth_excursion_survives_next_reading(self):
        move = self.move()
        move.observe_depth(3.0)
        move.observe_depth(1.5)
        self.assertIn('Exceeded depth bounds', move.error)

    def test_target_sample_survives_next_reading(self):
        move = self.move()
        move.observe_depth(2.0)
        move.observe_depth(2.02)
        self.assertTrue(move.reached)
        self.assertIsNone(move.error)

    def test_already_at_target(self):
        move = self.move(target=1.0)
        self.assertTrue(move.reached)
        self.assertTrue(move.expected_time.is_zero())

    def test_encoder_wrap(self):
        with self.assertRaisesRegex(ValueError, 'Encoder position could wrap'):
            self.move(position=2**24 - 1)


class TestWinchTime(unittest.IsolatedAsyncioTestCase):
    def setup_clock(self):
        loop = asyncio.get_running_loop()
        with patch.object(rospy, 'get_param', return_value=True), \
             patch.object(rospy.Time, 'now', return_value=rospy.Time(10)), \
             patch.object(rospy, 'Subscriber'):
            self.clock = winch.AwaitableRosClock(loop)
        self.depth = winch.WinchStatus(self.clock)
        self.motor = winch.WinchStatus(self.clock)

    async def test_clock_paused_during_subscription(self):
        loop = asyncio.get_running_loop()
        stamp = rospy.Time(10)

        # Gazebo advances and pauses just before our callback is registered
        def subscribe(topic, message_type, callback):
            nonlocal stamp
            stamp = rospy.Time(100)
            return Mock()

        with patch.object(rospy, 'get_param', return_value=True), \
             patch.object(rospy, 'Subscriber', side_effect=subscribe), \
             patch.object(rospy.Time, 'now', side_effect=lambda: stamp):
            clock = winch.AwaitableRosClock(loop)
            self.assertEqual(clock.now(), rospy.Time(100))

    async def test_wait_returns_latest_value(self):
        self.setup_clock()
        depth = self.depth
        cancelled = asyncio.create_task(depth.wait())
        other = asyncio.create_task(depth.wait())
        await asyncio.sleep(0)
        cancelled.cancel()

        # Two updates can arrive before either waiter resumes
        depth.update_soon(2)
        depth.update_soon(3)
        self.assertEqual(await depth, 3)
        self.assertEqual(await other, 3)
        self.assertEqual(depth.value, 3)

    async def test_timeout_requires_duration(self):
        self.setup_clock()
        with self.assertRaises(TypeError):
            await winch.wait_for_updates(
                self.clock, [self.depth], rospy.Time(12))

    async def test_overlapping_waits(self):
        self.setup_clock()
        third = winch.AwaitableValue(
            asyncio.get_running_loop(), now=self.clock.now)
        first = asyncio.create_task(winch.wait_for_updates(
            self.clock, [self.depth, self.motor, third], rospy.Duration(2)))
        second = asyncio.create_task(winch.wait_for_updates(
            self.clock, [self.depth], rospy.Duration(1)))
        await asyncio.sleep(0)

        # Both waiters see the same depth update at the earlier deadline
        self.clock.tick(rospy.Time(11))
        self.depth.update(1.0)
        await asyncio.wait_for(second, timeout=1)
        self.motor.update(1.0)
        self.clock.tick(rospy.Time(13))
        await asyncio.wait([first], timeout=1)
        with self.assertRaises(asyncio.TimeoutError):
            first.result()

    async def test_silent_stream_expires_at_its_own_deadline(self):
        self.setup_clock()
        self.depth.update(1)
        self.motor.update(1)
        readings = SimpleNamespace(clock=self.clock, depth=self.depth,
                                   motor=self.motor)
        watchdog = winch.MoveWatchdog(readings, rospy.Duration(20))
        self.clock.tick(rospy.Time(11))
        self.depth.update(2)
        waiting = asyncio.create_task(watchdog.wait(watchdog.next_updates()))
        await asyncio.sleep(0)
        self.clock.tick(rospy.Time(12))
        with self.assertRaisesRegex(asyncio.TimeoutError, 'motor status'):
            await asyncio.wait_for(waiting, timeout=1)


class TestWinchAction(unittest.IsolatedAsyncioTestCase):
    def patch(self, patcher):
        self.addCleanup(patcher.stop)
        return patcher.start()

    def setup_move(self, simulated=True, start=10):
        loop = asyncio.get_running_loop()
        self.stamp = rospy.Time(start)
        self.params = {
            '/use_sim_time': simulated,
            '~epsilon': 0.01,
            '~gear_ratio': 60,
            '~half_speed_dist': 0.05,
            '~max_speed': 0.02,
            '~safety_envelopes': {'position': 0.1},
            '~spool_circumference': 0.6604,
        }
        self.patch(patch.object(rospy, 'get_param',
                                side_effect=self.params.get))
        now = self.patch(patch.object(rospy.Time, 'now'))
        now.side_effect = lambda: self.stamp
        self.patch(patch.object(rospy, 'Subscriber'))
        self.clock = winch.AwaitableRosClock(loop)
        self.readings = winch.WinchReadings(self.clock)
        self.depth = self.readings.depth
        self.motor = self.readings.motor
        self.commanded = asyncio.Event()
        self.velocity = Mock(side_effect=lambda **kw:
                             loop.call_soon_threadsafe(self.commanded.set))
        self.patch(patch.multiple(
            winch, readings=self.readings,
            set_position_envelope=Mock(), set_velocity=self.velocity,
            stop=Mock()
        ))
        self.server = Mock()
        self.server.is_preempt_requested.return_value = False
        self.server.is_active.return_value = False

    async def start_goal(self, depth=1.0):
        goal = Mock(depth=depth, velocity=0.01)
        task = asyncio.create_task(winch.move_to_depth_chk(self.server, goal))
        await asyncio.sleep(0)
        return task

    def cancel_goal(self):
        self.server.is_preempt_requested.return_value = True
        self.server.register_preempt_callback.call_args.args[0]()

    def send_readings(self):
        loop = asyncio.get_running_loop()
        loop.call_soon(self.readings.update_depth, Mock(depth=0.7))
        loop.call_soon(self.readings.update_motor,
                       Mock(mode=winch.Motion.MODE_PASSIVE, position=0))

    async def start_moving(self):
        task = await self.start_goal()
        first_feedback = asyncio.Event()
        self.server.publish_feedback.side_effect = (
            lambda msg: first_feedback.set())
        self.send_readings()
        await asyncio.wait_for(first_feedback.wait(), timeout=1)
        return task

    async def test_cancel_during_startup(self):
        self.setup_move()
        task = await self.start_goal()
        self.cancel_goal()
        await asyncio.wait_for(task, timeout=1)
        self.server.set_preempted.assert_called_once()
        self.velocity.assert_not_called()

    async def test_first_goal_with_queued_ticks(self):
        self.setup_move()
        self.stamp = rospy.Time(12)
        task = await self.start_goal()

        # The rospy cache is ahead of the callbacks queued for the winch
        self.clock.tick(rospy.Time(11))
        self.send_readings()
        asyncio.get_running_loop().call_soon(self.clock.tick, rospy.Time(12))
        await asyncio.wait_for(self.commanded.wait(), timeout=1)
        self.cancel_goal()
        await asyncio.wait_for(task, timeout=1)
        self.server.set_aborted.assert_not_called()

    async def test_depth_and_motor_updates(self):
        self.setup_move()
        task = await self.start_moving()
        envelope = winch.set_position_envelope
        feedback = asyncio.Event()
        self.server.publish_feedback.side_effect = lambda msg: feedback.set()

        # Encoder reports alone cannot advance the envelope or change speed
        self.clock.tick(rospy.Time(10, 100000000))
        self.readings.update_motor(
            Mock(mode=winch.Motion.MODE_VELOCITY, position=10))
        await asyncio.wait_for(feedback.wait(), timeout=1)
        self.velocity.assert_called_once()
        envelope.assert_called_once()

        # An unchanged depth confirms the encoder position without a speed write
        feedback.clear()
        self.readings.update_depth(Mock(depth=0.7))
        await asyncio.wait_for(feedback.wait(), timeout=1)
        self.velocity.assert_called_once()
        self.assertEqual(envelope.call_count, 2)

        # Depth alone changes speed and can finish the move before the next tick
        feedback.clear()
        self.readings.update_depth(Mock(depth=0.9))
        await asyncio.wait_for(feedback.wait(), timeout=1)
        first, second = self.velocity.call_args_list
        self.assertLess(second.kwargs['velocity'], first.kwargs['velocity'])
        self.assertEqual(envelope.call_count, 2)
        self.readings.update_depth(Mock(depth=1.0))
        await asyncio.wait_for(task, timeout=1)
        self.server.set_succeeded.assert_called_once()
        winch.stop.assert_called_with()

    async def test_unexpected_motor_stop(self):
        self.setup_move()
        task = await self.start_moving()

        # All three samples arrive before the move coroutine can run
        for mode in (winch.Motion.MODE_VELOCITY, winch.Motion.MODE_PASSIVE):
            self.readings.update_motor(Mock(mode=mode, position=1))
        self.readings.update_depth(Mock(depth=0.8))
        await asyncio.wait_for(task, timeout=1)
        self.server.set_aborted.assert_called_once_with(
            text='Motor unexpectedly stopped, mode: 0')
        self.velocity.assert_called_once()

    async def test_stale_status(self):
        for missing in ('depth', 'motor'):
            with self.subTest(missing=missing):
                self.setup_move()
                task = await self.start_moving()
                self.clock.tick(rospy.Time(11))
                if missing == 'depth':
                    self.readings.update_motor(
                        Mock(mode=winch.Motion.MODE_VELOCITY, position=1))
                else:
                    self.readings.update_depth(Mock(depth=0.7))
                self.clock.tick(rospy.Time(12))
                await asyncio.wait_for(task, timeout=1)
                self.server.set_aborted.assert_called_once_with(
                    text=f'Timed out waiting for {missing} status')
                self.velocity.assert_called_once()
                winch.set_position_envelope.assert_called_with(min=0, max=0)

    async def test_late_status_cannot_restart_motion(self):
        self.setup_move()
        task = await self.start_moving()

        # Both streams recover before the waiting coroutine resumes
        for second in (11, 14):
            self.clock.tick(rospy.Time(second))
            self.readings.update_depth(Mock(depth=0.9))
            self.readings.update_motor(
                Mock(mode=winch.Motion.MODE_VELOCITY, position=1))
        await asyncio.wait_for(task, timeout=1)
        self.server.set_aborted.assert_called_once_with(
            text='Timed out waiting for depth status')
        self.velocity.assert_called_once()

    async def test_move_deadline_with_fresh_status(self):
        self.setup_move()
        self.patch(patch.object(winch, 'estimate_time',
                                return_value=rospy.Duration(0)))
        task = await self.start_moving()
        feedback = asyncio.Event()
        self.server.publish_feedback.side_effect = lambda msg: feedback.set()
        for second in range(11, 20):
            feedback.clear()
            self.clock.tick(rospy.Time(second))
            self.send_readings()
            await asyncio.wait_for(feedback.wait(), timeout=1)
        self.clock.tick(rospy.Time(20))
        await asyncio.wait_for(task, timeout=1)
        self.server.set_aborted.assert_called_once()
        self.assertIn('Time limit exceeded',
                      self.server.set_aborted.call_args.kwargs['text'])

    async def test_paused_cancel(self):
        self.setup_move()
        task = await self.start_moving()
        winch.stop.reset_mock()
        self.cancel_goal()
        await asyncio.wait_for(task, timeout=1)
        self.server.set_preempted.assert_called_once()
        winch.stop.assert_called_with()
        winch.set_position_envelope.assert_called_with(min=0, max=0)

    async def test_slow_motor_service(self):
        self.setup_move(simulated=False)
        loop = asyncio.get_running_loop()
        entered = asyncio.Event()
        release = threading.Event()

        def slow_velocity(**kwargs):
            loop.call_soon_threadsafe(entered.set)
            self.assertTrue(release.wait(2))

        self.velocity.side_effect = slow_velocity
        task = await self.start_goal()
        self.send_readings()
        try:
            await asyncio.wait_for(entered.wait(), timeout=1)

            # The motor call spans three seconds with both streams healthy
            for second in (11, 12, 13):
                self.stamp = rospy.Time(second)
                self.readings.update_motor(
                    Mock(mode=winch.Motion.MODE_VELOCITY, position=1))
                self.readings.update_depth(Mock(depth=0.9))
            self.readings.update_depth(Mock(depth=1.0))
        finally:
            release.set()
        await asyncio.wait_for(task, timeout=1)
        self.velocity.assert_called_once()
        self.server.set_succeeded.assert_called_once()

    async def test_cancel_during_envelope_write(self):
        self.setup_move()
        loop = asyncio.get_running_loop()
        entered = asyncio.Event()
        release = threading.Event()
        calls = []

        def write_envelope(min, max):
            if min == max == 0:
                calls.append('clear')
                return
            loop.call_soon_threadsafe(entered.set)
            self.assertTrue(release.wait(2))
            calls.append('envelope')

        winch.stop.side_effect = lambda: calls.append('stop')
        winch.set_position_envelope.side_effect = write_envelope
        task = await self.start_goal()
        self.send_readings()
        try:
            await asyncio.wait_for(entered.wait(), timeout=1)
            self.cancel_goal()
            self.cancel_goal()
        finally:
            release.set()
        await asyncio.wait_for(task, timeout=1)
        self.velocity.assert_not_called()
        self.assertEqual(calls, ['stop', 'envelope', 'stop', 'clear'])
        self.server.set_preempted.assert_called_once()

    async def test_shutdown_during_preempt_cleanup(self):
        self.setup_move()
        loop = asyncio.get_running_loop()
        writing = asyncio.Event()
        stopping = asyncio.Event()
        release = threading.Event()
        calls = []

        def write(**kwargs):
            loop.call_soon_threadsafe(writing.set)
            self.assertTrue(release.wait(2))
            calls.append('velocity')

        # Cancel a second time after the stop has entered the worker queue
        submit = winch.MotorCommands.submit

        def observe_submit(motor, service, **kwargs):
            future = submit(motor, service, **kwargs)
            if service == motor.finish:
                stopping.set()
            return future

        self.patch(patch.object(winch.MotorCommands, 'submit', observe_submit))
        self.velocity.side_effect = write
        winch.stop.side_effect = lambda: calls.append('stop')
        winch.set_position_envelope.side_effect = (
            lambda min, max: calls.append('clear' if min == max else 'limit'))
        task = await self.start_goal()
        self.send_readings()
        try:
            await asyncio.wait_for(writing.wait(), timeout=1)
            self.cancel_goal()
            await asyncio.wait_for(stopping.wait(), timeout=1)
            task.cancel()
            await asyncio.sleep(0)
        finally:
            release.set()
        await asyncio.wait_for(task, timeout=1)
        self.assertEqual(calls, ['stop', 'limit', 'velocity', 'stop', 'clear'])

    async def test_status_expires_during_envelope_write(self):
        self.setup_move(simulated=False)
        task = await self.start_goal()

        def write_envelope(min, max):
            if min != max:
                self.stamp += rospy.Duration(3)

        winch.set_position_envelope.side_effect = write_envelope
        self.send_readings()
        await asyncio.wait_for(task, timeout=1)
        self.velocity.assert_not_called()
        self.server.set_aborted.assert_called_once_with(
            text='Timed out waiting for depth status')

    async def test_command_error_stops_before_clearing_envelope(self):
        self.setup_move()
        self.velocity.side_effect = rospy.ServiceException('command failed')
        calls = Mock()
        calls.attach_mock(winch.stop, 'stop')
        calls.attach_mock(winch.set_position_envelope, 'envelope')
        task = await self.start_goal()
        self.send_readings()
        await asyncio.wait_for(task, timeout=1)
        calls.assert_has_calls([call.stop(), call.envelope(min=0, max=0)])
        self.server.set_aborted.assert_called_once_with(text='command failed')

    async def test_wall_clock_rewinds_during_command(self):
        self.setup_move(simulated=False)

        def rewind(**kwargs):
            self.stamp -= rospy.Duration(2)

        self.velocity.side_effect = rewind
        task = await self.start_goal()
        self.send_readings()
        await asyncio.wait_for(task, timeout=1)
        self.server.set_aborted.assert_called_once_with(
            text='ROS time moved backwards')
        winch.set_position_envelope.assert_called_with(min=0, max=0)

    async def test_on_time_readings_before_clock_jump(self):
        self.setup_move()
        task = await self.start_goal()

        # Both readings arrive before the deadline but the goal resumes after it
        self.clock.tick(rospy.Time(11))
        self.send_readings()
        asyncio.get_running_loop().call_soon(self.clock.tick, rospy.Time(13))
        self.send_readings()
        await asyncio.wait_for(self.commanded.wait(), timeout=1)
        self.cancel_goal()
        await asyncio.wait_for(task, timeout=1)
        self.server.set_aborted.assert_not_called()

    async def test_late_motor_status(self):
        self.setup_move()
        task = await self.start_goal()
        self.depth.update_soon(Mock(depth=0.7))
        asyncio.get_running_loop().call_soon(self.clock.tick, rospy.Time(13))
        self.send_readings()
        await asyncio.wait_for(task, timeout=1)
        self.server.set_aborted.assert_called_once_with(
            text='Timed out waiting for initial winch status')
        self.velocity.assert_not_called()

    async def test_startup_deadline(self):
        self.setup_move(start=0)
        task = await self.start_goal()
        self.clock.tick(rospy.Time(100))
        await asyncio.sleep(0)
        self.assertFalse(task.done())
        self.clock.tick(rospy.Time(102))
        await asyncio.wait_for(task, timeout=1)
        self.server.set_aborted.assert_called_once_with(
            text='Timed out waiting for initial winch status')

    async def test_readings_before_first_tick(self):
        self.setup_move(start=0)
        task = await self.start_goal()
        self.send_readings()
        await asyncio.sleep(0)
        self.clock.tick(rospy.Time(100))
        await asyncio.wait_for(self.commanded.wait(), timeout=1)
        self.clock.tick(rospy.Time(103))
        self.readings.update_depth(Mock(depth=1.0))
        await asyncio.wait_for(task, timeout=1)
        self.server.set_aborted.assert_called_once_with(
            text='Timed out waiting for depth status')

    async def test_already_at_target_skips_calibration(self):
        self.setup_move(simulated=False)
        self.params['~epsilon'] = 2.0
        set_param = self.patch(patch.object(rospy, 'set_param'))

        def slow_stop():
            self.stamp += rospy.Duration.from_sec(0.1)

        winch.stop.side_effect = slow_stop
        task = await self.start_goal(depth=2.0)
        self.send_readings()
        await asyncio.wait_for(task, timeout=1)
        self.server.set_succeeded.assert_called_once()
        self.velocity.assert_not_called()
        set_param.assert_not_called()

    async def test_target_reached_without_clock_advancing(self):
        self.setup_move()
        set_param = self.patch(patch.object(rospy, 'set_param'))
        task = await self.start_goal(depth=2.0)
        self.send_readings()
        await asyncio.wait_for(self.commanded.wait(), timeout=1)
        self.readings.update_depth(Mock(depth=2.0))
        await asyncio.wait_for(task, timeout=1)
        self.server.set_succeeded.assert_called_once()
        set_param.assert_not_called()

    async def test_rewind(self):
        self.setup_move()
        task = await self.start_goal()

        # The clock rewinds and catches up before the goal resumes
        self.clock.tick(rospy.Time(5))
        self.clock.tick(rospy.Time(11))
        await asyncio.wait_for(task, timeout=1)
        self.server.set_aborted.assert_called_once_with(
            text='ROS time moved backwards')
