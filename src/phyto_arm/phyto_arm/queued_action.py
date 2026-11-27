from threading import RLock, Thread

from action_msgs.msg import GoalStatus
from rclpy.action import ActionServer, CancelResponse
from rclpy.action.server import GoalEvent
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.task import Future


class QueuedGoal:
    def __init__(self, handle, action_type, on_finish):
        self.handle = handle
        self.action_type = action_type
        self.on_finish = on_finish
        self.preempt_requested = False
        self.result = Future()

    def finish(self, status, result=None, text=''):
        if self.result.done():
            return
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.handle.succeed()
        elif status == GoalStatus.STATUS_ABORTED:
            self.handle.abort()
        else:
            # A newer goal can preempt this one without a client cancel request.
            if not self.handle.is_cancel_requested:
                self.handle._update_state(GoalEvent.CANCEL_GOAL)
            self.handle.canceled()
        if result is None:
            result = self.action_type.Result()
        if self.on_finish is not None:
            self.on_finish(self.handle, result, text)
        self.result.set_result(result)


class QueuedActionServer:
    """Run one instrument operation at a time and retain only the newest waiting goal."""

    def __init__(self, node, action_type, name, execute, *, on_accept=None, on_finish=None):
        self.node = node
        self.action_type = action_type
        self.execute = execute
        self.on_accept, self.on_finish = on_accept, on_finish
        self.lock = RLock()
        self.current, self.pending = None, None
        self.worker = None
        self.results = {}
        self.server = ActionServer(node, action_type, name, self._execute_result,
                                   callback_group=MutuallyExclusiveCallbackGroup(),
                                   handle_accepted_callback=self._accept, cancel_callback=self._cancel)

    async def _execute_result(self, handle):
        # The native action gets its result as soon as the instrument reaches a
        # terminal state, even while the worker is still cleaning up the operation.
        with self.lock:
            result = self.results.pop(bytes(handle.goal_id.uuid))
        return await result

    def _accept(self, handle):
        with self.lock:
            if self.on_accept is not None:
                self.on_accept(handle)
            goal = QueuedGoal(handle, self.action_type, self.on_finish)
            self.results[bytes(handle.goal_id.uuid)] = goal.result
            handle.execute()
            if self.pending is not None:
                self.pending.finish(GoalStatus.STATUS_CANCELED)
            self.pending = goal
            if self.current is not None:
                self.current.preempt_requested = True
            if self.worker is None:
                self.worker = Thread(target=self._run, daemon=True)
                self.worker.start()

    def _cancel(self, handle):
        with self.lock:
            for goal in (self.current, self.pending):
                if goal is not None and goal.handle == handle:
                    goal.preempt_requested = True
                    return CancelResponse.ACCEPT
        return CancelResponse.REJECT

    def _run(self):
        while True:
            with self.lock:
                if self.pending is None:
                    self.worker = None
                    return
                goal = self.current = self.pending
                self.pending = None
            try:
                self.execute(goal)
            except Exception as exc:
                self.node.get_logger().error(f'Exception in action execute callback: {exc}')
            with self.lock:
                if not goal.result.done():
                    goal.finish(GoalStatus.STATUS_ABORTED, text='Execute callback returned without a result')
                self.current = None
