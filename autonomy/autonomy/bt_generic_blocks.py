import py_trees
import time
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from rclpy.qos import qos_profile_sensor_data

Status = py_trees.common.Status
Access = py_trees.common.Access
SETUP_WAIT_SEC = 30.0   # Maximum wait in s to any service/action during setup

class BBBehaviour(py_trees.behaviour.Behaviour):
    """Base client for blackboard for passed keys in `reads` and `writes` with ROS node passed in setup()."""

    def __init__(self, name, reads=(), writes=()):
        super().__init__(name)
        self.node = None
        self.bb = self.attach_blackboard_client(name=name)
        for key in reads:
            self.bb.register_key(key=key, access=Access.READ)
        for key in writes:
            self.bb.register_key(key=key, access=Access.WRITE)

    def setup(self, **kwargs):
        self.node = kwargs["node"]

    def bb_get(self, key, default=None):
        try:
            return self.bb.get(key)
        except KeyError:
            return default

class TopicToBlackboard(BBBehaviour):
    """Copy topic's data on a blackboard. Always return SUCCESS.
    `transform(msg)` returns dict {key: value} or None to ignore message.
    If `count_key` given, count valid messages received.
    """

    def __init__(self, name, topic, msg_type, keys, transform, count_key=None):
        writes = list(keys) + ([count_key] if count_key else [])
        super().__init__(name, writes=writes)
        self.topic, self.msg_type = topic, msg_type
        self.transform, self.count_key = transform, count_key
        self._pending, self._count = {}, 0

    def setup(self, **kwargs):
        super().setup(**kwargs)
        self.node.create_subscription(self.msg_type, self.topic, self._on_msg, qos_profile_sensor_data)

    def _on_msg(self, msg):
        data = self.transform(msg)
        if data is not None:
            self._pending.update(data)
            self._count += 1

    def update(self):
        if self._pending:
            for key, value in self._pending.items():
                self.bb.set(key, value)
            if self.count_key:
                self.bb.set(self.count_key, self._count)
            self._pending = {}
        return Status.SUCCESS

class ActionCall(BBBehaviour):
    """Send a Goal to an Action server and remain RUNNING until it finishes."""

    def __init__(self, name, action_type, action_name, make_goal, reads=(), format_feedback=None):
        super().__init__(name, reads=reads)
        self.action_type, self.action_name = action_type, action_name
        self.make_goal, self.format_feedback = make_goal, format_feedback
        self.client = None
        self._goal_future = self._result_future = self._goal_handle = None
        self._active, self._error = False, None

    def setup(self, **kwargs):
        super().setup(**kwargs)
        self.client = ActionClient(self.node, self.action_type, self.action_name)
        if not self.client.wait_for_server(timeout_sec=SETUP_WAIT_SEC):
            raise RuntimeError(f"[{self.name}] action server '{self.action_name}' not available")

    def initialise(self):
        self._goal_future = self._result_future = self._goal_handle = None
        self._active, self._error = False, None
        try:
            goal = self.make_goal(self.bb)
        except KeyError as e:
            self._error = f"Missing data on blackboard: {e}"
            return
        self.node.get_logger().info(f"[{self.name}] sending goal to '{self.action_name}'")
        self._goal_future = self.client.send_goal_async(goal, feedback_callback=self._on_feedback)
        self._active = True

    def _on_feedback(self, fb):
        if self.format_feedback: self.feedback_message = self.format_feedback(fb)

    def update(self):
        if self._error:
            self.feedback_message = self._error
            return Status.FAILURE
        
        if self._result_future is None:
            if not self._goal_future.done():
                return Status.RUNNING
            
            handle = self._goal_future.result()
            if not handle.accepted:
                self._active = False
                self.feedback_message = "Goal Refused"
                return Status.FAILURE
            
            self._goal_handle = handle
            self._result_future = handle.get_result_async()
            return Status.RUNNING
        
        if not self._result_future.done():
            return Status.RUNNING
        
        self._active = False
        code = self._result_future.result().status
        if code == GoalStatus.STATUS_SUCCEEDED:
            self.feedback_message = "Completed"
            return Status.SUCCESS
        self.feedback_message = f"Ended with status {code}"
        return Status.FAILURE

    def terminate(self, new_status):
        if not self._active: return

        self._active = False
        if self._goal_handle is not None: self._goal_handle.cancel_goal_async()
        elif self._goal_future is not None: self._goal_future.add_done_callback(self._cancel_when_accepted)

    @staticmethod
    def _cancel_when_accepted(future):
        handle = future.result()
        if handle.accepted: handle.cancel_goal_async()

class ServiceCall(BBBehaviour):
    """Call a service (non-blocking).
    `on_response(bb, res) -> bool` callback determines how response is handled."""

    def __init__(self, name, srv_type, srv_name, make_request, on_response=None,
                 reads=(), writes=(), timeout=10.0):
        super().__init__(name, reads=reads, writes=writes)
        self.srv_type, self.srv_name = srv_type, srv_name
        self.make_request, self.on_response, self.timeout = make_request, on_response, timeout
        self.client = None
        self._future, self._deadline, self._error = None, 0.0, None

    def setup(self, **kwargs):
        super().setup(**kwargs)
        self.client = self.node.create_client(self.srv_type, self.srv_name)
        if not self.client.wait_for_service(timeout_sec=SETUP_WAIT_SEC):
            raise RuntimeError(f"[{self.name}] service '{self.srv_name}' not available")

    def initialise(self):
        self._future, self._error = None, None
        self._deadline = time.monotonic() + self.timeout
        try:
            request = self.make_request(self.bb)
        except KeyError as e:
            self._error = f"Missing data on the blackboard: {e}"
            return
        self.node.get_logger().info(f"[{self.name}] call '{self.srv_name}'")
        self._future = self.client.call_async(request)

    def update(self):
        if self._error:
            self.feedback_message = self._error
            return Status.FAILURE
        if not self._future.done():
            if time.monotonic() > self._deadline:
                self.feedback_message = "timeout"
                return Status.FAILURE
            return Status.RUNNING
        ok = self.on_response(self.bb, self._future.result()) if self.on_response else True
        self.feedback_message = "ok" if ok else "not ok"
        return Status.SUCCESS if ok else Status.FAILURE