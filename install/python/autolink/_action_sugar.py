"""Python sugar for ActionClient / SimpleActionServer / ActionServer."""


def _retain(node, handle):
    actions = getattr(node, "_autolink_actions", None)
    if actions is None:
        actions = []
        setattr(node, "_autolink_actions", actions)
    actions.append(handle)
    return handle


def _serialize(message_class, message):
    if message_class is None:
        if isinstance(message, (bytes, bytearray)):
            return bytes(message)
        raise TypeError("goal/feedback/result must be bytes for RawData actions")
    if hasattr(message, "SerializeToString"):
        return message.SerializeToString()
    if isinstance(message, (bytes, bytearray)):
        return bytes(message)
    raise TypeError("expected protobuf message or bytes")


def _parse(message_class, data):
    if message_class is None:
        return data if isinstance(data, (bytes, bytearray)) else bytes(data)
    msg = message_class()
    msg.ParseFromString(data)
    return msg


def _resolve_types(goal_type, feedback_type, result_type):
    def one(t):
        if t is None or t == "RawData":
            return None
        if isinstance(t, str):
            raise TypeError("Action message types must be protobuf classes or None")
        if getattr(t, "DESCRIPTOR", None) is None:
            raise TypeError("Action message types must be protobuf classes or None")
        return t

    return one(goal_type), one(feedback_type), one(result_type)


class ActionClient:
    def __init__(self, node, action_name, goal_type=None, feedback_type=None,
                 result_type=None):
        from ._core import _ActionClient

        self._goal_cls, self._feedback_cls, self._result_cls = _resolve_types(
            goal_type, feedback_type, result_type)
        self._raw = _ActionClient(node, action_name)
        _retain(node, self)

    def server_is_ready(self):
        return self._raw.server_is_ready()

    def wait_for_server(self, timeout_sec=-1.0):
        return self._raw.wait_for_server(timeout_sec)

    def send_goal(self, goal, feedback_cb=None, timeout_sec=30.0):
        goal_bytes = _serialize(self._goal_cls, goal)
        fb = None
        if feedback_cb is not None:
            def fb(data, handle):
                feedback_cb(_parse(self._feedback_cls, data), handle)

        handle = self._raw.send_goal(goal_bytes, fb, timeout_sec)
        if handle is None:
            return None
        return _ClientGoalHandle(handle, self._result_cls)

    def send_goal_async(self, goal, goal_response_cb=None, feedback_cb=None,
                        result_cb=None):
        goal_bytes = _serialize(self._goal_cls, goal)

        def wrap_goal(handle):
            if goal_response_cb is None:
                return
            goal_response_cb(
                None if handle is None else _ClientGoalHandle(
                    handle, self._result_cls))

        def wrap_fb(data, handle):
            if feedback_cb is None:
                return
            feedback_cb(_parse(self._feedback_cls, data),
                        _ClientGoalHandle(handle, self._result_cls))

        def wrap_result(data, code):
            if result_cb is None:
                return
            result_cb(_parse(self._result_cls, data), code)

        return self._raw.send_goal_async(
            goal_bytes,
            None if goal_response_cb is None else wrap_goal,
            None if feedback_cb is None else wrap_fb,
            None if result_cb is None else wrap_result)


class _ClientGoalHandle:
    def __init__(self, raw, result_cls):
        self._raw = raw
        self._result_cls = result_cls

    @property
    def goal_id(self):
        return self._raw.goal_id

    @property
    def status(self):
        return self._raw.status

    def is_succeeded(self):
        return self._raw.is_succeeded()

    def is_canceled(self):
        return self._raw.is_canceled()

    def is_aborted(self):
        return self._raw.is_aborted()

    def get_result(self, timeout_sec=60.0):
        data = self._raw.get_result(timeout_sec)
        return _parse(self._result_cls, data)

    def get_result_code(self):
        return self._raw.get_result_code()

    def cancel(self, timeout_sec=5.0):
        return self._raw.cancel(timeout_sec)


class SimpleActionServer:
    def __init__(self, node, action_name, execute_cb, goal_type=None,
                 feedback_type=None, result_type=None, completion_cb=None):
        from ._core import _SimpleActionServer

        self._goal_cls, self._feedback_cls, self._result_cls = _resolve_types(
            goal_type, feedback_type, result_type)
        self._raw = _SimpleActionServer(node, action_name, execute_cb,
                                        completion_cb)
        _retain(node, self)

    def get_current_goal(self):
        return _parse(self._goal_cls, self._raw.get_current_goal())

    def is_cancel_requested(self):
        return self._raw.is_cancel_requested()

    def is_preempt_requested(self):
        return self._raw.is_preempt_requested()

    def accept_pending_goal(self):
        return self._raw.accept_pending_goal()

    def publish_feedback(self, feedback):
        self._raw.publish_feedback(_serialize(self._feedback_cls, feedback))

    def succeeded_current(self, result=b""):
        self._raw.succeeded_current(_serialize(self._result_cls, result))

    def terminate_current(self, result=b""):
        self._raw.terminate_current(_serialize(self._result_cls, result))

    def has_feedback_subscriber(self):
        return self._raw.has_feedback_subscriber()

    def activate(self):
        self._raw.activate()

    def deactivate(self):
        self._raw.deactivate()


class ActionServer:
    def __init__(self, node, action_name, handle_goal, handle_cancel,
                 handle_accepted, goal_type=None, feedback_type=None,
                 result_type=None):
        from ._core import GoalResponse, CancelResponse, _ActionServer

        self._goal_cls, self._feedback_cls, self._result_cls = _resolve_types(
            goal_type, feedback_type, result_type)

        def on_goal(goal_id, goal_bytes):
            out = handle_goal(goal_id, _parse(self._goal_cls, goal_bytes))
            return out if out is not None else GoalResponse.ACCEPT_AND_EXECUTE

        def on_cancel(handle):
            wrapped = _ServerGoalHandle(handle, self._goal_cls,
                                        self._feedback_cls, self._result_cls)
            out = handle_cancel(wrapped)
            return out if out is not None else CancelResponse.ACCEPT

        def on_accepted(handle):
            handle_accepted(_ServerGoalHandle(handle, self._goal_cls,
                                              self._feedback_cls,
                                              self._result_cls))

        self._raw = _ActionServer(node, action_name, on_goal, on_cancel,
                                  on_accepted)
        _retain(node, self)

    def has_feedback_subscriber(self):
        return self._raw.has_feedback_subscriber()


class _ServerGoalHandle:
    def __init__(self, raw, goal_cls, feedback_cls, result_cls):
        self._raw = raw
        self._goal_cls = goal_cls
        self._feedback_cls = feedback_cls
        self._result_cls = result_cls

    def get_goal(self):
        return _parse(self._goal_cls, self._raw.get_goal())

    @property
    def goal_id(self):
        return self._raw.goal_id

    def publish_feedback(self, feedback):
        self._raw.publish_feedback(_serialize(self._feedback_cls, feedback))

    def succeed(self, result=b""):
        self._raw.succeed(_serialize(self._result_cls, result))

    def abort(self, result=b""):
        self._raw.abort(_serialize(self._result_cls, result))

    def canceled(self, result=b""):
        self._raw.canceled(_serialize(self._result_cls, result))

    def execute(self):
        self._raw.execute()

    def is_active(self):
        return self._raw.is_active()

    def is_executing(self):
        return self._raw.is_executing()

    def is_canceling(self):
        return self._raw.is_canceling()

    def get_status(self):
        return self._raw.get_status()
