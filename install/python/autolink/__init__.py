from . import _core as _core  # noqa: F401
from ._action_sugar import ActionClient, ActionServer, SimpleActionServer
from ._core import (
    BagMessage,
    CancelResponse,
    ChannelReader,
    ChannelUtils,
    ChannelWriter,
    Client,
    ClientGoalHandle,
    Duration,
    GoalResponse,
    GoalStatus,
    Node,
    NodeUtils,
    Parameter,
    ParameterClient,
    ParameterServer,
    Rate,
    RecordReader,
    RecordWriter,
    ResultCode,
    ServerGoalHandle,
    Service,
    ServiceUtils,
    Time,
    Timer,
    init,
    is_shutdown,
    ok,
    shutdown,
    wait_for_shutdown,
)
from ._node_sugar import (
    create_client,
    create_reader,
    create_service,
    create_writer,
    spin,
)

Node.create_writer = create_writer
Node.create_reader = create_reader
Node.create_service = create_service
Node.create_client = create_client


def _node_spin(_self=None):
    spin()


Node.spin = _node_spin

__all__ = [
    "_core",
    "init",
    "ok",
    "shutdown",
    "is_shutdown",
    "wait_for_shutdown",
    "Node",
    "ChannelWriter",
    "ChannelReader",
    "Client",
    "Service",
    "create_writer",
    "create_reader",
    "create_service",
    "create_client",
    "spin",
    "Time",
    "Duration",
    "Rate",
    "Timer",
    "Parameter",
    "ParameterClient",
    "ParameterServer",
    "BagMessage",
    "RecordReader",
    "RecordWriter",
    "ChannelUtils",
    "NodeUtils",
    "ServiceUtils",
    "ActionClient",
    "ActionServer",
    "SimpleActionServer",
    "ClientGoalHandle",
    "ServerGoalHandle",
    "GoalStatus",
    "GoalResponse",
    "CancelResponse",
    "ResultCode",
]
