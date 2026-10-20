# Copyright 2026 The Openbot Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Autolink client for HuNav ``/compute_agents`` (outer envelope messages)."""

from __future__ import annotations

import logging
from typing import Any

logger = logging.getLogger(__name__)


def create_compute_agents_client(
    node: Any, channel: str = "/compute_agents"
) -> Any | None:
    """Return an Autolink protobuf client or ``None`` if stubs / API missing."""
    try:
        from automsgs.srvs import compute_agents_pb2
    except Exception:
        logger.warning("create_compute_agents_client: hunav stubs missing")
        return None
    create = getattr(node, "create_client", None)
    if create is None:
        return None
    return create(channel, compute_agents_pb2.ComputeAgents)


def call_compute_agents(
    client: Any,
    agents_msg: Any,
    robot_msg: Any,
    *,
    dt: float = 0.0,
    timeout_sec: float = 1.0,
) -> Any | None:
    """Invoke ``/compute_agents``; return updated ``Agents`` or ``None``."""
    if client is None:
        return None
    try:
        from automsgs.srvs import compute_agents_pb2
    except Exception:
        return None
    envelope = compute_agents_pb2.ComputeAgents()
    envelope.request.current_agents.CopyFrom(agents_msg)
    envelope.request.robot.CopyFrom(robot_msg)
    envelope.request.dt = float(dt)
    try:
        resp = client.send_request(envelope, timeout_sec=timeout_sec)
    except Exception as exc:
        logger.debug("call_compute_agents failed: %s", exc)
        return None
    if resp is None:
        return None
    body = resp.response if hasattr(resp, "response") else resp
    if not getattr(body, "success", True):
        return None
    return body.updated_agents
