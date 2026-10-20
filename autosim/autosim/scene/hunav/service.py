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

"""Autolink services mirroring HuNavSim ``/compute_agents`` etc.

Autolink Python services use one protobuf type for request and response.
We therefore exchange the outer ``ComputeAgents`` / ``ComputeAgent`` /
``MoveAgent`` messages (``.request`` in, ``.response`` out).
"""

from __future__ import annotations

import logging
from typing import Any, Mapping

from autosim.scene.hunav.compute import ComputeAgentsEngine

logger = logging.getLogger(__name__)

DEFAULT_CHANNELS = {
    "compute_agents": "/compute_agents",
    "compute_agent": "/compute_agent",
    "move_agent": "/move_agent",
}


def _import_srvs() -> tuple[Any, Any, Any] | None:
    try:
        from automsgs.srvs import (
            compute_agent_pb2,
            compute_agents_pb2,
            move_agent_pb2,
        )

        return compute_agents_pb2, compute_agent_pb2, move_agent_pb2
    except Exception:
        return None


class HunavServiceHost:
    """Register HuNav Compute* services on an Autolink ``Node``."""

    def __init__(
        self,
        node: Any,
        engine: ComputeAgentsEngine | None = None,
        *,
        channels: Mapping[str, str] | None = None,
        settings: Mapping[str, Any] | None = None,
    ) -> None:
        self.node = node
        self.engine = engine or ComputeAgentsEngine(settings)
        self.channels = dict(DEFAULT_CHANNELS)
        if channels:
            self.channels.update(dict(channels))
        self._handles: list[Any] = []
        self._pb = _import_srvs()

    @property
    def available(self) -> bool:
        return self._pb is not None and self.engine.codec.available

    def start(self) -> bool:
        """Create Autolink services; return False if stubs / API missing."""
        if not self.available:
            logger.warning(
                "HunavServiceHost: automsgs hunav stubs missing; skip /compute_agents"
            )
            return False
        compute_agents_pb2, compute_agent_pb2, move_agent_pb2 = self._pb
        create = getattr(self.node, "create_service", None)
        if create is None:
            logger.warning("HunavServiceHost: node has no create_service")
            return False

        def on_compute_agents(msg: Any) -> Any:
            out = compute_agents_pb2.ComputeAgents()
            try:
                req = msg.request if hasattr(msg, "request") else msg
                dt = float(getattr(req, "dt", 0.0) or 0.0)
                updated = self.engine.step(
                    req.current_agents, req.robot, dt=dt or None
                )
                out.response.updated_agents.CopyFrom(updated)
                out.response.success = True
            except Exception as exc:
                out.response.success = False
                out.response.message = str(exc)
                logger.exception("compute_agents failed")
            return out

        def on_compute_agent(msg: Any) -> Any:
            out = compute_agent_pb2.ComputeAgent()
            try:
                req = msg.request if hasattr(msg, "request") else msg
                agent = self.engine.step_one(
                    req.current_agents,
                    req.robot,
                    int(req.agent_id),
                    dt=float(getattr(req, "dt", 0.0) or 0.0) or None,
                )
                out.response.updated_agent.CopyFrom(agent)
                out.response.success = True
            except Exception as exc:
                out.response.success = False
                out.response.message = str(exc)
            return out

        def on_move_agent(msg: Any) -> Any:
            out = move_agent_pb2.MoveAgent()
            try:
                req = msg.request if hasattr(msg, "request") else msg
                agent = self.engine.step_one(
                    req.current_agents,
                    req.robot,
                    int(req.agent_id),
                    dt=float(getattr(req, "dt", 0.0) or 0.0) or None,
                )
                out.response.updated_agent.CopyFrom(agent)
                out.response.success = True
            except Exception as exc:
                out.response.success = False
                out.response.message = str(exc)
            return out

        self._handles.append(
            create(
                self.channels["compute_agents"],
                on_compute_agents,
                compute_agents_pb2.ComputeAgents,
            )
        )
        self._handles.append(
            create(
                self.channels["compute_agent"],
                on_compute_agent,
                compute_agent_pb2.ComputeAgent,
            )
        )
        self._handles.append(
            create(
                self.channels["move_agent"],
                on_move_agent,
                move_agent_pb2.MoveAgent,
            )
        )
        logger.info(
            "HunavServiceHost listening on %s, %s, %s",
            self.channels["compute_agents"],
            self.channels["compute_agent"],
            self.channels["move_agent"],
        )
        return True

    def stop(self) -> None:
        """Drop service handle references (Autolink has no destroy API)."""
        self._handles.clear()
