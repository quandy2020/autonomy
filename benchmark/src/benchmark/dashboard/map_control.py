# Copyright 2026 The Openbot Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0.

"""Map interactive control → Navigation / Teleop gRPC (Python grpcio)."""

from __future__ import annotations

import math
from typing import Any

from benchmark.dashboard.ota_client import OtaClient, OtaClientError

NAVIGATE_RPC = "automsgs.rpcs.navigation.NavigationService/Navigate"
TELEOP_VELOCITY_RPC = "automsgs.rpcs.teleop.TeleopService/Velocity"
TELEOP_CANCEL_RPC = "automsgs.rpcs.teleop.TeleopService/Cancel"

# Last Goal Pose from the dashboard map (meters), kept even when gRPC is down.
_pending_map_goal: dict[str, Any] | None = None


def get_pending_map_goal() -> dict[str, Any] | None:
    return dict(_pending_map_goal) if _pending_map_goal else None


def store_pending_map_goal(x: float, y: float, yaw: float = 0.0) -> dict[str, Any]:
    global _pending_map_goal
    _pending_map_goal = {
        "x": float(x),
        "y": float(y),
        "yaw": float(yaw),
    }
    return dict(_pending_map_goal)


def _is_unavailable(exc: BaseException) -> bool:
    text = str(exc)
    return "UNAVAILABLE" in text or "Connection refused" in text or "failed to connect" in text


def yaw_to_quat(yaw: float) -> dict[str, float]:
    half = float(yaw) * 0.5
    return {"x": 0.0, "y": 0.0, "z": math.sin(half), "w": math.cos(half)}


def pose_stamped(x: float, y: float, yaw: float = 0.0, *, frame_id: str = "map") -> dict[str, Any]:
    return {
        "header": {"frame_id": frame_id},
        "pose": {
            "position": {"x": float(x), "y": float(y), "z": 0.0},
            "orientation": yaw_to_quat(yaw),
        },
    }


def navigate_request(x: float, y: float, yaw: float = 0.0, *, goal_id: str = "") -> dict[str, Any]:
    body: dict[str, Any] = {
        "waypoints": [pose_stamped(x, y, yaw)],
    }
    if goal_id:
        body["goal_id"] = goal_id
    return body


def send_navigate(
    x: float,
    y: float,
    yaw: float = 0.0,
    *,
    target: str | None = None,
    timeout_s: float = 8.0,
) -> dict[str, Any]:
    stored = store_pending_map_goal(x, y, yaw)
    client = OtaClient(target=target)
    data = navigate_request(x, y, yaw, goal_id="dash-map")
    try:
        # Navigate is server-streaming; RpcClient.call returns the last frame.
        result = client.call(NAVIGATE_RPC, data, timeout_s=timeout_s)
    except OtaClientError as exc:
        if _is_unavailable(exc):
            # Soft-queue: keep goal for 命令/重试; UI still treats as recorded.
            return {
                "ok": True,
                "published": False,
                "stored": True,
                "goal": stored,
                "method": NAVIGATE_RPC,
                "request": data,
                "warning": str(exc),
                "error": str(exc),
            }
        raise
    return {
        "ok": True,
        "published": True,
        "stored": True,
        "goal": stored,
        "method": NAVIGATE_RPC,
        "request": data,
        "result": result,
    }


def send_teleop(
    action: str,
    *,
    linear_x: float = 0.0,
    angular_z: float = 0.0,
    target: str | None = None,
    goal_id: str = "dash-teleop",
    timeout_s: float = 3.0,
) -> dict[str, Any]:
    """Send TeleopService.Velocity client-stream frames (START / TWIST / STOP)."""
    client = OtaClient(target=target)
    act = (action or "twist").strip().lower()
    if act == "cancel":
        result = client.call(TELEOP_CANCEL_RPC, {"goal_id": goal_id}, timeout_s=timeout_s)
        return {"ok": True, "method": TELEOP_CANCEL_RPC, "result": result}

    if act == "start":
        messages = [{"command": "VELOCITY_COMMAND_START", "goal_id": goal_id}]
    elif act == "stop":
        messages = [{"command": "VELOCITY_COMMAND_STOP", "goal_id": goal_id}]
    else:
        # One bidi session: START + TWIST.
        messages = [
            {"command": "VELOCITY_COMMAND_START", "goal_id": goal_id},
            {
                "command": "VELOCITY_COMMAND_TWIST",
                "goal_id": goal_id,
                "twist": {
                    "header": {"frame_id": "base_link"},
                    "twist": {
                        "linear": {"x": float(linear_x), "y": 0.0, "z": 0.0},
                        "angular": {"x": 0.0, "y": 0.0, "z": float(angular_z)},
                    },
                },
            },
        ]
    try:
        result = client.call_client_stream(
            TELEOP_VELOCITY_RPC, messages, timeout_s=timeout_s
        )
    except OtaClientError:
        raise
    return {
        "ok": True,
        "method": TELEOP_VELOCITY_RPC,
        "action": act,
        "result": result,
    }
