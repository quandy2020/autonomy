# Copyright 2026 The Openbot Authors (duyongquan)

from __future__ import annotations

from unittest.mock import patch

from benchmark.dashboard.map_control import (
    NAVIGATE_RPC,
    navigate_request,
    pose_stamped,
    yaw_to_quat,
)


def test_yaw_to_quat_identity() -> None:
    q = yaw_to_quat(0.0)
    assert abs(q["w"] - 1.0) < 1e-6
    assert abs(q["z"]) < 1e-6


def test_navigate_request_shape() -> None:
    req = navigate_request(1.5, -2.0, 0.5, goal_id="g1")
    assert req["goal_id"] == "g1"
    assert len(req["waypoints"]) == 1
    pose = req["waypoints"][0]["pose"]
    assert pose["position"]["x"] == 1.5
    assert pose["position"]["y"] == -2.0
    assert "orientation" in pose


def test_pose_stamped_frame() -> None:
    p = pose_stamped(0, 0, frame_id="odom")
    assert p["header"]["frame_id"] == "odom"


def test_api_map_navigate_validation() -> None:
    from benchmark.dashboard.app import create_app

    client = create_app().test_client()
    bad = client.post("/api/map/navigate", json={})
    assert bad.status_code == 400
    with patch("benchmark.dashboard.map_control.send_navigate") as mock_send:
        mock_send.return_value = {
            "ok": True,
            "published": True,
            "stored": True,
            "method": NAVIGATE_RPC,
            "request": {},
            "result": {},
        }
        ok = client.post("/api/map/navigate", json={"x": 1.0, "y": 2.0, "yaw": 0.1})
    assert ok.status_code == 200
    body = ok.get_json()
    assert body["ok"] is True
    mock_send.assert_called_once()


def test_send_navigate_soft_queue_when_unavailable() -> None:
    from benchmark.dashboard.map_control import OtaClientError, get_pending_map_goal, send_navigate

    with patch("benchmark.dashboard.map_control.OtaClient") as mock_cls:
        mock_cls.return_value.call.side_effect = OtaClientError(
            "/automsgs.rpcs.navigation.NavigationService/Navigate failed: "
            "UNAVAILABLE: failed to connect"
        )
        out = send_navigate(3.0, -1.5, 0.25, target="localhost:50051")
    assert out["ok"] is True
    assert out["published"] is False
    assert out["stored"] is True
    goal = get_pending_map_goal()
    assert goal is not None
    assert goal["x"] == 3.0
    assert goal["y"] == -1.5


def test_api_map_teleop_validation() -> None:
    from benchmark.dashboard.app import create_app

    client = create_app().test_client()
    with patch("benchmark.dashboard.map_control.send_teleop") as mock_send:
        mock_send.return_value = {"ok": True, "action": "twist", "result": {}}
        res = client.post(
            "/api/map/teleop",
            json={"action": "twist", "linear_x": 0.2, "angular_z": -0.1},
        )
    assert res.status_code == 200
    assert res.get_json()["ok"] is True
