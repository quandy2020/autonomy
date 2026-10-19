# Copyright 2026 The Openbot Authors (duyongquan)

from __future__ import annotations

from unittest.mock import MagicMock, patch

from benchmark.dashboard.ota_client import (
    OTA_STATE_LABELS,
    OtaClient,
    OtaClientError,
    enrich_ota_status,
)


def test_enrich_ota_status_labels() -> None:
    out = enrich_ota_status(
        {
            "state": "OTA_STATE_DOWNLOADING",
            "progress": 0.42,
            "current_version": "1.0",
            "target_version": "1.1",
            "detail": "fetch",
        }
    )
    assert out["state_label"] == OTA_STATE_LABELS["OTA_STATE_DOWNLOADING"]
    assert out["progress_pct"] == 42.0
    assert out["current_version"] == "1.0"


def test_probe_without_deps() -> None:
    with patch("benchmark.dashboard.rpc_client.grpc_available", return_value=False):
        with patch(
            "benchmark.dashboard.rpc_client.find_automsgs_root",
            return_value=None,
        ):
            with patch("benchmark.dashboard.rpc_client.find_protoc", return_value=None):
                probe = OtaClient(target="127.0.0.1:50051").probe()
    assert probe["ready"] is False
    assert probe["grpcio"] is False
    assert probe["transport"] == "python-grpc"


def test_call_raises_without_grpcio() -> None:
    client = OtaClient(target="127.0.0.1:9")
    with patch("benchmark.dashboard.rpc_client.grpc_available", return_value=False):
        with patch(
            "benchmark.dashboard.rpc_client.find_automsgs_root",
            return_value=MagicMock(),
        ):
            try:
                client.get_ota_status()
                raise AssertionError("expected OtaClientError")
            except OtaClientError as exc:
                assert "grpcio" in str(exc).lower() or "protoc" in str(exc).lower()


def test_api_ota_snapshot_mocked() -> None:
    from benchmark.dashboard.app import create_app

    fake = MagicMock()
    fake.snapshot.return_value = {
        "ok": True,
        "probe": {"ready": True, "target": "localhost:50051", "transport": "python-grpc"},
        "supports_ota": True,
        "capabilities": {"supportsOta": True},
        "info": {"model": "demo", "firmwareVersion": "0.1"},
        "ota": enrich_ota_status({"state": "OTA_STATE_IDLE", "progress": 0}),
        "error": "",
    }
    with patch("benchmark.dashboard.ota_client.get_client", return_value=fake):
        client = create_app().test_client()
        res = client.get("/api/ota/snapshot")
    assert res.status_code == 200
    body = res.get_json()
    assert body["ok"] is True
    assert body["supports_ota"] is True


def test_api_ota_start_requires_uri() -> None:
    from benchmark.dashboard.app import create_app

    res = create_app().test_client().post("/api/ota/start", json={})
    assert res.status_code == 400
    assert res.get_json()["ok"] is False


def test_api_rpc_link_mocked() -> None:
    from benchmark.dashboard.app import create_app

    fake = MagicMock()
    fake.check_link.return_value = {
        "ok": True,
        "linked": True,
        "state": "online",
        "label": "已连接",
        "target": "127.0.0.1:50051",
        "latency_ms": 12.3,
        "error": "",
    }
    with patch("benchmark.dashboard.rpc_client.RpcClient", return_value=fake):
        res = create_app().test_client().get("/api/rpc/link?target=127.0.0.1:50051")
    assert res.status_code == 200
    body = res.get_json()
    assert body["linked"] is True
    assert body["state"] == "online"
    fake.check_link.assert_called_once()
