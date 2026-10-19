# Copyright 2026 The Openbot Authors (duyongquan)

from __future__ import annotations

from pathlib import Path
import tempfile

from benchmark.dashboard.rpc_client import (
    RpcClient,
    describe_rpc_request,
    ensure_proto_include,
    find_automsgs_root,
    load_descriptor_pool,
)


def test_load_automsgs_rpc_descriptors() -> None:
    root = find_automsgs_root()
    assert root is not None
    include = ensure_proto_include(
        root, Path(tempfile.gettempdir()) / "benchmark-ota-proto-include"
    )
    pool = load_descriptor_pool(include)
    svc = pool.FindServiceByName("automsgs.rpcs.navigation.NavigationService")
    names = {m.name for m in svc.methods}
    assert "Navigate" in names
    assert "GetStatus" in names


def test_rpc_probe_ready_when_deps_present() -> None:
    probe = RpcClient(target="127.0.0.1:50051").probe()
    assert probe["transport"] == "python-grpc"
    if probe["grpcio"] and probe["protoc"] and probe["automsgs_root"]:
        assert probe["ready"] is True


def test_describe_rpc_request_autofills_navigate_json() -> None:
    info = describe_rpc_request(
        "automsgs.rpcs.navigation.NavigationService/Navigate"
    )
    assert info["ok"] is True
    body = info["default_body"]
    assert isinstance(body, dict)
    assert "goal_id" in body
    assert isinstance(body.get("waypoints"), list)
    assert body["waypoints"]
    pose = body["waypoints"][0]["pose"]["position"]
    assert set(pose) >= {"x", "y", "z"}
    paths = {f["path"] for f in info["fields"]}
    assert "goal_id" in paths
    assert any(p.startswith("waypoints.0.") for p in paths)


def test_named_catalog_autofills_non_curated_rpc() -> None:
    from benchmark.dashboard.catalog import list_named_rpc_tasks

    named = list_named_rpc_tasks()
    # Pick a scanned method that is not the curated navigate shortcut.
    item = next(
        (
            m
            for m in named
            if m.get("method") == "GetStatus"
            and m.get("service") == "NavigationService"
        ),
        None,
    )
    assert item is not None
    assert isinstance(item.get("default_body"), dict)
    # Empty request messages stay {}, otherwise proto fields appear.
    assert "fields" in item
