# Copyright 2026 The Openbot Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0.

from __future__ import annotations

from benchmark.dashboard.catalog import (
    catalog_payload,
    collect_options_from_config,
    list_task_configs,
    params_to_overrides,
    replay_options_from_config,
    sanitize_params,
)


def test_catalog_payload_has_tasks_and_registry() -> None:
    payload = catalog_payload()
    assert "registry" in payload
    assert "navigate" in payload["registry"]["tasks"]
    assert "dock" in payload["registry"]["tasks"]
    assert payload["registry"]["simulators"] == ["isaac", "habitat"]
    assert payload["registry"]["humans"]
    assert payload["scene"]["simulator"] in {"isaac", "habitat"}
    assert payload["missions"]
    assert {m["id"] for m in payload["missions"]} >= {
        "navigate",
        "dock",
        "explore_mapping",
        "patrol",
        "teleop",
    }
    assert payload["models"]
    assert any(m["id"] == "pure_pursuit" for m in payload["models"])
    assert payload["layout"].get("compact") is True
    assert payload["bicmap"]["offline"] is True
    assert payload["bicmap"]["local_js"].endswith("bicMap.ext.min.js")
    assert any(s["id"] == "navigate" for s in payload["bicmap"]["scenes"])
    assert payload["curve_channels"]
    assert payload["tasks"]
    assert payload["experiment"]["task_name"]
    assert payload["collect"]["channels"]
    assert "datasets" in payload
    assert payload["config_path"].endswith(".yaml")
    assert isinstance(payload["config_text"], str)
    assert payload["scene"]["simulator"]
    assert payload["scene"]["humans"]
    assert payload["params"]["fields"]
    assert "task.timeout_s" in payload["params"]["values"]
    assert "task.difficulty" in {f["key"] for f in payload["params"]["fields"]}
    assert "worlds" in payload["registry"]
    assert payload["replay"]["source"]
    assert "rate" in payload["replay"]


def test_replay_options_defaults() -> None:
    replay = replay_options_from_config()
    assert replay["source"]
    assert replay["rate"] > 0
    assert "loop" in replay
    assert "dry_run" in replay


def test_sanitize_params_allowlist_and_overrides() -> None:
    clean = sanitize_params(
        {
            "task.difficulty": 0.7,
            "task.episodes": 5,
            "os.system": "nope",
            "humans.enabled": True,
        }
    )
    assert clean == {
        "task.difficulty": 0.7,
        "task.episodes": 5,
        "humans.enabled": True,
    }
    overrides = params_to_overrides(clean)
    assert "task.difficulty=0.7" in overrides
    assert "humans.enabled=true" in overrides
    assert not any("os.system" in o for o in overrides)


def test_flask_app_exposes_export_and_history_routes() -> None:
    from benchmark.dashboard.app import create_app

    app = create_app()
    client = app.test_client()
    assert client.get("/api/history").status_code == 200
    summary = client.get("/api/export/summary")
    assert summary.status_code == 200
    assert summary.mimetype == "application/json"
    curves = client.get("/api/export/curves")
    assert curves.status_code == 200
    catalog = client.get("/api/catalog")
    assert catalog.status_code == 200
    body = catalog.get_json()
    assert "simulators" in body["registry"]
    assert "humans" in body["registry"]


def test_index_compact_layout() -> None:
    from benchmark.dashboard.app import create_app

    html = create_app().test_client().get("/").get_data(as_text=True)
    assert 'id="btn-run"' in html
    assert 'id="planner-select"' in html
    assert 'id="mission-grid"' in html
    assert 'id="bicmap-container"' in html
    assert "map-stage" in html
    assert 'id="btn-map-fit"' in html
    assert 'id="scene-select"' in html
    assert 'id="teleop-pad"' in html
    assert 'data-map-mode="navigate"' in html
    assert "map-glass" in html
    assert 'id="map-pose-ghost"' in html
    assert "Goal Pose" in html
    assert 'id="map-displays"' in html
    assert 'id="btn-map-displays"' in html
    assert "Displays" in html
    assert "map-toolbar" in html
    assert 'id="btn-map-display-add"' in html
    assert 'id="map-display-add-kind"' in html
    assert 'id="map-display-add-channel"' not in html
    assert "选 topic 后 Add" in html
    assert 'id="map-images"' in html
    assert 'id="btn-map-images"' in html
    assert 'id="map-images-body"' in html
    assert 'id="btn-map-image-add"' in html
    assert 'id="map-image-add-topic"' in html
    assert 'id="map-image-add-channel"' in html
    assert "选 channel 后 Add" in html
    assert 'id="result-cards"' in html
    assert 'id="curve-tip"' in html
    assert 'data-tab="curves"' in html
    assert 'data-tab="data"' in html
    assert 'id="tab-curves"' in html
    assert 'id="tab-params"' in html
    assert 'id="tab-data"' in html
    assert 'id="tab-btn-params"' not in html
    assert 'id="param-fields"' in html
    assert 'id="btn-params-reset"' in html
    assert 'class="settings-layout"' in html
    assert 'id="settings-runtime"' in html
    assert 'id="settings-model"' in html
    assert html.count("settings-layout") >= 4
    assert 'settings-groups' in html
    assert 'id="btn-params-reset-side"' not in html
    assert 'id="btn-record-run"' in html
    assert 'id="btn-play-run"' in html
    assert 'id="channel-list"' in html
    assert 'class="channel-tree"' in html
    assert 'data-data-mode="record"' in html
    assert 'data-data-mode="play"' in html
    assert 'id="data-pane-record"' in html
    assert 'id="data-pane-play"' in html
    assert 'data-tab="ota"' in html
    assert 'id="tab-ota"' in html
    assert 'id="btn-ota-start"' in html
    assert 'id="btn-ota-rpc"' in html
    assert 'class="side-nav"' in html
    assert 'data-nav="settings"' in html
    assert 'data-nav="tasks"' in html
    assert 'data-nav="data"' in html
    assert 'data-nav="test"' in html
    assert 'data-nav="ops"' in html
    assert 'data-task-kind="orchestration"' in html
    assert 'data-task-kind="schedule"' in html
    assert 'data-task-kind="named"' in html
    assert "task-kind-btn" not in html
    assert 'id="tasks-chrome-actions"' in html
    assert 'id="record-enable"' in html
    assert 'id="tab-orchestrate"' in html
    assert "orch-layout" in html
    assert 'id="mission-grid"' in html
    assert 'id="orch-summary"' in html
    assert 'id="btn-orch-all"' in html
    assert 'id="tab-schedule"' in html
    assert 'id="sched-mission-list"' in html
    assert 'id="sched-steps"' in html
    assert 'id="sched-resize"' in html
    assert "sched-layout" in html
    assert 'id="tab-named"' in html
    assert 'id="named-task-list"' in html
    assert 'id="named-group-tabs"' in html
    assert "cmd-layout" in html
    assert 'id="btn-named-send"' in html
    assert 'id="btn-named-copy"' in html
    assert 'id="named-history"' in html
    assert 'id="named-json-summary"' in html
    assert 'id="named-form"' in html
    assert 'id="named-response"' in html
    assert 'id="named-link"' in html
    assert 'id="btn-named-link"' in html
    assert 'id="cmd-resize"' in html
    assert 'id="mobile-bar"' not in html
    assert 'data-preset=' not in html


def test_task_kinds_catalog() -> None:
    from benchmark.dashboard.catalog import list_named_rpc_tasks, task_kinds_payload

    kinds = task_kinds_payload()
    assert set(kinds) >= {"orchestration", "schedule", "named"}
    orch_ids = {m["id"] for m in kinds["orchestration"]["items"]}
    assert {"navigate", "dock", "follow", "teleop"} <= orch_ids
    named = list_named_rpc_tasks()
    named_ids = {m["id"] for m in named}
    assert "rpc.navigate" in named_ids
    assert any(m.get("rpc", "").endswith("Navigate") for m in named)
    # Full proto scan: every major service should appear.
    services = {m.get("service") for m in named}
    assert {
        "NavigationService",
        "ChargeService",
        "FollowService",
        "TeleopService",
        "MapService",
        "LocalizationService",
        "SystemService",
        "SensorService",
        "ExplorationService",
        "VoiceService",
    } <= services
    assert len(named) >= 50


def test_history_archive_and_restore() -> None:
    from benchmark.dashboard.monitor import JobState, MonitorHub

    hub = MonitorHub()
    job = JobState(
        job_id="abc123dead",
        status="success",
        tasks=["navigate", "dock"],
        task_states={"navigate": "done", "dock": "done"},
        summary={"navigate": {"summary": {"ok": 1}}},
        curves={"navigate": {"latest": {"t": [0.0, 1.0], "v": [0.0, 0.5]}}},
        map_tracks={"navigate": {"task": "navigate", "path": [{"x": 0, "y": 0}]}},
        finished_at=1.0,
    )
    hub._archive_job(job)
    entry = hub.get_history_entry("abc123dead")
    assert entry is not None
    assert entry["curves"]["navigate"]["latest"]["v"] == [0.0, 0.5]
    assert entry["map_tracks"]["navigate"]["task"] == "navigate"
    assert hub.get_history_entry("missing") is None

    app = __import__("benchmark.dashboard.app", fromlist=["create_app"]).create_app()
    client = app.test_client()
    # Inject into the process-wide hub used by Flask routes.
    from benchmark.dashboard import monitor as mon

    mon._HUB = hub
    res = client.get("/api/history/abc123dead")
    assert res.status_code == 200
    payload = res.get_json()
    assert payload["ok"] is True
    assert payload["job"]["job_id"] == "abc123dead"
    assert client.get("/api/history/nope").status_code == 404


def test_collect_channels_include_laser_default() -> None:
    collect = collect_options_from_config()
    by_id = {c["id"]: c for c in collect["channels"]}
    assert by_id["include_laser"]["enabled"] is True
    assert by_id["pose"]["enabled"] is True
    assert "locked" not in by_id["pose"] or not by_id["pose"].get("locked")
    assert {c["id"] for c in collect["channels"]} >= {
        "pose",
        "velocity",
        "action",
        "include_laser",
        "include_image",
        "lerobot",
    }
    tree = collect["channel_tree"]
    by_msg = {c["id"]: c for c in tree}
    assert by_msg["pose"]["msg"] == "geometry_msgs/Pose2D"
    assert by_msg["include_laser"]["msg"] == "sensor_msgs/LaserScan"
    assert {a["id"] for a in by_msg["pose"]["attrs"]} >= {"x", "y", "theta"}


def test_task_configs_include_collect() -> None:
    names = {t["id"] for t in list_task_configs()}
    assert "collect" in names
    assert "navigate" in names
