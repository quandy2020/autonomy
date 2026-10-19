# Copyright 2026 The Openbot Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0.

"""Flask web UI for benchmark monitoring (Jinja pages + JSON/SSE APIs)."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

from benchmark.dashboard.catalog import (
    catalog_payload,
    list_config_files,
    read_config_file,
)
from benchmark.dashboard.monitor import event_stream, get_hub


def create_app():  # type: ignore[no-untyped-def]
    """Build Flask webpage app bound to the ``benchmark`` package."""
    try:
        from flask import Flask, Response, jsonify, render_template, request

        import benchmark  # noqa: F401 — dashboard is not standalone
    except ImportError as exc:  # pragma: no cover
        raise ImportError(
            "benchmark.dashboard requires the autonomy-benchmark package and Flask. "
            "Install with: pip install -e '.[dashboard]'"
        ) from exc

    root = Path(__file__).resolve().parent
    app = Flask(
        "benchmark.dashboard",
        template_folder=str(root / "templates"),
        static_folder=str(root / "static"),
        static_url_path="/static",
    )
    app.config["JSON_SORT_KEYS"] = False

    @app.get("/")
    def index():  # type: ignore[no-untyped-def]
        """Jinja webpage: missions / config / collect / live curves."""
        config = request.args.get("config")
        catalog = catalog_payload(config)
        return render_template(
            "index.html",
            catalog=catalog,
            catalog_json=json.dumps(catalog, ensure_ascii=False, default=str),
            page_title="Benchmark Monitor",
        )

    @app.get("/api/catalog")
    def api_catalog():  # type: ignore[no-untyped-def]
        config = request.args.get("config")
        return jsonify(catalog_payload(config))

    @app.get("/api/datasets")
    def api_datasets():  # type: ignore[no-untyped-def]
        from benchmark.dashboard.catalog import list_datasets

        return jsonify({"datasets": list_datasets()})

    @app.get("/api/configs")
    def api_configs():  # type: ignore[no-untyped-def]
        return jsonify({"configs": list_config_files()})

    @app.get("/api/config")
    def api_config():  # type: ignore[no-untyped-def]
        rel = request.args.get("path", "configs/default.yaml")
        try:
            return jsonify(read_config_file(rel))
        except Exception as exc:  # noqa: BLE001
            return jsonify({"error": str(exc)}), 400

    @app.get("/api/status")
    def api_status():  # type: ignore[no-untyped-def]
        return jsonify(get_hub().snapshot())

    @app.get("/api/history")
    def api_history():  # type: ignore[no-untyped-def]
        return jsonify({"history": get_hub().snapshot().get("history") or []})

    @app.get("/api/history/<job_id>")
    def api_history_entry(job_id: str):  # type: ignore[no-untyped-def]
        entry = get_hub().get_history_entry(job_id)
        if entry is None:
            return jsonify({"ok": False, "error": "job not found"}), 404
        return jsonify({"ok": True, "job": entry})

    @app.get("/api/events")
    def api_events():  # type: ignore[no-untyped-def]
        hub = get_hub()
        return Response(
            event_stream(hub),
            mimetype="text/event-stream",
            headers={
                "Cache-Control": "no-cache",
                "X-Accel-Buffering": "no",
                "Connection": "keep-alive",
            },
        )

    @app.post("/api/run")
    def api_run():  # type: ignore[no-untyped-def]
        body: dict[str, Any] = request.get_json(silent=True) or {}
        tasks = body.get("tasks") or []
        if isinstance(tasks, str):
            tasks = [tasks]
        config_path: str | None = body.get("config")
        mode: str | None = body.get("mode")
        episodes = body.get("episodes")
        collect = body.get("collect") or {}
        planner: str | None = body.get("planner")
        controller: str | None = body.get("controller")
        model_path: str | None = body.get("model_path")
        simulator: str | None = body.get("simulator")
        humans: str | None = body.get("humans")
        record = bool(body.get("record"))
        raw_params = body.get("params")
        params = raw_params if isinstance(raw_params, dict) else {}
        raw_replay = body.get("replay")
        replay = raw_replay if isinstance(raw_replay, dict) else {}
        execution = str(body.get("execution") or "sync")
        kind = str(body.get("kind") or "orchestration")
        raw_named = body.get("named_calls")
        named_calls = raw_named if isinstance(raw_named, list) else []
        raw_schedule = body.get("schedule")
        schedule = raw_schedule if isinstance(raw_schedule, dict) else {}
        try:
            job = get_hub().start_batch(
                tasks=[str(t) for t in tasks],
                config_path=config_path,
                mode=mode,
                episodes=int(episodes) if episodes is not None else None,
                collect=collect,
                planner=planner,
                controller=controller,
                model_path=model_path,
                simulator=simulator,
                humans=humans,
                record=record,
                params=params,
                replay=replay,
                execution=execution,
                kind=kind,
                named_calls=named_calls,
                schedule=schedule,
            )
            return jsonify(
                {
                    "ok": True,
                    "job": {
                        "job_id": job.job_id,
                        "status": job.status,
                        "tasks": job.tasks,
                    },
                }
            )
        except Exception as exc:  # noqa: BLE001
            return jsonify({"ok": False, "error": str(exc)}), 409

    @app.post("/api/cancel")
    def api_cancel():  # type: ignore[no-untyped-def]
        ok = get_hub().cancel()
        return jsonify({"ok": ok})

    @app.get("/api/curves")
    def api_curves():  # type: ignore[no-untyped-def]
        snap = get_hub().snapshot()
        job = snap.get("job") or {}
        curves = job.get("curves") or {}
        task = request.args.get("task")
        if task:
            return jsonify({"task": task, "curves": curves.get(task) or {}})
        return jsonify({"curves": curves, "tasks": list(curves.keys())})

    @app.get("/api/export/summary")
    def api_export_summary():  # type: ignore[no-untyped-def]
        snap = get_hub().snapshot()
        job = snap.get("job") or {}
        job_id = job.get("job_id") or "job"
        payload = {
            "job_id": job.get("job_id"),
            "status": job.get("status"),
            "tasks": job.get("tasks") or [],
            "summary": job.get("summary") or {},
            "error": job.get("error") or "",
            "started_at": job.get("started_at"),
            "finished_at": job.get("finished_at"),
        }
        body = json.dumps(payload, ensure_ascii=False, indent=2, default=str)
        return Response(
            body,
            mimetype="application/json",
            headers={
                "Content-Disposition": f'attachment; filename="benchmark-{job_id}-summary.json"'
            },
        )

    @app.get("/api/export/curves")
    def api_export_curves():  # type: ignore[no-untyped-def]
        snap = get_hub().snapshot()
        job = snap.get("job") or {}
        job_id = job.get("job_id") or "job"
        task = request.args.get("task")
        curves = job.get("curves") or {}
        payload: dict[str, Any] = {
            "job_id": job.get("job_id"),
            "task": task,
            "curves": curves.get(task) if task else curves,
        }
        suffix = f"-{task}" if task else ""
        body = json.dumps(payload, ensure_ascii=False, indent=2, default=str)
        return Response(
            body,
            mimetype="application/json",
            headers={
                "Content-Disposition": (
                    f'attachment; filename="benchmark-{job_id}{suffix}-curves.json"'
                )
            },
        )

    @app.get("/api/map")
    def api_map():  # type: ignore[no-untyped-def]
        """BicMap tracks from the latest job (path / pose / goal)."""
        from benchmark.dashboard.bicmap_scenes import bicmap_payload

        snap = get_hub().snapshot()
        job = snap.get("job") or {}
        tracks = job.get("map_tracks") or {}
        task = request.args.get("task")
        payload = {
            "bicmap": bicmap_payload(),
            "job_id": (job or {}).get("job_id"),
            "status": (job or {}).get("status"),
            "tracks": tracks,
        }
        if task:
            payload["track"] = tracks.get(task) or {}
        return jsonify(payload)

    @app.get("/api/map/goal")
    def api_map_goal():  # type: ignore[no-untyped-def]
        """Last Goal Pose recorded from the map (meters), if any."""
        from benchmark.dashboard.map_control import get_pending_map_goal

        goal = get_pending_map_goal()
        return jsonify({"ok": True, "goal": goal})

    @app.post("/api/map/navigate")
    def api_map_navigate():  # type: ignore[no-untyped-def]
        """Send a single Navigate waypoint from map click (meters in map frame)."""
        from benchmark.dashboard.map_control import send_navigate, store_pending_map_goal
        from benchmark.dashboard.ota_client import OtaClientError

        body: dict[str, Any] = request.get_json(silent=True) or {}
        try:
            x = float(body["x"])
            y = float(body["y"])
        except (KeyError, TypeError, ValueError):
            return jsonify({"ok": False, "error": "x/y required (meters)"}), 400
        yaw = float(body.get("yaw") or 0.0)
        target = str(body.get("target") or "").strip() or None
        try:
            out = send_navigate(x, y, yaw, target=target)
            return jsonify(out)
        except OtaClientError as exc:
            stored = store_pending_map_goal(x, y, yaw)
            return jsonify(
                {
                    "ok": False,
                    "published": False,
                    "stored": True,
                    "goal": stored,
                    "error": str(exc),
                    "x": x,
                    "y": y,
                    "yaw": yaw,
                }
            ), 502

    @app.post("/api/map/teleop")
    def api_map_teleop():  # type: ignore[no-untyped-def]
        """TeleopService.Velocity: action=start|twist|stop|cancel."""
        from benchmark.dashboard.map_control import send_teleop
        from benchmark.dashboard.ota_client import OtaClientError

        body: dict[str, Any] = request.get_json(silent=True) or {}
        action = str(body.get("action") or "twist").strip().lower()
        target = str(body.get("target") or "").strip() or None
        try:
            out = send_teleop(
                action,
                linear_x=float(body.get("linear_x") or body.get("vx") or 0.0),
                angular_z=float(body.get("angular_z") or body.get("wz") or 0.0),
                target=target,
            )
            return jsonify(out)
        except OtaClientError as exc:
            return jsonify({"ok": False, "error": str(exc), "action": action}), 502

    def _ota_client():  # type: ignore[no-untyped-def]
        from benchmark.dashboard.ota_client import get_client

        target = request.args.get("target") or (request.get_json(silent=True) or {}).get(
            "target"
        )
        return get_client(str(target) if target else None)

    @app.get("/api/rpc/link")
    def api_rpc_link():  # type: ignore[no-untyped-def]
        """Live gRPC link status for the command / OTA target."""
        from benchmark.dashboard.rpc_client import RpcClient

        target = request.args.get("target") or None
        try:
            timeout = float(request.args.get("timeout") or 2.5)
        except (TypeError, ValueError):
            timeout = 2.5
        client = RpcClient(target=target, timeout_s=max(1.0, timeout))
        return jsonify(client.check_link(timeout_s=max(1.0, timeout)))

    @app.get("/api/ota/probe")
    def api_ota_probe():  # type: ignore[no-untyped-def]
        from benchmark.dashboard.ota_client import get_client

        return jsonify(get_client(request.args.get("target")).probe())

    @app.get("/api/ota/snapshot")
    def api_ota_snapshot():  # type: ignore[no-untyped-def]
        return jsonify(_ota_client().snapshot())

    @app.get("/api/ota/status")
    def api_ota_status():  # type: ignore[no-untyped-def]
        from benchmark.dashboard.ota_client import OtaClientError, enrich_ota_status

        try:
            raw = _ota_client().get_ota_status()
            return jsonify({"ok": True, "ota": enrich_ota_status(raw)})
        except OtaClientError as exc:
            return jsonify({"ok": False, "error": str(exc)}), 502

    @app.post("/api/ota/start")
    def api_ota_start():  # type: ignore[no-untyped-def]
        from benchmark.dashboard.ota_client import OtaClientError, enrich_ota_status

        body: dict[str, Any] = request.get_json(silent=True) or {}
        uri = str(body.get("package_uri") or "").strip()
        if not uri:
            return jsonify({"ok": False, "error": "package_uri required"}), 400
        try:
            raw = _ota_client().start_ota(
                uri,
                package_type=str(body.get("package_type") or ""),
                prefer_delta=bool(body.get("prefer_delta")),
            )
            return jsonify({"ok": True, "result": enrich_ota_status(raw)})
        except OtaClientError as exc:
            return jsonify({"ok": False, "error": str(exc)}), 502

    @app.post("/api/ota/abort")
    def api_ota_abort():  # type: ignore[no-untyped-def]
        from benchmark.dashboard.ota_client import OtaClientError, enrich_ota_status

        body: dict[str, Any] = request.get_json(silent=True) or {}
        try:
            raw = _ota_client().abort_ota(str(body.get("reason") or "operator"))
            return jsonify({"ok": True, "result": enrich_ota_status(raw)})
        except OtaClientError as exc:
            return jsonify({"ok": False, "error": str(exc)}), 502

    @app.post("/api/ota/call")
    def api_ota_call():  # type: ignore[no-untyped-def]
        """Generic SystemService / other RPC passthrough (JSON body)."""
        from benchmark.dashboard.ota_client import OtaClientError

        body: dict[str, Any] = request.get_json(silent=True) or {}
        method = str(body.get("method") or "").strip()
        if not method:
            return jsonify({"ok": False, "error": "method required"}), 400
        data = body.get("data")
        if data is not None and not isinstance(data, dict):
            return jsonify({"ok": False, "error": "data must be object"}), 400
        timeout = body.get("timeout_s")
        try:
            result = _ota_client().call(
                method,
                data if isinstance(data, dict) else {},
                timeout_s=float(timeout) if timeout is not None else None,
            )
            return jsonify({"ok": True, "result": result})
        except OtaClientError as exc:
            return jsonify({"ok": False, "error": str(exc)}), 502

    return app
