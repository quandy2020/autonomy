# Copyright 2026 The Openbot Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0.

"""Simulator / task run routes."""

from __future__ import annotations

from typing import Any, Dict

from benchmark.api.schemas import GenerateWorldRequest, RunRequest


def mount(app: Any) -> None:
    @app.get("/health")
    def health() -> dict:
        return {"status": "ok"}

    @app.get("/list")
    def list_plugins() -> Dict[str, Any]:
        from benchmark.core.registry import get_registry
        from benchmark.humans import list_human_models
        from benchmark.planning.factory import list_planners
        from benchmark.tasks.modes import list_modes

        hub = get_registry()
        hub.ensure_builtins()
        return {
            "simulators": list(hub.simulators.names()),
            "tasks": list(hub.tasks.names()),
            "planners": list_planners(),
            "modes": list_modes(),
            "humans": list_human_models(),
        }

    @app.post("/run")
    def run_endpoint(req: RunRequest) -> Dict[str, Any]:
        from benchmark.core.config import load_config, package_root
        from benchmark.evaluation.evaluator import Evaluator
        from benchmark.tasks.manager import run_task

        root = package_root()
        path = req.config if req.config.startswith("/") else str(root / req.config)
        ovr = list(req.overrides)
        if req.task:
            ovr.append(f"task.name={req.task}")
        if req.mode:
            ovr.append(f"task.mode={req.mode}")
        cfg = load_config(path, overrides=ovr)
        results = run_task(cfg)
        return {"summary": Evaluator().summarize(results)}

    @app.post("/generate-world")
    def generate_world_endpoint(req: GenerateWorldRequest) -> Dict[str, Any]:
        from benchmark.worlds import generate_world
        from benchmark.worlds.generators.arena_gen import generate_from_prompt

        if req.prompt:
            return generate_from_prompt(req.prompt, seed=req.seed)
        if not req.name:
            return {"error": "provide name or prompt"}
        return generate_world(req.name, seed=req.seed)
