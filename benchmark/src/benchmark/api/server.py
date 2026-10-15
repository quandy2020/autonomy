# Copyright 2026 The Openbot Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0.

"""Minimal FastAPI server for remote task launch."""

from __future__ import annotations

from typing import Any, Dict, List, Optional


def create_app():  # type: ignore[no-untyped-def]
    try:
        from fastapi import FastAPI
        from pydantic import BaseModel, Field
    except ImportError as exc:  # pragma: no cover
        raise ImportError("Install with: pip install -e '.[api]'") from exc

    from benchmark.core.config import load_config, package_root
    from benchmark.evaluation.arena_evaluation import ArenaEvaluation
    from benchmark.evaluation.evaluator import Evaluator
    from benchmark.tasks.manager import run_task

    class RunRequest(BaseModel):
        config: str = "configs/default.yaml"
        task: Optional[str] = None
        mode: Optional[str] = None
        overrides: List[str] = Field(default_factory=list)

    class EvalRequest(BaseModel):
        config: str = "configs/default.yaml"
        episodes: Optional[int] = 5
        overrides: List[str] = Field(default_factory=list)

    app = FastAPI(title="autonomy-benchmark", version="0.1.0")

    @app.get("/health")
    def health() -> dict:
        return {"status": "ok"}

    @app.get("/list")
    def list_plugins() -> Dict[str, Any]:
        from benchmark.core.registry import get_registry
        from benchmark.planning.factory import list_planners
        from benchmark.tasks.modes import list_modes

        hub = get_registry()
        hub.ensure_builtins()
        return {
            "simulators": list(hub.simulators.names()),
            "tasks": list(hub.tasks.names()),
            "planners": list_planners(),
            "modes": list_modes(),
        }

    @app.post("/run")
    def run_endpoint(req: RunRequest) -> Dict[str, Any]:
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

    @app.post("/eval")
    def eval_endpoint(req: EvalRequest) -> Dict[str, Any]:
        root = package_root()
        path = req.config if req.config.startswith("/") else str(root / req.config)
        cfg = load_config(path, overrides=list(req.overrides))
        pipe = ArenaEvaluation(cfg)
        return pipe.run(episodes=req.episodes)

    return app
