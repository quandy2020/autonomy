# Copyright 2026 The Openbot Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0.

"""Evaluation routes."""

from __future__ import annotations

from typing import Any, Dict

from benchmark.api.schemas import EvalRequest


def mount(app: Any) -> None:
    @app.post("/eval")
    def eval_endpoint(req: EvalRequest) -> Dict[str, Any]:
        from benchmark.core.config import load_config, package_root
        from benchmark.evaluation.arena_evaluation import ArenaEvaluation

        root = package_root()
        path = req.config if req.config.startswith("/") else str(root / req.config)
        cfg = load_config(path, overrides=list(req.overrides))
        return ArenaEvaluation(cfg).run(episodes=req.episodes)
