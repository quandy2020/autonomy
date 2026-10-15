# Copyright 2026 The Openbot Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0.

"""Training routes."""

from __future__ import annotations

from typing import Any, Dict

from benchmark.api.schemas import TrainRequest


def mount(app: Any) -> None:
    @app.post("/train")
    def train_endpoint(req: TrainRequest) -> Dict[str, Any]:
        from pathlib import Path

        from benchmark.rl.deployment.export_onnx import export_linear_policy
        from benchmark.rl.env import NavEnv
        from benchmark.rl.training.trainer import SimpleTrainer

        env = NavEnv(
            {
                "mode": "parametrized",
                "max_steps": 120,
                "mode_config": {"difficulty": 0.4},
            }
        )
        trainer = SimpleTrainer(
            env,
            {"curriculum": req.curriculum, "episodes": req.episodes, "seed": req.seed},
        )
        summary = trainer.train(total_episodes=req.episodes)
        meta = None
        if req.export:
            meta = export_linear_policy(
                trainer.best_weight, trainer.best_action_scale, Path(req.export)
            )
        env.close()
        return {"summary": summary, "export": meta}
