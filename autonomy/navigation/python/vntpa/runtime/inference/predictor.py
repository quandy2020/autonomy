"""VNT product facade: one object for predict/train/val/export/benchmark."""

from __future__ import annotations

import json
import subprocess
import sys
from dataclasses import dataclass
from typing import List, Optional, Sequence

import numpy as np

from ..sim.action_adapter import Action, ActionPrimitive
from ...training.evaluation.metrics import Benchmark, BenchmarkReport
from ...utils.config import (
    DEFAULT_REGISTRY,
    DeployConfig,
    PlannerConfig,
    load_product_config,
)
from ...data.dataset import ContextMemory
from ...models.encoder import OnnxDefmEncoder
from ..deployment.onnx_validator import write_manifest
from ...models.vntpa import OnnxPlanner
from ...training.trainer import CallbackList, TrainConfig, Trainer, ValConfig, Validator


@dataclass
class VNTResult:
    """One prediction result."""

    logits: np.ndarray
    actions: List[Action]
    primitives: List[ActionPrimitive]
    memory_size: int = 0

    def to_dict(self) -> dict:
        return {
            "actions": [int(action) for action in self.actions],
            "primitives": [
                {"action": int(item.action), "distance": item.distance, "angle": item.angle}
                for item in self.primitives
            ],
            "memory_size": self.memory_size,
        }

    def to_json(self, indent: int = 2) -> str:
        return json.dumps(self.to_dict(), indent=indent)

    def save(self, path: str) -> None:
        from pathlib import Path

        Path(path).write_text(self.to_json(), encoding="utf-8")

    def summary(self) -> str:
        return f"actions={[action.name for action in self.actions]} memory={self.memory_size}"


class Predictor:
    """DeFM + VNT-PA inference with pose-indexed memory."""

    def __init__(self, deploy: DeployConfig, *,
                 planner: Optional[OnnxPlanner] = None,
                 defm: Optional[OnnxDefmEncoder] = None) -> None:
        self.planner = planner or OnnxPlanner(
            deploy.planner_onnx, deploy.planner, deploy.providers)
        self.defm = defm or OnnxDefmEncoder(deploy.defm_onnx, deploy.providers)
        self.memory = ContextMemory(max_keyframes=deploy.planner.context_size)
        self._step = 0

    def reset(self) -> None:
        self.memory.clear()
        self._step = 0

    def predict(self, depth: np.ndarray, pose: np.ndarray, goal: np.ndarray) -> VNTResult:
        encoded = self.defm.encode(depth)
        self._step += 1
        self.memory.add(encoded.tokens, pose, float(self._step) * 0.2, encoded.pooled)
        plan = self.planner.plan(pose, goal, self.memory)
        return VNTResult(plan.logits, plan.actions, plan.primitives, len(self.memory))


class VNT:
    """Small YOLO-like facade; all heavy work stays in Predictor/Trainer."""

    def __init__(self, deploy: Optional[DeployConfig] = None, *,
                 planner: Optional[OnnxPlanner] = None,
                 defm: Optional[OnnxDefmEncoder] = None) -> None:
        self.deploy = deploy or DeployConfig()
        self.predictor = Predictor(self.deploy, planner=planner, defm=defm)

    @classmethod
    def from_pretrained(cls, planner: str, defm: str,
                        config: Optional[PlannerConfig] = None) -> "VNT":
        planner_config = config or PlannerConfig()
        return cls(DeployConfig(
            planner_onnx=planner, defm_onnx=defm,
            planner=planner_config, providers=planner_config.providers))

    @classmethod
    def from_config(cls, path: str) -> "VNT":
        return cls(load_product_config(path).deploy)

    @classmethod
    def load(cls, name_or_path: str) -> "VNT":
        spec = DEFAULT_REGISTRY.resolve(name_or_path)
        return cls.from_pretrained(spec.planner_onnx, spec.defm_onnx)

    @property
    def memory(self) -> ContextMemory:
        return self.predictor.memory

    def reset(self) -> None:
        self.predictor.reset()

    def predict(self, depth: np.ndarray, pose: np.ndarray, goal: np.ndarray) -> VNTResult:
        return self.predictor.predict(depth, pose, goal)

    def benchmark(self, depth: np.ndarray, pose: np.ndarray, goal: np.ndarray,
                  *, warmup: int = 3, runs: int = 20) -> BenchmarkReport:
        return Benchmark(self.predictor, depth, pose, goal,
                         warmup=warmup, runs=runs).run()

    @staticmethod
    def export(checkpoint: str, out: str, *, context_size: int = 64,
               format: str = "onnx") -> int:
        if format != "onnx":
            raise ValueError(f"unsupported export format {format!r}; supported: ['onnx']")
        command = [
            sys.executable, "-m", "vntpa.runtime.deployment.onnx_exporter",
            "--checkpoint", checkpoint, "--out", out,
            "--context-size", str(context_size),
        ]
        code = subprocess.run(command, check=False).returncode
        if code == 0:
            try:
                write_manifest(out, PlannerConfig(context_size=context_size))
            except Exception:  # noqa: BLE001 - metadata is best-effort
                pass
        return code

    @staticmethod
    def train(args: Sequence[str], *, nproc: int = 1,
              callbacks: Optional[CallbackList] = None) -> int:
        return Trainer(TrainConfig(list(args), nproc), callbacks).train()

    @staticmethod
    def val(args: Sequence[str]) -> int:
        return Validator(ValConfig(list(args))).validate()
