# Copyright 2026 The Navigation Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""VNT model facade."""

from __future__ import annotations

import subprocess
import sys
from typing import Optional, Sequence

import numpy as np

from vntpa.data.dataset import ContextMemory
from vntpa.engine.deployment.validator import manifest
from vntpa.engine.metric import BenchmarkReport, benchmark
from vntpa.engine.predictor import Predictor
from vntpa.engine.result import VNTResult
from vntpa.engine.trainer import TrainConfig, ValConfig, train, validate
from vntpa.models.encoder import OnnxDefmEncoder
from vntpa.models.vntpa import OnnxPlanner
from vntpa.utils.config import DeployConfig, PlannerConfig, configure, resolve


class VNT:
    """Small YOLO-like facade; all heavy work stays in Predictor and training functions."""

    def __init__(
        self,
        deploy: Optional[DeployConfig] = None,
        *,
        planner: Optional[OnnxPlanner] = None,
        defm: Optional[OnnxDefmEncoder] = None,
    ) -> None:
        self.deploy = deploy or DeployConfig()
        self.predictor = Predictor(self.deploy, planner=planner, defm=defm)

    @classmethod
    def load(cls, planner: str, defm: str, config: Optional[PlannerConfig] = None) -> "VNT":
        planner_config = config or PlannerConfig()
        return cls(
            DeployConfig(
                planner_onnx=planner,
                defm_onnx=defm,
                planner=planner_config,
                providers=planner_config.providers,
            )
        )

    @classmethod
    def configure(cls, path: str) -> "VNT":
        return cls(configure(path).deploy)

    @classmethod
    def open(cls, name_or_path: str) -> "VNT":
        spec = resolve(name_or_path)
        return cls.load(spec.planner_onnx, spec.defm_onnx)

    @property
    def memory(self) -> ContextMemory:
        return self.predictor.memory

    def reset(self) -> None:
        self.predictor.reset()

    def predict(self, depth: np.ndarray, pose: np.ndarray, goal: np.ndarray) -> VNTResult:
        return self.predictor.predict(depth, pose, goal)

    def benchmark(
        self,
        depth: np.ndarray,
        pose: np.ndarray,
        goal: np.ndarray,
        *,
        warmup: int = 3,
        runs: int = 20,
    ) -> BenchmarkReport:
        return benchmark(self.predictor, depth, pose, goal, warmup=warmup, runs=runs)

    @staticmethod
    def export(checkpoint: str, out: str, *, context_size: int = 64, format: str = "onnx") -> int:
        if format != "onnx":
            raise ValueError(f"unsupported export format {format!r}; supported: ['onnx']")
        command = [
            sys.executable,
            "-m",
            "vntpa.engine.deployment.exporter",
            "--checkpoint",
            checkpoint,
            "--out",
            out,
            "--context-size",
            str(context_size),
        ]
        code = subprocess.run(command, check=False).returncode
        if code == 0:
            try:
                manifest(out, PlannerConfig(context_size=context_size))
            except Exception:  # noqa: BLE001 - metadata is best-effort
                pass
        return code

    @staticmethod
    def train(
        args: Sequence[str],
        *,
        nproc: int = 1,
        start_hooks: Sequence = (),
        end_hooks: Sequence = (),
    ) -> int:
        return train(
            TrainConfig(list(args), nproc),
            start_hooks=start_hooks,
            end_hooks=end_hooks,
        )

    @staticmethod
    def validate(args: Sequence[str]) -> int:
        return validate(ValConfig(list(args)))
