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

"""DeFM + VNT-PA predictor."""

from __future__ import annotations

from typing import Optional

import numpy as np

from vntpa.data.dataset import ContextMemory
from vntpa.engine.result import VNTResult
from vntpa.models.encoder import OnnxDefmEncoder
from vntpa.models.vntpa import OnnxPlanner
from vntpa.utils.config import DeployConfig


class Predictor:
    """DeFM + VNT-PA inference with pose-indexed memory."""

    def __init__(
        self,
        deploy: DeployConfig,
        *,
        planner: Optional[OnnxPlanner] = None,
        defm: Optional[OnnxDefmEncoder] = None,
    ) -> None:
        self.planner = planner or OnnxPlanner(deploy.planner_onnx, deploy.planner, deploy.providers)
        self.defm = defm or OnnxDefmEncoder(deploy.defm_onnx, deploy.providers)
        self.memory = ContextMemory(max_keyframes=deploy.planner.context_size)
        self.step_ = 0

    def reset(self) -> None:
        self.memory.clear()
        self.step_ = 0

    def predict(self, depth: np.ndarray, pose: np.ndarray, goal: np.ndarray) -> VNTResult:
        encoded = self.defm.encode(depth)
        self.step_ += 1
        self.memory.add(encoded.tokens, pose, float(self.step_) * 0.2, encoded.pooled)
        plan = self.planner.plan(pose, goal, self.memory)
        return VNTResult(plan.logits, plan.actions, plan.primitives, len(self.memory))
