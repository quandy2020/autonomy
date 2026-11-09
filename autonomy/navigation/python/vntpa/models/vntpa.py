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

"""Single VNT-PA planner ONNX inference."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Sequence

import numpy as np

from vntpa.simulator.action import (
    Action,
    ActionPrimitive,
    decode,
    primitive,
)
from vntpa.utils.config import PlannerConfig
from vntpa.data.dataset import ContextMemory
from vntpa.engine.deployment.validator import (
    inspect,
    require_model_inputs,
    require_model_outputs,
)


@dataclass
class PlanResult:
    logits: np.ndarray
    actions: list[Action]
    primitives: list[ActionPrimitive]


class OnnxPlanner:
    """K=context_size single planner; inputs match navigation C++."""

    def __init__(
        self,
        model_path: str,
        config: Optional[PlannerConfig] = None,
        providers: Optional[Sequence[str]] = None,
    ) -> None:
        import onnxruntime as ort

        self.config = config or PlannerConfig()
        self.config.validate_config()
        info = inspect(model_path)
        require_model_inputs(info, ("kf_tokens", "kf_pose", "kf_valid", "query_pose", "goal"))
        require_model_outputs(info, ("logits",))
        self.session = ort.InferenceSession(
            model_path, providers=list(providers or self.config.providers)
        )
        self.input_names_ = {item.name for item in self.session.get_inputs()}

    def tensor(self, name: str, value: np.ndarray) -> dict:
        if name not in self.input_names_:
            return {}
        return {name: np.asarray(value, dtype=np.float32)}

    def plan(self, pose: np.ndarray, goal: np.ndarray, memory: ContextMemory) -> PlanResult:
        cfg = self.config
        tokens, poses, mask = memory.context(cfg.context_size, cfg.token_count, cfg.token_dim)
        inputs = {}
        inputs.update(self.tensor("kf_tokens", tokens))
        inputs.update(self.tensor("kf_pose", poses))
        inputs.update(self.tensor("kf_valid", mask))
        inputs.update(self.tensor("query_pose", np.asarray(pose, np.float32).reshape(1, 3)))
        inputs.update(self.tensor("goal", np.asarray(goal, np.float32).reshape(1, 2)))
        outputs = self.session.run(None, inputs)
        logits = np.asarray(outputs[0], dtype=np.float32)
        actions = decode(logits, cfg.chunk, cfg.action_count)
        primitives = [primitive(a, cfg.forward_step, cfg.turn_step_rad) for a in actions]
        return PlanResult(logits=logits, actions=actions, primitives=primitives)
