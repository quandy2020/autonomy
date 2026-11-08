"""Single VNT-PA planner ONNX inference."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Sequence

import numpy as np

from ..runtime.sim.action_adapter import Action, ActionPrimitive, decode_logits, primitive
from ..utils.config import PlannerConfig
from ..data.dataset import ContextMemory
from ..runtime.deployment.onnx_validator import inspect_onnx, require_inputs, require_outputs


@dataclass
class PlanResult:
    logits: np.ndarray
    actions: list[Action]
    primitives: list[ActionPrimitive]


class OnnxPlanner:
    """K=context_size single planner; inputs match navigation C++."""

    def __init__(self, model_path: str, config: Optional[PlannerConfig] = None,
                 providers: Optional[Sequence[str]] = None) -> None:
        import onnxruntime as ort

        self.config = config or PlannerConfig()
        self.config.validate()
        info = inspect_onnx(model_path)
        require_inputs(info, ("kf_tokens", "kf_pose", "kf_valid", "query_pose", "goal"))
        require_outputs(info, ("logits",))
        self.session = ort.InferenceSession(
            model_path, providers=list(providers or self.config.providers))
        self._input_names = {item.name for item in self.session.get_inputs()}

    def _tensor(self, name: str, value: np.ndarray) -> dict:
        if name not in self._input_names:
            return {}
        return {name: np.asarray(value, dtype=np.float32)}

    def plan(self, pose: np.ndarray, goal: np.ndarray,
             memory: ContextMemory) -> PlanResult:
        cfg = self.config
        tokens, poses, mask = memory.context(
            cfg.context_size, cfg.token_count, cfg.token_dim)
        inputs = {}
        inputs.update(self._tensor("kf_tokens", tokens))
        inputs.update(self._tensor("kf_pose", poses))
        inputs.update(self._tensor("kf_valid", mask))
        inputs.update(self._tensor("query_pose", np.asarray(pose, np.float32).reshape(1, 3)))
        inputs.update(self._tensor("goal", np.asarray(goal, np.float32).reshape(1, 2)))
        outputs = self.session.run(None, inputs)
        logits = np.asarray(outputs[0], dtype=np.float32)
        actions = decode_logits(logits, cfg.chunk, cfg.action_count)
        primitives = [primitive(a, cfg.forward_step, cfg.turn_step_rad) for a in actions]
        return PlanResult(logits=logits, actions=actions, primitives=primitives)
