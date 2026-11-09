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

"""DeFM encoder wrappers for ONNX deployment and PyTorch training."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Sequence

import numpy as np

from vntpa.data.transform import preprocess
from vntpa.engine.deployment.validator import (
    inspect,
    require_model_inputs,
    require_model_outputs,
)


@dataclass
class DefmOutput:
    tokens: np.ndarray  # (P, D)
    pooled: np.ndarray  # (D,)


class OnnxDefmEncoder:
    """Frozen DeFM ONNX encoder with C++-equivalent preprocessing."""

    def __init__(
        self,
        model_path: str,
        providers: Optional[Sequence[str]] = None,
        *,
        input_size: int = 224,
        input_name: Optional[str] = None,
        token_name: str = "tokens",
        pooled_name: str = "pooled",
    ) -> None:
        import onnxruntime as ort

        info = inspect(model_path)
        self.input_name = input_name or next(iter(info.inputs))
        require_model_inputs(info, (self.input_name,))
        require_model_outputs(info, (token_name, pooled_name))
        self.session = ort.InferenceSession(
            model_path, providers=list(providers or ("CPUExecutionProvider",))
        )
        self.token_name = token_name
        self.pooled_name = pooled_name
        self.input_size = input_size

    def encode(self, depth: np.ndarray) -> DefmOutput:
        tensor = preprocess(depth, self.input_size)
        tokens, pooled = self.session.run(
            [self.token_name, self.pooled_name], {self.input_name: tensor}
        )
        tokens = np.asarray(tokens, dtype=np.float32)
        if tokens.ndim == 3:
            tokens = tokens[0]
        pooled = np.asarray(pooled, dtype=np.float32).reshape(-1)
        return DefmOutput(tokens=tokens, pooled=pooled)


class TorchDefmEncoder:
    """Training/eval adapter over pan.util.defm_engine.DeFMEncoder."""

    def __init__(
        self, device: str = "cuda", amp_dtype=None, weights_path: Optional[str] = None
    ) -> None:
        import torch
        from thirdparty.pan.util.defm_engine import DeFMEncoder

        self.torch_ = torch
        self.encoder_ = DeFMEncoder(
            torch.device(device), amp_dtype=amp_dtype or torch.bfloat16, weights_path=weights_path
        )

    def encode(self, depth: np.ndarray) -> DefmOutput:
        tokens = self.encoder_.encode(np.asarray(depth, dtype=np.float32))
        tokens = tokens.detach().float().cpu().numpy()
        if tokens.ndim == 3:
            tokens = tokens[0]
        return DefmOutput(tokens=tokens, pooled=tokens.mean(axis=0))
