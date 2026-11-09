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

"""Single-frame inference for both model families.

``FrontierPredictor`` hides the one thing that differs between the official
model and the DeFM variant: the preprocessing contract. It picks the contract
from ``model.n_channels`` (4 = RGB-D official, 3 = DeFM depth-only) so callers
cannot accidentally feed the wrong tensor.
"""

from __future__ import annotations

from typing import Optional, Sequence

import numpy as np
import torch

from frontiernet.postprocess import FrontierMap, prediction2frontiermap
from frontiernet.preprocess import defm_preprocess, frontiernet_preprocess

__all__ = ["FrontierPredictor"]


class FrontierPredictor:
    """Run a FrontierNet-family model on one RGB-D or depth frame."""

    def __init__(
        self,
        model: torch.nn.Module,
        device: str = "cpu",
        input_size: int = 224,
        classes: int = 11,
        df_normalizer: float = 10.0,
        df_threshold: float = 0.5,
        scale_factor: float = 0.5,
    ) -> None:
        self.model = model.to(device).eval()
        self.device = torch.device(device)
        self.input_size = int(input_size)
        self.classes = int(classes)
        self.df_normalizer = float(df_normalizer)
        self.df_threshold = float(df_threshold)
        self.scale_factor = float(scale_factor)
        self.n_channels = int(getattr(model, "n_channels", 4))
        if self.n_channels not in (3, 4):
            raise ValueError(f"unsupported model input channels: {self.n_channels}")

    def preprocess(
        self, rgb: Optional[np.ndarray], depth: np.ndarray
    ) -> np.ndarray:
        if self.n_channels == 3:
            return defm_preprocess(depth, self.input_size)
        if rgb is None:
            raise ValueError("the RGB-D model needs an rgb frame")
        return frontiernet_preprocess(
            rgb, depth, scale=self.scale_factor, size=self.input_size
        )

    @torch.no_grad()
    def predict(self, rgb: Optional[np.ndarray] = None, depth: Optional[np.ndarray] = None) -> FrontierMap:
        if depth is None:
            raise ValueError("depth is required")
        tensor = self.preprocess(rgb, np.asarray(depth, dtype=np.float32))
        batch = torch.from_numpy(tensor).unsqueeze(0).to(self.device, dtype=torch.float32)
        df, logits = self.model(batch)
        return prediction2frontiermap(
            df.detach().cpu().numpy(),
            logits.detach().cpu().numpy(),
            n_classes=self.classes,
            df_normalizer=self.df_normalizer,
            threshold=self.df_threshold,
        )

    def predict_batch(self, inputs: Sequence[np.ndarray]) -> FrontierMap:
        """Run several already-preprocessed ``(C, H, W)`` frames as one batch."""
        if not inputs:
            raise ValueError("inputs must not be empty")
        batch = torch.from_numpy(np.stack(inputs)).to(self.device, dtype=torch.float32)
        with torch.no_grad():
            df, logits = self.model(batch)
        return prediction2frontiermap(
            df.detach().cpu().numpy(),
            logits.detach().cpu().numpy(),
            n_classes=self.classes,
            df_normalizer=self.df_normalizer,
            threshold=self.df_threshold,
        )
