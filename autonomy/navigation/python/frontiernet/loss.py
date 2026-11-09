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

"""Training objective for the two FrontierNet heads.

The paper optimises a distance-field regression and an 11-way gain
classification jointly. This module keeps that structure but adds the two
things a DeFM encoder swap needs:

* the regression term is *weighted* towards the frontier band, because a
  randomly initialised adapter first has to get the few frontier pixels right
  before the far field matters;
* the classification term uses inverse-frequency class weights, because the
  label histogram is dominated by ``unknown`` (class 0) and the largest gain
  bin.

Everything is expressed on the normalised distance field ``d = -log(D / 10)``
so the loss is directly comparable with :mod:`frontiernet.postprocess`.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, Optional, Sequence

import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F

__all__ = [
    "DEFAULT_FRONTIER_BAND_M",
    "FrontierLoss",
    "LossWeights",
    "class_weights_from_histogram",
    "df_band_mask",
    "df_to_metres",
    "region_iou_torch",
]

DEFAULT_FRONTIER_BAND_M = 1.0


def df_to_metres(df: torch.Tensor, normalizer: float = 10.0) -> torch.Tensor:
    """Inverse of ``-log(d / normalizer)``; clamped so ``exp`` cannot overflow."""
    return torch.exp(-df.clamp(max=30.0)) * float(normalizer)


def df_band_mask(
    df_target: torch.Tensor,
    band_m: float = DEFAULT_FRONTIER_BAND_M,
    normalizer: float = 10.0,
) -> torch.Tensor:
    """Boolean mask of targets whose distance to the frontier is below ``band_m``."""
    threshold = math.log(float(normalizer) / float(band_m))
    return df_target > threshold


def region_iou_torch(
    df_pred: torch.Tensor,
    df_target: torch.Tensor,
    threshold_m: float = 0.5,
    normalizer: float = 10.0,
) -> torch.Tensor:
    """IoU of the thresholded frontier regions over the whole batch."""
    pred = df_to_metres(df_pred, normalizer) < float(threshold_m)
    target = df_to_metres(df_target, normalizer) < float(threshold_m)
    union = (pred | target).sum().to(torch.float64)
    if float(union) == 0.0:
        return torch.tensor(1.0, dtype=torch.float64, device=df_pred.device)
    return (pred & target).sum().to(torch.float64) / union


def class_weights_from_histogram(
    histogram: Sequence[int],
    power: float = 0.5,
    clip: Sequence[float] = (0.2, 5.0),
) -> torch.Tensor:
    """Inverse-frequency weights, damped by ``power`` and clipped.

    ``power=0.5`` is the usual compromise between "ignore the majority class"
    and "chase a handful of pixels"; ``power=1.0`` is full inverse frequency.
    """
    counts = np.asarray(histogram, dtype=np.float64)
    if counts.ndim != 1 or counts.size == 0:
        raise ValueError("histogram must be a non-empty 1-D sequence")
    if (counts < 0).any():
        raise ValueError("histogram counts must be non-negative")
    total = counts.sum()
    if total <= 0:
        return torch.ones(counts.size, dtype=torch.float32)
    frequency = np.maximum(counts, 1.0) / total
    weights = np.power(frequency, -float(power))
    weights = weights / weights.mean()
    low, high = float(clip[0]), float(clip[1])
    weights = np.clip(weights, low, high)
    return torch.from_numpy(weights.astype(np.float32))


@dataclass(frozen=True)
class LossWeights:
    """Relative weights of the two heads plus the frontier emphasis."""

    df: float = 1.0
    seg: float = 1.0
    frontier_boost: float = 4.0
    smooth_beta: float = 0.1
    label_smoothing: float = 0.05
    frontier_band_m: float = DEFAULT_FRONTIER_BAND_M
    df_normalizer: float = 10.0
    region_threshold_m: float = 0.5


class FrontierLoss(nn.Module):
    """Weighted distance-field regression + gain classification.

    ``forward`` returns a dict; the differentiable entries are ``loss``,
    ``df`` and ``seg``, the rest are detached scalars for logging.
    """

    def __init__(
        self,
        weights: Optional[LossWeights] = None,
        class_weights: Optional[torch.Tensor] = None,
    ) -> None:
        super().__init__()
        self.weights = weights or LossWeights()
        if class_weights is not None:
            self.register_buffer("class_weights", class_weights.to(torch.float32))
        else:
            self.class_weights = None

    def forward(
        self,
        df_pred: torch.Tensor,
        logits_pred: torch.Tensor,
        df_target: torch.Tensor,
        label_target: torch.Tensor,
    ) -> Dict[str, torch.Tensor]:
        if df_pred.shape != df_target.shape:
            raise ValueError(
                f"df shape mismatch: pred {tuple(df_pred.shape)} vs target {tuple(df_target.shape)}"
            )
        if logits_pred.shape[0] != label_target.shape[0]:
            raise ValueError("logits and labels disagree on batch size")
        if logits_pred.shape[-2:] != label_target.shape[-2:]:
            raise ValueError("logits and labels disagree on spatial size")

        df_target = df_target.to(df_pred.dtype)
        weight = torch.ones_like(df_target)
        band = df_band_mask(
            df_target, self.weights.frontier_band_m, self.weights.df_normalizer
        )
        weight = weight + float(self.weights.frontier_boost) * band.to(df_target.dtype)
        elementwise = F.smooth_l1_loss(
            df_pred, df_target, beta=float(self.weights.smooth_beta), reduction="none"
        )
        df_loss = (elementwise * weight).sum() / weight.sum().clamp_min(1.0)

        seg_loss = F.cross_entropy(
            logits_pred,
            label_target.to(torch.long),
            weight=self.class_weights,
            label_smoothing=float(self.weights.label_smoothing),
        )

        total = float(self.weights.df) * df_loss + float(self.weights.seg) * seg_loss
        with torch.no_grad():
            metres_pred = df_to_metres(df_pred, self.weights.df_normalizer)
            metres_target = df_to_metres(df_target, self.weights.df_normalizer)
            metrics = {
                "loss": total.detach(),
                "df": df_loss.detach(),
                "seg": seg_loss.detach(),
                "df_mae_m": (metres_pred - metres_target).abs().mean(),
                "region_iou": region_iou_torch(
                    df_pred,
                    df_target,
                    self.weights.region_threshold_m,
                    self.weights.df_normalizer,
                ).to(torch.float32),
                "label_acc": (
                    logits_pred.argmax(dim=1) == label_target.to(torch.long)
                ).to(torch.float32).mean(),
                "frontier_ratio": band.to(torch.float32).mean(),
            }
        return {"loss": total, "df": df_loss, "seg": seg_loss, "metrics": metrics}
