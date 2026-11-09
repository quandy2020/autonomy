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

"""FrontierNet head decoding: distance field + class logits -> frontier map.

Equations (6) and (7) of the FrontierNet paper:

* the distance field ``D`` is denormalized with ``exp(-d) * normalizer`` and
  thresholded to obtain the frontier region ``F``;
* the class logits ``Y`` are arg-maxed, mapped to the bin centre of the
  log1p gain bins and exponentiated back to ``m^3`` to obtain the gain ``G``.

Everything here is numpy-only so the runtime side can reuse it without torch.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

__all__ = [
    "FrontierMap",
    "cls2gainmap",
    "cls_bin_edges",
    "denormalize_df",
    "df2region",
    "logits_to_labels",
    "prediction2frontiermap",
    "region_iou",
]


def cls_bin_edges(n_classes: int = 11) -> np.ndarray:
    """Fixed log1p gain bin edges from the official implementation."""
    if n_classes == 11:
        edges = [1, 50, 150, 300, 450, 750, 1000, 1500, 2000, 2500, 3500]
    elif n_classes == 7:
        edges = [1, 100, 300, 500, 1000, 2000, 3500]
    else:
        raise ValueError(f"unsupported class count {n_classes}; expected 7 or 11")
    return np.log1p(np.asarray(edges, dtype=np.float64))


def denormalize_df(df: np.ndarray, normalizer: float = 10.0) -> np.ndarray:
    """Inverse of ``-log(d / normalizer)``; returns metres."""
    return np.exp(-np.asarray(df, dtype=np.float64)) * float(normalizer)


def df2region(
    df: np.ndarray, normalizer: float = 10.0, threshold: float = 0.5
) -> np.ndarray:
    """Binary frontier region mask from the predicted distance field."""
    metres = denormalize_df(df, normalizer)
    region = np.zeros(metres.shape, dtype=np.uint8)
    region[metres < float(threshold)] = 1
    return region


def logits_to_labels(logits: np.ndarray) -> np.ndarray:
    """Arg-max class labels from ``(C, H, W)`` or ``(1, C, H, W)`` logits."""
    array = np.asarray(logits)
    if array.ndim == 4:
        array = array[0]
    if array.ndim != 3:
        raise ValueError(f"expected (C, H, W) logits, got shape {array.shape}")
    return np.argmax(array, axis=0).astype(np.int16)


def cls2gainmap(labels: np.ndarray, n_classes: int = 11) -> np.ndarray:
    """Map class labels to the bin-centre gain in ``m^3`` (label 0 -> 0)."""
    edges = cls_bin_edges(n_classes)
    centres = 0.5 * (edges[:-1] + edges[1:])
    label_array = np.asarray(labels)
    gain = np.zeros(label_array.shape, dtype=np.float32)
    for index in range(1, edges.size):
        gain[label_array == index] = np.expm1(centres[index - 1])
    return gain


@dataclass
class FrontierMap:
    """Decoded frontier prediction for one frame.

    ``region``/``gain`` follow the paper; ``logits`` and ``df`` are kept so a
    DeFM run can be compared against the official checkpoint without rerunning
    either model.
    """

    region: np.ndarray
    gain: np.ndarray
    labels: np.ndarray
    df: np.ndarray
    logits: np.ndarray

    @property
    def shape(self) -> tuple:
        return tuple(self.region.shape)

    def save(self, path: str) -> None:
        np.savez_compressed(
            path,
            region=self.region,
            gain=self.gain,
            labels=self.labels,
            df=self.df,
            logits=self.logits,
        )

    @classmethod
    def load(cls, path: str) -> "FrontierMap":
        with np.load(path) as data:
            return cls(
                region=data["region"],
                gain=data["gain"],
                labels=data["labels"],
                df=data["df"],
                logits=data["logits"],
            )

    def summary(self) -> dict:
        region = self.region.astype(bool)
        return {
            "shape": list(self.shape),
            "frontier_pixels": int(region.sum()),
            "frontier_ratio": float(region.mean()) if region.size else 0.0,
            "gain_sum": float(self.gain.sum()),
            "gain_max": float(self.gain.max()) if self.gain.size else 0.0,
            "df_mean": float(self.df.mean()),
            "df_min": float(self.df.min()),
            "df_max": float(self.df.max()),
            "label_histogram": np.bincount(
                self.labels.reshape(-1), minlength=int(self.logits.shape[0])
            ).tolist(),
        }


def prediction2frontiermap(
    df: np.ndarray,
    logits: np.ndarray,
    n_classes: int = 11,
    df_normalizer: float = 10.0,
    threshold: float = 0.5,
) -> FrontierMap:
    """Combine both heads into a :class:`FrontierMap`."""
    df_array = np.asarray(df, dtype=np.float32)
    if df_array.ndim == 4:
        df_array = df_array[0]
    if df_array.ndim == 3:
        df_array = df_array[0]
    logits_array = np.asarray(logits, dtype=np.float32)
    if logits_array.ndim == 4:
        logits_array = logits_array[0]
    region = df2region(df_array, df_normalizer, threshold)
    labels = logits_to_labels(logits_array)
    gain_map = cls2gainmap(labels, n_classes)
    gain = np.zeros_like(gain_map)
    gain[region > 0] = gain_map[region > 0]
    return FrontierMap(
        region=region,
        gain=gain,
        labels=labels,
        df=df_array,
        logits=logits_array,
    )


def region_iou(left: np.ndarray, right: np.ndarray) -> float:
    """IoU of two binary region masks; used by the alignment baseline."""
    a = np.asarray(left).astype(bool)
    b = np.asarray(right).astype(bool)
    union = np.logical_or(a, b).sum()
    if union == 0:
        return 1.0
    return float(np.logical_and(a, b).sum() / union)

