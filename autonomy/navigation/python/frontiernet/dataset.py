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

"""Dataset plumbing for retraining the DeFM FrontierNet heads.

Two directories make up a training set:

``frames/``
    ``<name>.npz`` written by :mod:`frontiernet.collect`, holding ``depth``
    (metres, ``float32``), ``rgb`` (``uint8``), ``K`` and ``stamp``.
``labels/``
    ``<name>.npz`` written by :mod:`frontiernet.label`, holding the teacher
    ``df`` (``float16``) and the arg-maxed ``labels`` (``uint8``).

The dataset only needs ``depth`` at training time; ``rgb`` is used once, when
the teacher labels are generated. That keeps the deployed model depth-only.
"""

from __future__ import annotations

import os
from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np
import torch
from torch.utils.data import Dataset

from frontiernet.preprocess import defm_preprocess

__all__ = [
    "FrontierFrameDataset",
    "FrameIndex",
    "index_frames",
    "split_indices",
]


@dataclass(frozen=True)
class FrameIndex:
    """One captured frame and, when available, its teacher label file."""

    name: str
    frame_path: str
    label_path: Optional[str]

    @property
    def labelled(self) -> bool:
        return self.label_path is not None


def index_frames(frames_dir: str, labels_dir: Optional[str] = None) -> List[FrameIndex]:
    """Pair every ``frames/*.npz`` with ``labels/<same name>.npz`` when present."""
    if not os.path.isdir(frames_dir):
        raise FileNotFoundError(f"frames directory not found: {frames_dir}")
    names = sorted(
        os.path.splitext(entry)[0]
        for entry in os.listdir(frames_dir)
        if entry.endswith(".npz")
    )
    if not names:
        raise FileNotFoundError(f"no .npz frames in {frames_dir}")
    entries: List[FrameIndex] = []
    for name in names:
        label_path = None
        if labels_dir:
            candidate = os.path.join(labels_dir, f"{name}.npz")
            if os.path.isfile(candidate):
                label_path = candidate
        entries.append(
            FrameIndex(
                name=name,
                frame_path=os.path.join(frames_dir, f"{name}.npz"),
                label_path=label_path,
            )
        )
    return entries


def split_indices(
    count: int, val_fraction: float = 0.15, seed: int = 0
) -> Tuple[List[int], List[int]]:
    """Deterministic train/val split that always leaves at least one val frame."""
    if count <= 0:
        raise ValueError("count must be positive")
    if not 0.0 <= float(val_fraction) < 1.0:
        raise ValueError("val_fraction must be in [0, 1)")
    if count == 1 or val_fraction == 0.0:
        return list(range(count)), []
    val_count = max(1, int(round(count * float(val_fraction))))
    val_count = min(val_count, count - 1)
    generator = np.random.default_rng(int(seed))
    order = generator.permutation(count)
    val = sorted(int(index) for index in order[:val_count])
    train = sorted(int(index) for index in order[val_count:])
    return train, val


class FrontierFrameDataset(Dataset):
    """Depth frames plus teacher distance-field / class targets.

    ``flip`` enables a horizontal flip that is applied to the input *and* both
    targets, which is the only augmentation that is exactly label preserving
    for a camera-view distance field.
    """

    def __init__(
        self,
        frames_dir: str,
        labels_dir: Optional[str] = None,
        indices: Optional[Sequence[int]] = None,
        input_size: int = 224,
        flip: bool = False,
        seed: int = 0,
        require_labels: bool = True,
    ) -> None:
        self.entries = index_frames(frames_dir, labels_dir)
        if require_labels:
            missing = [entry.name for entry in self.entries if not entry.labelled]
            if missing:
                raise FileNotFoundError(
                    f"{len(missing)} frames have no teacher label (first: {missing[0]}); "
                    "run `python -m frontiernet.label` first"
                )
        if indices is not None:
            self.entries = [self.entries[int(index)] for index in indices]
        if not self.entries:
            raise ValueError("dataset is empty after applying the index selection")
        self.input_size = int(input_size)
        self.flip = bool(flip)
        self.seed = int(seed)

    def __len__(self) -> int:
        return len(self.entries)

    def label_histogram(self, classes: int = 11) -> np.ndarray:
        """Class histogram over the whole dataset (used for loss weighting)."""
        histogram = np.zeros(int(classes), dtype=np.int64)
        for entry in self.entries:
            if not entry.labelled:
                continue
            with np.load(entry.label_path) as payload:
                labels = np.asarray(payload["labels"]).reshape(-1)
            histogram += np.bincount(labels, minlength=int(classes))[: int(classes)]
        return histogram

    def __getitem__(self, index: int) -> Dict[str, torch.Tensor]:
        entry = self.entries[int(index)]
        with np.load(entry.frame_path) as payload:
            depth = np.asarray(payload["depth"], dtype=np.float32)
        inputs = defm_preprocess(depth, self.input_size)

        if entry.labelled:
            with np.load(entry.label_path) as payload:
                df = np.asarray(payload["df"], dtype=np.float32)
                labels = np.asarray(payload["labels"], dtype=np.int64)
            if df.shape[-2:] != (self.input_size, self.input_size):
                raise ValueError(
                    f"{entry.name}: label size {df.shape[-2:]} does not match "
                    f"input size {self.input_size}"
                )
            df = df[None, ...]
        else:
            df = np.zeros((1, self.input_size, self.input_size), dtype=np.float32)
            labels = np.zeros((self.input_size, self.input_size), dtype=np.int64)

        if self.flip and np.random.default_rng(self.seed + int(index)).random() < 0.5:
            inputs = inputs[:, :, ::-1].copy()
            df = df[:, :, ::-1].copy()
            labels = labels[:, ::-1].copy()

        return {
            "input": torch.from_numpy(np.ascontiguousarray(inputs)),
            "df": torch.from_numpy(np.ascontiguousarray(df)),
            "label": torch.from_numpy(np.ascontiguousarray(labels)),
            "name": entry.name,
        }
