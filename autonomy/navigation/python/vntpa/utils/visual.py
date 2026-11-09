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

"""Depth and action visualization helpers."""

from __future__ import annotations

from pathlib import Path

import numpy as np

from vntpa.simulator.action import Action


def convert(depth: np.ndarray, max_depth: float = 10.0) -> np.ndarray:
    """Metric depth -> (1,H,W) uint8 image for TensorBoard/PIL."""
    depth = np.nan_to_num(np.asarray(depth, dtype=np.float32), nan=0.0, posinf=0.0, neginf=0.0)
    normalized = np.clip(depth / max(max_depth, 1e-6), 0.0, 1.0)
    return (normalized[None, ...] * 255.0).astype(np.uint8)


def save(path: str, depth: np.ndarray, max_depth: float = 10.0) -> None:
    """Save a grayscale depth preview; requires Pillow."""
    from PIL import Image

    image = Image.fromarray(convert(depth, max_depth)[0], mode="L")
    destination = Path(path)
    destination.parent.mkdir(parents=True, exist_ok=True)
    image.save(destination)


def log(writer, tag: str, depth: np.ndarray, step: int, max_depth: float = 10.0) -> None:
    """Write a depth image to a TensorBoard ``SummaryWriter``."""
    writer.add_image(tag, convert(depth, max_depth), step)


def symbol(action: Action) -> str:
    return {Action.STOP: "■", Action.FORWARD: "↑", Action.TURN_LEFT: "↰", Action.TURN_RIGHT: "↱"}[
        action
    ]
