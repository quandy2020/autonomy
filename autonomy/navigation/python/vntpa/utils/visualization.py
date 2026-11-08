"""Depth and action visualization helpers."""

from __future__ import annotations

from pathlib import Path

import numpy as np

from ..runtime.sim.action_adapter import Action


def depth_uint8(depth: np.ndarray, max_depth: float = 10.0) -> np.ndarray:
    """Metric depth -> (1,H,W) uint8 image for TensorBoard/PIL."""
    depth = np.nan_to_num(np.asarray(depth, dtype=np.float32), nan=0.0, posinf=0.0, neginf=0.0)
    normalized = np.clip(depth / max(max_depth, 1e-6), 0.0, 1.0)
    return (normalized[None, ...] * 255.0).astype(np.uint8)


def save_depth_png(path: str, depth: np.ndarray, max_depth: float = 10.0) -> None:
    """Save a grayscale depth preview; requires Pillow."""
    from PIL import Image

    image = Image.fromarray(depth_uint8(depth, max_depth)[0], mode="L")
    destination = Path(path)
    destination.parent.mkdir(parents=True, exist_ok=True)
    image.save(destination)


def log_depth(writer, tag: str, depth: np.ndarray, step: int, max_depth: float = 10.0) -> None:
    """Write a depth image to a TensorBoard ``SummaryWriter``."""
    writer.add_image(tag, depth_uint8(depth, max_depth), step)


def action_arrow(action: Action) -> str:
    return {Action.STOP: "■", Action.FORWARD: "↑",
            Action.TURN_LEFT: "↰", Action.TURN_RIGHT: "↱"}[action]
