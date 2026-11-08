"""Convert keyframe memory into fixed-size context tensors."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from ..data.dataset import ContextMemory


@dataclass
class ContextTensors:
    tokens: np.ndarray
    poses: np.ndarray
    valid: np.ndarray


def build_context(memory: ContextMemory, context_size: int,
                  token_count: int, token_dim: int) -> ContextTensors:
    """Return padded tokens ``[K,P,D]``, poses ``[K,3]`` and a boolean mask."""
    tokens, poses, mask = memory.context(context_size, token_count, token_dim)
    return ContextTensors(tokens, poses, mask.astype(bool))
