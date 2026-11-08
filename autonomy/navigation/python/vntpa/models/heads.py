"""Output heads shared by runtime and export."""

from __future__ import annotations

import numpy as np

from ..runtime.sim.action_adapter import Action, decode_logits


def argmax_actions(logits: np.ndarray) -> list[Action]:
    """Return one action per chunk step from ``[chunk, 4]`` logits."""
    return decode_logits(np.asarray(logits, dtype=np.float32))
