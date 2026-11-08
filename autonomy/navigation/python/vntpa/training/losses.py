"""Loss helpers used by the vendored trainer and tests."""

from __future__ import annotations

import numpy as np


def cross_entropy(logits: np.ndarray, targets: np.ndarray) -> float:
    """Mean cross-entropy for ``logits [N,C]`` and integer ``targets [N]``."""
    logits = np.asarray(logits, dtype=np.float64)
    targets = np.asarray(targets, dtype=np.int64).reshape(-1)
    if logits.ndim != 2 or logits.shape[0] != targets.shape[0]:
        raise ValueError("logits/targets shape mismatch")
    shifted = logits - logits.max(axis=1, keepdims=True)
    log_probs = shifted - np.log(np.exp(shifted).sum(axis=1, keepdims=True))
    return float(-log_probs[np.arange(len(targets)), targets].mean())
