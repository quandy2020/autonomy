"""Optimizer factory kept separate from training orchestration."""

from __future__ import annotations


def build_optimizer(model, learning_rate: float = 3e-4, weight_decay: float = 1e-4):
    """AdamW factory; imports torch lazily so deployment stays lightweight."""
    import torch

    return torch.optim.AdamW(model.parameters(), lr=learning_rate,
                             weight_decay=weight_decay)
