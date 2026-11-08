"""PyTorch runtime adapter used for export/parity checks."""

from __future__ import annotations


def load_checkpoint(path: str, device: str = "cpu"):
    import torch

    return torch.load(path, map_location=device)
