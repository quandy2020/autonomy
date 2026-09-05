"""Fine-tuning loop and checkpoint helpers for Fathom depth models."""

from __future__ import annotations

from collections.abc import Iterable, Mapping
from pathlib import Path
from typing import Any

import torch
from torch import nn
from torch.optim import Optimizer

from .loss import fathom_loss


def save_checkpoint(
    path: str | Path,
    model: nn.Module,
    model_config: dict[str, Any],
    optimizer: Optimizer,
    step: int,
) -> None:
    """Write a released-model-compatible checkpoint with training state."""
    checkpoint_path = Path(path)
    checkpoint_path.parent.mkdir(parents=True, exist_ok=True)
    torch.save(
        {
            "model_config": model_config,
            "model": model.state_dict(),
            "optimizer": optimizer.state_dict(),
            "step": step,
        },
        checkpoint_path,
    )


def train_epoch(
    model: nn.Module,
    loader: Iterable[Mapping[str, torch.Tensor]],
    optimizer: Optimizer,
    device: str | torch.device,
    mask_weight: float,
) -> dict[str, float]:
    """Optimize ``model`` over one loader pass and return mean losses."""
    target_device = torch.device(device)
    model.to(target_device).train()
    total_loss = 0.0
    depth_loss = 0.0
    mask_loss = 0.0
    batch_count = 0

    for batch in loader:
        inputs = {name: value.to(target_device) for name, value in batch.items()}
        optimizer.zero_grad()
        outputs = model(
            image=inputs["image"],
            depth=inputs["raw_depth"],
            num_tokens=_num_tokens(model),
        )
        pred_mask = outputs.get("mask")
        losses = fathom_loss(
            outputs["depth_reg"],
            inputs["target_depth"],
            inputs["valid_mask"],
            pred_mask=_mask_logits(pred_mask),
            mask_weight=mask_weight,
        )

        losses.total.backward()
        optimizer.step()

        total_loss += float(losses.total.detach().item())
        depth_loss += float(losses.depth.detach().item())
        mask_loss += 0.0 if losses.mask is None else float(losses.mask.detach().item())
        batch_count += 1

    if batch_count == 0:
        raise ValueError("train_epoch requires at least one batch")

    return {
        "total": total_loss / batch_count,
        "depth": depth_loss / batch_count,
        "mask": mask_loss / batch_count,
    }


def _num_tokens(model: nn.Module) -> int:
    """Use the released model's highest supported token count for fine-tuning."""
    token_range = getattr(model, "num_tokens_range", (3600,))
    return int(token_range[-1])


def _mask_logits(mask: torch.Tensor | None) -> torch.Tensor | None:
    """Convert released-model mask probabilities to stable BCE logits."""
    if mask is None:
        return None
    return torch.logit(mask.clamp(min=1e-6, max=1.0 - 1e-6))
