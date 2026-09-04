"""Fine-tuning losses for Fathom RGB-D depth refinement."""

from dataclasses import dataclass

import torch
import torch.nn.functional as F


@dataclass(frozen=True)
class LossOutput:
    """The individual and combined fine-tuning losses."""

    total: torch.Tensor
    depth: torch.Tensor
    mask: torch.Tensor | None
    valid_pixels: int


def fathom_loss(
    pred_depth: torch.Tensor,
    target_depth: torch.Tensor,
    valid_mask: torch.Tensor,
    pred_mask: torch.Tensor | None = None,
    mask_weight: float = 0.1,
) -> LossOutput:
    """Calculate masked depth regression and optional mask classification losses."""
    valid_pixels = int(valid_mask.sum().item())
    if valid_pixels == 0:
        raise ValueError("fathom_loss requires valid target pixels")

    depth_loss = F.smooth_l1_loss(pred_depth[valid_mask], target_depth[valid_mask])
    mask_loss = None
    total = depth_loss
    if pred_mask is not None:
        mask_loss = F.binary_cross_entropy_with_logits(pred_mask, valid_mask.float())
        total = total + mask_weight * mask_loss

    return LossOutput(
        total=total,
        depth=depth_loss,
        mask=mask_loss,
        valid_pixels=valid_pixels,
    )
