"""Small, stable adapter around the vendored LingBot-Depth model."""

from __future__ import annotations

from pathlib import Path

import torch

from .model.v2 import MDMModel


def load_model(source: str | Path, device: str | torch.device) -> MDMModel:
    """Load a released Fathom checkpoint and place it in evaluation mode."""
    return MDMModel.from_pretrained(source).to(device).eval()


def infer(
    model: MDMModel,
    image: torch.Tensor,
    raw_depth: torch.Tensor,
    intrinsics: torch.Tensor | None = None,
    apply_mask: bool = True,
    use_fp16: bool = True,
) -> dict[str, torch.Tensor]:
    """Refine a raw depth image with a normalized RGB image and depth map."""
    image_batch, image_was_unbatched = _prepare_image(image)
    depth_batch, depth_was_unbatched = _prepare_depth(raw_depth)
    if image_was_unbatched != depth_was_unbatched:
        raise ValueError("image and raw_depth must both be unbatched or batched")
    if image_batch.shape[0] != depth_batch.shape[0]:
        raise ValueError(
            "image and raw_depth must have matching batch dimensions: "
            f"{image_batch.shape[0]} != {depth_batch.shape[0]}"
        )
    if image_batch.shape[-2:] != depth_batch.shape[-2:]:
        raise ValueError(
            "image and raw_depth must have matching spatial dimensions: "
            f"{tuple(image_batch.shape[-2:])} != "
            f"{tuple(depth_batch.shape[-2:])}"
        )
    intrinsics_batch = _prepare_intrinsics(
        intrinsics, image_batch.shape[0], image_was_unbatched
    )
    model_device = getattr(model, "device", None)

    if model_device is not None:
        image_batch = image_batch.to(device=model_device)
        depth_batch = depth_batch.to(device=model_device)
        if intrinsics_batch is not None:
            intrinsics_batch = intrinsics_batch.to(device=model_device)

    output = model.infer(
        image=image_batch,
        depth_in=depth_batch,
        intrinsics=intrinsics_batch,
        apply_mask=apply_mask,
        use_fp16=use_fp16,
    )

    result = {
        "depth": output["depth"],
        "mask": output["mask"],
        "raw_depth": depth_batch,
    }
    if image_was_unbatched:
        return {name: value.squeeze(0) for name, value in result.items()}
    return result


def _prepare_image(image: torch.Tensor) -> tuple[torch.Tensor, bool]:
    image = torch.as_tensor(image)
    was_unbatched = image.ndim == 3
    if was_unbatched:
        if image.dtype == torch.uint8:
            if image.shape[-1] != 3:
                raise ValueError("HWC uint8 image must have three RGB channels")
            image = image.permute(2, 0, 1)
        elif not torch.is_floating_point(image) or image.shape[0] != 3:
            raise ValueError(
                "image must be HWC uint8, CHW floating-point RGB, or NCHW RGB"
            )
        image = image.unsqueeze(0)
    elif image.ndim == 4:
        if image.shape[1] != 3 or (
            image.dtype != torch.uint8 and not torch.is_floating_point(image)
        ):
            raise ValueError("batched image must be NCHW RGB with three channels")
    else:
        raise ValueError(
            "image must be HWC uint8, CHW floating-point RGB, or NCHW RGB"
        )
    if image.shape[0] == 0 or image.shape[-2] == 0 or image.shape[-1] == 0:
        raise ValueError("image batch and spatial dimensions must be positive")
    if image.dtype == torch.uint8:
        return image.to(dtype=torch.float32).div(255.0), was_unbatched
    if not torch.all(torch.isfinite(image) & (image >= 0) & (image <= 1)):
        raise ValueError("floating-point RGB image values must be in [0, 1]")
    return image.to(dtype=torch.float32), was_unbatched


def _prepare_depth(raw_depth: torch.Tensor) -> tuple[torch.Tensor, bool]:
    depth = torch.as_tensor(raw_depth, dtype=torch.float32)
    was_unbatched = depth.ndim == 2
    if was_unbatched:
        depth = depth.unsqueeze(0)
    elif depth.ndim != 3:
        raise ValueError("raw_depth must be an HW or BHW depth tensor")
    if depth.shape[0] == 0 or depth.shape[-2] == 0 or depth.shape[-1] == 0:
        raise ValueError("raw_depth batch and spatial dimensions must be positive")
    sanitized = torch.where(
        torch.isfinite(depth) & (depth > 0), depth, torch.zeros_like(depth)
    )
    return sanitized, was_unbatched


def _prepare_intrinsics(
    intrinsics: torch.Tensor | None, batch_size: int, inputs_are_unbatched: bool
) -> torch.Tensor | None:
    if intrinsics is None:
        return None
    intrinsics = torch.as_tensor(intrinsics, dtype=torch.float32)
    if inputs_are_unbatched:
        if intrinsics.shape != (3, 3):
            raise ValueError("unbatched intrinsics must be a single 3x3 matrix")
        return intrinsics.unsqueeze(0)
    if intrinsics.ndim != 3 or intrinsics.shape[-2:] != (3, 3):
        raise ValueError("batched intrinsics must have shape [B, 3, 3]")
    if intrinsics.shape[0] != batch_size:
        raise ValueError(
            "intrinsics batch dimension must match image and raw_depth: "
            f"{intrinsics.shape[0]} != {batch_size}"
        )
    return intrinsics
