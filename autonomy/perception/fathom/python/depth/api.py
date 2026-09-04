"""Small, stable adapter around the vendored LingBot-Depth model."""

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
    image_batch = _prepare_image(image).unsqueeze(0)
    depth = _prepare_depth(raw_depth)
    intrinsics_batch = _prepare_intrinsics(intrinsics)

    if image_batch.shape[-2:] != depth.shape:
        raise ValueError(
            "image and raw_depth must have matching spatial dimensions: "
            f"{tuple(image_batch.shape[-2:])} != {tuple(depth.shape)}"
        )

    output = model.infer(
        image=image_batch,
        depth_in=depth.unsqueeze(0),
        intrinsics=intrinsics_batch,
        apply_mask=apply_mask,
        use_fp16=use_fp16,
    )

    return {
        "depth": output["depth"].squeeze(0),
        "mask": output["mask"].squeeze(0),
        "raw_depth": depth,
    }


def _prepare_image(image: torch.Tensor) -> torch.Tensor:
    image = torch.as_tensor(image)
    if image.ndim != 3:
        raise ValueError("image must be HWC uint8 or CHW floating-point RGB")

    if image.dtype == torch.uint8:
        if image.shape[-1] != 3:
            raise ValueError("HWC uint8 image must have three RGB channels")
        return image.permute(2, 0, 1).to(dtype=torch.float32).div(255.0)

    if not torch.is_floating_point(image) or image.shape[0] != 3:
        raise ValueError("image must be HWC uint8 or CHW floating-point RGB")
    return image.to(dtype=torch.float32)


def _prepare_depth(raw_depth: torch.Tensor) -> torch.Tensor:
    depth = torch.as_tensor(raw_depth, dtype=torch.float32)
    if depth.ndim != 2:
        raise ValueError("raw_depth must be a two-dimensional depth map")
    return torch.where(torch.isfinite(depth) & (depth > 0), depth, torch.zeros_like(depth))


def _prepare_intrinsics(intrinsics: torch.Tensor | None) -> torch.Tensor | None:
    if intrinsics is None:
        return None
    intrinsics = torch.as_tensor(intrinsics)
    return intrinsics.unsqueeze(0) if intrinsics.ndim == 2 else intrinsics
