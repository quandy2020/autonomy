"""Export Fathom depth-refinement models to fixed-profile ONNX graphs."""

from __future__ import annotations

from collections.abc import Mapping
from pathlib import Path

import torch
from torch import nn


class _FixedProfileModel(nn.Module):
    """Adapt the released model's Python-only arguments to ONNX inputs."""

    def __init__(self, model: nn.Module, num_tokens: int) -> None:
        super().__init__()
        self.model = model
        self.num_tokens = num_tokens

    def forward(
        self, image: torch.Tensor, raw_depth: torch.Tensor
    ) -> tuple[torch.Tensor, torch.Tensor]:
        output = self.model(image, num_tokens=self.num_tokens, depth=raw_depth)
        if not isinstance(output, Mapping):
            raise TypeError("Fathom model forward must return a mapping")

        refined_depth = output.get("depth_reg")
        if refined_depth is None:
            raise RuntimeError("Fathom model forward did not return 'depth_reg'")

        validity = output.get("mask")
        if validity is None:
            validity = torch.ones_like(refined_depth)
        return refined_depth, validity


def export_onnx(
    model: nn.Module,
    output: str | Path,
    height: int,
    width: int,
    num_tokens: int,
    opset: int = 17,
) -> Path:
    """Export and validate a one-image fixed-profile Fathom ONNX graph.

    The graph accepts a float32 ``image`` of shape ``(1, 3, height, width)``
    and ``raw_depth`` of shape ``(1, height, width)``.  Batch, spatial, and
    token dimensions are deliberately static; camera intrinsics and point
    projection remain outside the graph.
    """
    if height <= 0 or width <= 0:
        raise ValueError("height and width must be positive")
    if num_tokens <= 0:
        raise ValueError("num_tokens must be positive")

    try:
        import onnx
        import onnxruntime as ort
    except ImportError as error:
        raise RuntimeError(
            "ONNX export validation requires the optional onnx and onnxruntime packages"
        ) from error

    output_path = Path(output)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    device = _model_device(model)
    original_training = model.training
    original_dtype = _floating_dtype(model)
    encoder = getattr(model, "encoder", None)
    original_onnx_mode = (
        getattr(encoder, "onnx_compatible_mode") if encoder is not None else None
    )

    try:
        model.eval()
        if original_dtype is not None and original_dtype != torch.float32:
            model.float()
        if encoder is not None:
            encoder.onnx_compatible_mode = True

        wrapper = _FixedProfileModel(model, num_tokens).eval()
        image, raw_depth = _deterministic_inputs(height, width, device)
        with torch.no_grad():
            expected_depth, expected_validity = wrapper(image, raw_depth)

        torch.onnx.export(
            wrapper,
            (image, raw_depth),
            output_path,
            export_params=True,
            opset_version=opset,
            do_constant_folding=True,
            input_names=["image", "raw_depth"],
            output_names=["refined_depth", "validity"],
        )
        onnx.checker.check_model(onnx.load(output_path))

        session = ort.InferenceSession(
            output_path.as_posix(), providers=["CPUExecutionProvider"]
        )
        actual_depth, actual_validity = session.run(
            ["refined_depth", "validity"],
            {
                "image": image.detach().cpu().numpy(),
                "raw_depth": raw_depth.detach().cpu().numpy(),
            },
        )
        torch.testing.assert_close(
            torch.from_numpy(actual_depth),
            expected_depth.detach().cpu(),
            rtol=1e-5,
            atol=1e-6,
        )
        torch.testing.assert_close(
            torch.from_numpy(actual_validity),
            expected_validity.detach().cpu(),
            rtol=1e-5,
            atol=1e-6,
        )
    except Exception as error:
        output_path.unlink(missing_ok=True)
        raise RuntimeError(f"ONNX export validation failed: {error}") from error
    finally:
        if encoder is not None:
            encoder.onnx_compatible_mode = original_onnx_mode
        if original_dtype is not None and original_dtype != torch.float32:
            model.to(dtype=original_dtype)
        model.train(original_training)

    return output_path


def _model_device(model: nn.Module) -> torch.device:
    for tensor in (*model.parameters(), *model.buffers()):
        return tensor.device
    return torch.device("cpu")


def _floating_dtype(model: nn.Module) -> torch.dtype | None:
    for tensor in (*model.parameters(), *model.buffers()):
        if tensor.is_floating_point():
            return tensor.dtype
    return None


def _deterministic_inputs(
    height: int, width: int, device: torch.device
) -> tuple[torch.Tensor, torch.Tensor]:
    image = torch.linspace(
        0.0, 1.0, steps=3 * height * width, dtype=torch.float32, device=device
    ).reshape(1, 3, height, width)
    raw_depth = torch.linspace(
        1.0, 2.0, steps=height * width, dtype=torch.float32, device=device
    ).reshape(1, height, width)
    return image, raw_depth
