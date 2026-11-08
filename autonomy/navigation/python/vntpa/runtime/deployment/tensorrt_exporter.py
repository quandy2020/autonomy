"""TensorRT export placeholder with an explicit capability error."""

from __future__ import annotations

from ...utils.errors import UnsupportedFormatError


def export_tensorrt(*_args, **_kwargs) -> None:
    raise UnsupportedFormatError(
        "TensorRT export is not implemented; use ONNX and build the engine externally")
