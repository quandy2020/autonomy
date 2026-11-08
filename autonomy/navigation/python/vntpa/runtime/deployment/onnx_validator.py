"""ONNX inspection, contract checks and artifact metadata."""

from __future__ import annotations

import json
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Dict, Iterable, List, Tuple

from ...utils.errors import ModelNotFoundError, ModelValidationError


@dataclass(frozen=True)
class TensorSpec:
    name: str
    shape: Tuple[object, ...]
    dtype: str


@dataclass(frozen=True)
class ModelInfo:
    path: str
    inputs: Dict[str, TensorSpec]
    outputs: Dict[str, TensorSpec]


def _spec(item) -> TensorSpec:
    return TensorSpec(item.name, tuple(item.shape), item.type)


def inspect_onnx(path: str | Path) -> ModelInfo:
    if not Path(path).is_file():
        raise ModelNotFoundError(f"ONNX model not found: {path}")
    try:
        import onnxruntime as ort

        session = ort.InferenceSession(str(path), providers=["CPUExecutionProvider"])
    except Exception as exc:  # noqa: BLE001
        raise ModelValidationError(f"cannot load ONNX model {path}: {exc}") from exc
    return ModelInfo(
        path=str(path),
        inputs={item.name: _spec(item) for item in session.get_inputs()},
        outputs={item.name: _spec(item) for item in session.get_outputs()},
    )


def require_inputs(info: ModelInfo, names: Iterable[str]) -> None:
    missing: List[str] = [name for name in names if name not in info.inputs]
    if missing:
        raise ModelValidationError(
            f"{info.path}: missing inputs {missing}; found {sorted(info.inputs)}")


def require_outputs(info: ModelInfo, names: Iterable[str]) -> None:
    missing: List[str] = [name for name in names if name not in info.outputs]
    if missing:
        raise ModelValidationError(
            f"{info.path}: missing outputs {missing}; found {sorted(info.outputs)}")


def write_manifest(path: str, config) -> Path:
    """Write ``<model>.json`` for an exported ONNX artifact."""
    info = inspect_onnx(path)
    destination = Path(f"{path}.json")
    payload = {
        "model": info.path,
        "planner": asdict(config),
        "inputs": {name: {"shape": list(spec.shape), "dtype": spec.dtype}
                   for name, spec in info.inputs.items()},
        "outputs": {name: {"shape": list(spec.shape), "dtype": spec.dtype}
                    for name, spec in info.outputs.items()},
    }
    destination.write_text(json.dumps(payload, indent=2), encoding="utf-8")
    return destination
