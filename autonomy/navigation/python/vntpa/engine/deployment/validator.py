# Copyright 2026 The Navigation Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""ONNX inspection, contract checks and artifact metadata."""

from __future__ import annotations

import json
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Dict, Iterable, List, Tuple

from vntpa.utils.errors import ModelNotFoundError, ModelValidationError


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


def build_tensor_spec(item) -> TensorSpec:
    return TensorSpec(item.name, tuple(item.shape), item.type)


def inspect(path: str | Path) -> ModelInfo:
    if not Path(path).is_file():
        raise ModelNotFoundError(f"ONNX model not found: {path}")
    try:
        import onnxruntime as ort

        session = ort.InferenceSession(str(path), providers=["CPUExecutionProvider"])
    except Exception as exc:  # noqa: BLE001
        raise ModelValidationError(f"cannot load ONNX model {path}: {exc}") from exc
    return ModelInfo(
        path=str(path),
        inputs={item.name: build_tensor_spec(item) for item in session.get_inputs()},
        outputs={item.name: build_tensor_spec(item) for item in session.get_outputs()},
    )


def require_model_inputs(info: ModelInfo, names: Iterable[str]) -> None:
    missing: List[str] = [name for name in names if name not in info.inputs]
    if missing:
        raise ModelValidationError(
            f"{info.path}: missing inputs {missing}; found {sorted(info.inputs)}"
        )


def require_model_outputs(info: ModelInfo, names: Iterable[str]) -> None:
    missing: List[str] = [name for name in names if name not in info.outputs]
    if missing:
        raise ModelValidationError(
            f"{info.path}: missing outputs {missing}; found {sorted(info.outputs)}"
        )


def manifest(path: str, config) -> Path:
    """Write ``<model>.json`` for an exported ONNX artifact."""
    info = inspect(path)
    destination = Path(f"{path}.json")
    payload = {
        "model": info.path,
        "planner": asdict(config),
        "inputs": {
            name: {"shape": list(spec.shape), "dtype": spec.dtype}
            for name, spec in info.inputs.items()
        },
        "outputs": {
            name: {"shape": list(spec.shape), "dtype": spec.dtype}
            for name, spec in info.outputs.items()
        },
    }
    destination.write_text(json.dumps(payload, indent=2), encoding="utf-8")
    return destination
