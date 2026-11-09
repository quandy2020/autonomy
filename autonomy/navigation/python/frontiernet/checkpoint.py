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

"""Checkpoint plumbing for the encoder swap.

``rgbd_11cls.pth`` contains ``encoder.*``, ``decoder.*``,
``segmentation_head.*`` and ``df_regression_head.*``. When the encoder becomes
DeFM, the ``encoder.*`` tensors are dropped and everything else is transplanted
verbatim. This module makes that transfer explicit and auditable instead of
relying on ``strict=False`` and hoping.
"""

from __future__ import annotations

import os
from dataclasses import dataclass, field
from typing import Dict, Iterable, List, Optional, Sequence

import torch

__all__ = [
    "HEAD_PREFIXES",
    "LoadReport",
    "head_state_dict",
    "load_frontiernet_heads",
    "load_heads",
    "read_state_dict",
    "resolve_weights",
    "save_checkpoint",
]

HEAD_PREFIXES: Sequence[str] = ("decoder.", "segmentation_head.", "df_regression_head.")
_NESTED_KEYS = ("state_dict", "model", "net", "weights")


@dataclass
class LoadReport:
    """Outcome of a partial state-dict load."""

    loaded: List[str] = field(default_factory=list)
    skipped: List[str] = field(default_factory=list)
    missing: List[str] = field(default_factory=list)
    unexpected: List[str] = field(default_factory=list)
    shape_mismatch: List[str] = field(default_factory=list)

    @property
    def ok(self) -> bool:
        return not (self.missing or self.unexpected or self.shape_mismatch)

    def summary(self) -> str:
        return (
            f"loaded={len(self.loaded)} skipped={len(self.skipped)} "
            f"missing={len(self.missing)} unexpected={len(self.unexpected)} "
            f"shape_mismatch={len(self.shape_mismatch)}"
        )

    def raise_for_errors(self) -> None:
        if self.ok:
            return
        details = []
        for label in ("missing", "unexpected", "shape_mismatch"):
            values = getattr(self, label)
            if values:
                preview = ", ".join(values[:5])
                suffix = "" if len(values) <= 5 else f" (+{len(values) - 5} more)"
                details.append(f"{label}: {preview}{suffix}")
        raise RuntimeError("checkpoint load failed -> " + "; ".join(details))


def resolve_weights(path: str) -> str:
    """Locate a checkpoint given either a repo-relative or a ``python/``-relative path.

    ``conf/models/rgbd_11cls.pth`` is how the config files spell the path, but
    the Python package lives one level down in ``python/``. Resolve the second
    spelling instead of forcing every caller to know which directory it is in.
    """
    if os.path.isfile(path):
        return path
    package_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    candidate = os.path.normpath(
        os.path.join(package_dir, os.pardir, "conf", "models", os.path.basename(path))
    )
    return candidate if os.path.isfile(candidate) else path


def read_state_dict(path: str, map_location: str = "cpu") -> Dict[str, torch.Tensor]:
    """Load a checkpoint, unwrapping common containers and ``module.`` prefixes."""
    payload = torch.load(path, map_location=map_location)
    if isinstance(payload, dict):
        for key in _NESTED_KEYS:
            nested = payload.get(key)
            if isinstance(nested, dict) and nested and all(
                isinstance(value, torch.Tensor) for value in nested.values()
            ):
                payload = nested
                break
    if not isinstance(payload, dict):
        raise TypeError(f"{path} does not contain a state dict")
    state: Dict[str, torch.Tensor] = {}
    for key, value in payload.items():
        if not isinstance(value, torch.Tensor):
            continue
        state[key[7:] if key.startswith("module.") else key] = value
    if not state:
        raise ValueError(f"{path} contains no tensors")
    return state


def _is_head_key(key: str, prefixes: Iterable[str] = HEAD_PREFIXES) -> bool:
    return any(key.startswith(prefix) for prefix in prefixes)


def head_state_dict(state: Dict[str, torch.Tensor]) -> Dict[str, torch.Tensor]:
    """Keep only decoder/head tensors from a full FrontierNet state dict."""
    return {key: value for key, value in state.items() if _is_head_key(key)}


def load_heads(
    model: torch.nn.Module,
    state: Dict[str, torch.Tensor],
    strict: bool = True,
) -> LoadReport:
    """Load decoder + heads into ``model``; encoder tensors are ignored."""
    target = model.state_dict()
    wanted: Dict[str, torch.Tensor] = {}
    report = LoadReport()
    for key, value in state.items():
        if not _is_head_key(key):
            report.skipped.append(key)
            continue
        if key not in target:
            report.unexpected.append(key)
            continue
        if tuple(target[key].shape) != tuple(value.shape):
            report.shape_mismatch.append(key)
            continue
        wanted[key] = value
        report.loaded.append(key)
    report.missing = [
        key for key in target if _is_head_key(key) and key not in wanted
    ]
    if strict:
        report.raise_for_errors()
    model.load_state_dict(wanted, strict=False)
    return report


def load_frontiernet_heads(
    model: torch.nn.Module,
    path: str,
    strict: bool = True,
    map_location: str = "cpu",
) -> LoadReport:
    """Convenience wrapper: read ``path`` and transplant decoder + heads."""
    resolved = resolve_weights(path)
    return load_heads(
        model, read_state_dict(resolved, map_location=map_location), strict=strict
    )


def save_checkpoint(
    model: torch.nn.Module,
    path: str,
    meta: Optional[Dict] = None,
) -> None:
    """Save weights plus a small metadata block (no optimizer state)."""
    payload = {"state_dict": model.state_dict()}
    if meta:
        payload["meta"] = meta
    torch.save(payload, path)
