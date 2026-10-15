# Copyright 2026 The Openbot Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0.

"""Export a linear NavEnv policy to ONNX (or NumPy npz fallback)."""

from __future__ import annotations

import json
import logging
from pathlib import Path
from typing import Any, Dict, Optional, Tuple

import numpy as np

_LOG = logging.getLogger(__name__)


def export_linear_policy(
    weight: np.ndarray,
    scale: np.ndarray,
    path: Path,
    *,
    obs_dim: Optional[int] = None,
) -> Dict[str, Any]:
    """Save ``action = scale * tanh(obs @ W)``.

    Prefers ONNX via torch; otherwise writes ``.npz`` + ``.json`` sidecars.
    """
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    w = np.asarray(weight, dtype=np.float32)
    s = np.asarray(scale, dtype=np.float32).reshape(-1)
    meta = {
        "type": "linear_tanh",
        "obs_dim": int(obs_dim or w.shape[0]),
        "act_dim": int(w.shape[1]),
        "scale": s.tolist(),
    }

    try:
        import torch
        import torch.nn as nn

        class Net(nn.Module):
            def __init__(self) -> None:
                super().__init__()
                self.fc = nn.Linear(w.shape[0], w.shape[1], bias=False)
                with torch.no_grad():
                    self.fc.weight.copy_(torch.from_numpy(w.T))
                self.register_buffer("scale", torch.from_numpy(s))

            def forward(self, obs: torch.Tensor) -> torch.Tensor:  # type: ignore[override]
                return torch.tanh(self.fc(obs)) * self.scale

        net = Net().eval()
        dummy = torch.zeros(1, w.shape[0], dtype=torch.float32)
        onnx_path = path.with_suffix(".onnx")
        torch.onnx.export(
            net,
            dummy,
            str(onnx_path),
            input_names=["observation"],
            output_names=["action"],
            dynamo=False,
        )
        meta["onnx"] = str(onnx_path)
        path.with_suffix(".json").write_text(json.dumps(meta, indent=2), encoding="utf-8")
        _LOG.info("exported ONNX policy → %s", onnx_path)
        return meta
    except Exception as exc:  # noqa: BLE001
        _LOG.info("torch/onnx unavailable (%s); saving npz", exc)
        npz_path = path.with_suffix(".npz")
        np.savez(npz_path, weight=w, scale=s)
        meta["npz"] = str(npz_path)
        path.with_suffix(".json").write_text(json.dumps(meta, indent=2), encoding="utf-8")
        return meta


def load_linear_policy(meta_path: Path) -> Tuple[np.ndarray, np.ndarray]:
    meta = json.loads(Path(meta_path).read_text(encoding="utf-8"))
    if meta.get("npz"):
        data = np.load(meta["npz"])
        return data["weight"], data["scale"]
    raise FileNotFoundError(f"no npz weights in {meta_path}; ONNX load via onnxruntime separately")
