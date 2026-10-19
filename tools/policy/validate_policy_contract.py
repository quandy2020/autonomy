#!/usr/bin/env python3
# Copyright 2026 The Openbot Authors (duyongquan)
#
# Validate ONNX I/O names/shapes against a PolicyConfig textproto sidecar.
#
# Usage:
#   python3 tools/policy/validate_policy_contract.py \
#       --config autonomy/control/conf/policy_config_example.pb.txt \
#       --onnx /path/to/policy.onnx
#
# Requires: onnx (pip install onnx). Textproto is parsed lightly without
# generated pb2 bindings.

"""Check PolicyConfig ↔ ONNX tensor contract before deploy."""

from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path
from typing import Any


def _parse_textproto(path: Path) -> dict[str, Any]:
    """Lightweight extraction of inputs / action / engine from textproto."""
    text = path.read_text(encoding="utf-8")
    cfg: dict[str, Any] = {
        "name": "",
        "inputs": [],
        "action_name": "action",
        "layout": [],
        "model_path": "",
        "observation_flags": {},
        "is_diffusion": False,
        "diffusion": {},
    }

    m = re.search(r'^\s*name:\s*"([^"]+)"', text, re.M)
    if m:
        cfg["name"] = m.group(1)

    m = re.search(r'engine\s*\{[^}]*model_path:\s*"([^"]+)"', text, re.S)
    if m:
        cfg["model_path"] = m.group(1)

    m = re.search(r'action\s*\{([^}]*(?:\{[^}]*\}[^}]*)*)\}', text, re.S)
    if m:
        block = m.group(1)
        n = re.search(r'name:\s*"([^"]+)"', block)
        if n:
            cfg["action_name"] = n.group(1)
        for layout in re.findall(r'layout:\s*"([^"]+)"', block):
            cfg["layout"].append(layout)

    if re.search(r'mode:\s*INFERENCE_DIFFUSION', text):
        cfg["is_diffusion"] = True
    dm = re.search(r'diffusion\s*\{([^}]*(?:\{[^}]*\}[^}]*)*)\}', text, re.S)
    if dm:
        dblock = dm.group(1)
        for key in ("sample", "timestep", "prediction"):
            n = re.search(rf'{key}:\s*"([^"]+)"', dblock)
            if n:
                cfg["diffusion"][key] = n.group(1)
        cfg["diffusion"].setdefault("sample", "sample")
        cfg["diffusion"].setdefault("timestep", "timestep")

    # observation { kinematic: true ... }
    om = re.search(r'observation\s*\{([^}]*)\}', text, re.S)
    if om:
        for flag in (
            "kinematic",
            "path",
            "costmap",
            "laser",
            "image",
            "point_cloud",
            "lidar_bev",
            "raw",
        ):
            if re.search(rf'\b{flag}\s*:\s*true\b', om.group(1)):
                cfg["observation_flags"][flag] = True

    defaults = {
        "kinematic": ("observation_kinematic", [1, 6]),
        "path": ("observation_path", [1, 40]),
        "costmap": ("observation_costmap", [1, 1, 64, 64]),
        "laser": ("observation_laser", [1, 180]),
        "image": ("observation_image", [1, 3, 224, 224]),
        "point_cloud": ("observation_point_cloud", [1, 2048, 3]),
        "lidar_bev": ("observation_lidar_bev", [1, 1, 200, 200]),
        "raw": ("observation_raw", [1, 1]),
    }
    for flag, (name, shape) in defaults.items():
        if cfg["observation_flags"].get(flag):
            cfg["inputs"].append({"name": name, "shape": shape, "required": True})

    # Explicit inputs { name / modality / shape }
    for block in re.finditer(r'inputs\s*\{([^}]*(?:\{[^}]*\}[^}]*)*)\}', text, re.S):
        body = block.group(1)
        name_m = re.search(r'name:\s*"([^"]+)"', body)
        if not name_m:
            continue
        shape = [int(x) for x in re.findall(r'shape:\s*(-?\d+)', body)]
        required = True
        if re.search(r'required:\s*false', body):
            required = False
        cfg["inputs"].append(
            {"name": name_m.group(1), "shape": shape, "required": required}
        )

    return cfg


def _shape_compatible(expected: list[int], actual: list[int]) -> bool:
    if not expected:
        return True
    if len(expected) != len(actual):
        return False
    dynamic = 0
    for e, a in zip(expected, actual):
        if e <= 0:
            dynamic += 1
            continue
        if a > 0 and e != a:
            return False
    return dynamic <= 1


def _onnx_io(path: Path) -> tuple[dict[str, list[int]], dict[str, list[int]]]:
    try:
        import onnx
        from onnx import shape_inference
    except ImportError as exc:
        raise SystemExit(
            "onnx is required: pip install onnx\n" + str(exc)
        ) from exc

    model = onnx.load(str(path))
    try:
        model = shape_inference.infer_shapes(model)
    except Exception:  # noqa: BLE001 — best-effort
        pass

    def dims(vi) -> list[int]:
        out: list[int] = []
        for d in vi.type.tensor_type.shape.dim:
            if d.dim_value:
                out.append(int(d.dim_value))
            else:
                out.append(-1)
        return out

    inputs = {i.name: dims(i) for i in model.graph.input if i.name}
    # Drop initializers that appear as inputs in some exports.
    init = {i.name for i in model.graph.initializer}
    inputs = {k: v for k, v in inputs.items() if k not in init}
    outputs = {o.name: dims(o) for o in model.graph.output if o.name}
    return inputs, outputs


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default="", help="PolicyConfig .pb.txt")
    parser.add_argument(
        "--onnx",
        default="",
        help="ONNX path (default: engine.model_path from config)",
    )
    parser.add_argument(
        "--checklist",
        action="store_true",
        help="Print export contract checklist and exit",
    )
    args = parser.parse_args()

    if args.checklist:
        print(
            """
Export contract checklist (algorithm → policy_controller)
=========================================================
[ ] Float32 tensors only
[ ] Observation inputs use sidecar names
      kinematic → observation_kinematic   shape [1,6] (or override)
      path       → observation_path       shape [1, 2*path_points]
      laser      → observation_laser      shape [1, laser_bins]
      image      → observation_image      shape [1,C,H,W] NCHW
      point_cloud→ observation_point_cloud shape [1,N,3]
[ ] Action output name matches action.name (default "action")
[ ] TWIST layout length matches output last-dim (default lx, az → 2)
[ ] 6DOF layout lists all axes present in the tensor
[ ] Diffusion: sample + timestep inputs + prediction/action output
[ ] Sidecar engine.model_path points at the exported file
[ ] Run: python3 tools/policy/validate_policy_contract.py --config ... --onnx ...
""".strip()
        )
        return 0

    if not args.config:
        print("--config is required (or pass --checklist)", file=sys.stderr)
        return 2

    cfg_path = Path(args.config)
    if not cfg_path.is_file():
        print(f"Config not found: {cfg_path}", file=sys.stderr)
        return 2

    cfg = _parse_textproto(cfg_path)
    onnx_path = Path(args.onnx) if args.onnx else Path(cfg["model_path"])
    if not onnx_path.is_file():
        print(f"ONNX not found: {onnx_path}", file=sys.stderr)
        return 2

    onnx_in, onnx_out = _onnx_io(onnx_path)
    errors: list[str] = []

    print(f"Policy: {cfg['name'] or cfg_path.name}")
    print(f"ONNX:   {onnx_path}")
    print(f"Inputs (engine):  {sorted(onnx_in)}")
    print(f"Outputs (engine): {sorted(onnx_out)}")

    for inp in cfg["inputs"]:
        if not inp["required"]:
            continue
        name = inp["name"]
        if name not in onnx_in:
            errors.append(f"Required input missing in ONNX: {name}")
            continue
        if not _shape_compatible(inp["shape"], onnx_in[name]):
            errors.append(
                f"Input shape mismatch for {name}: "
                f"config={inp['shape']} onnx={onnx_in[name]}"
            )

    if cfg["is_diffusion"]:
        for key in ("sample", "timestep"):
            name = cfg["diffusion"].get(key, key)
            if name not in onnx_in:
                errors.append(f"Diffusion {key} input missing: {name}")

    action = cfg["action_name"]
    if cfg["is_diffusion"] and cfg["diffusion"].get("prediction"):
        action = cfg["diffusion"]["prediction"]
    if action not in onnx_out:
        errors.append(f"Action/prediction output missing in ONNX: {action}")
    else:
        need = max(2, len(cfg["layout"])) if cfg["layout"] else 2
        have = 1
        for d in onnx_out[action]:
            if d > 0:
                have *= d
        if have > 0 and have < need:
            errors.append(
                f"Action output too short: need>={need} elements, got {have} "
                f"(shape={onnx_out[action]})"
            )

    if errors:
        print("FAILED:")
        for e in errors:
            print(f"  - {e}")
        return 1

    print("OK: PolicyConfig ↔ ONNX contract matches.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
