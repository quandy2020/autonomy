#!/usr/bin/env python3
# Copyright 2026 The Openbot Authors
# SPDX-License-Identifier: Apache-2.0
"""Accuracy metrics: ATE / RPE / Scale (evo wrapper + fallback)."""

from __future__ import annotations

import argparse
import json
import math
import shutil
import subprocess
import sys
from pathlib import Path
from typing import Any


def _load_tum(path: Path) -> list[tuple[float, list[float]]]:
    rows: list[tuple[float, list[float]]] = []
    for line in path.read_text(encoding="utf-8").splitlines():
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        parts = line.split()
        if len(parts) < 8:
            continue
        t = float(parts[0])
        xyz = [float(parts[i]) for i in range(1, 4)]
        rows.append((t, xyz))
    return rows


def _associate(
    a: list[tuple[float, list[float]]],
    b: list[tuple[float, list[float]]],
    tol: float = 0.02,
) -> list[tuple[list[float], list[float]]]:
    pairs: list[tuple[list[float], list[float]]] = []
    j = 0
    for ta, pa in a:
        while j + 1 < len(b) and abs(b[j + 1][0] - ta) < abs(b[j][0] - ta):
            j += 1
        if abs(b[j][0] - ta) <= tol:
            pairs.append((pa, b[j][1]))
    return pairs


def ate_rmse(est: Path, gt: Path) -> dict[str, float]:
    """Simple translational ATE RMSE after associating timestamps (no SE3 Umeyama)."""
    ea = _load_tum(est)
    ga = _load_tum(gt)
    pairs = _associate(ea, ga)
    if len(pairs) < 2:
        return {"ate_rmse_m": float("nan"), "n_pairs": 0.0}
    err = 0.0
    for pe, pg in pairs:
        dx = pe[0] - pg[0]
        dy = pe[1] - pg[1]
        dz = pe[2] - pg[2]
        err += dx * dx + dy * dy + dz * dz
    return {"ate_rmse_m": math.sqrt(err / len(pairs)), "n_pairs": float(len(pairs))}


def rpe_rmse(est: Path, gt: Path, delta: int = 1) -> dict[str, float]:
    ea = _load_tum(est)
    ga = _load_tum(gt)
    pairs = _associate(ea, ga)
    if len(pairs) <= delta:
        return {"rpe_rmse_m": float("nan"), "n_pairs": 0.0}
    err = 0.0
    n = 0
    for i in range(len(pairs) - delta):
        pe0, pg0 = pairs[i]
        pe1, pg1 = pairs[i + delta]
        de = [pe1[k] - pe0[k] for k in range(3)]
        dg = [pg1[k] - pg0[k] for k in range(3)]
        d = [(de[k] - dg[k]) for k in range(3)]
        err += d[0] * d[0] + d[1] * d[1] + d[2] * d[2]
        n += 1
    return {"rpe_rmse_m": math.sqrt(err / max(n, 1)), "n_pairs": float(n)}


def scale_ratio(est: Path, gt: Path) -> dict[str, float]:
    ea = _load_tum(est)
    ga = _load_tum(gt)
    pairs = _associate(ea, ga)
    if len(pairs) < 2:
        return {"scale": float("nan")}
    le = 0.0
    lg = 0.0
    for i in range(1, len(pairs)):
        pe0, pg0 = pairs[i - 1]
        pe1, pg1 = pairs[i]
        le += math.dist(pe0, pe1)
        lg += math.dist(pg0, pg1)
    if lg < 1e-9:
        return {"scale": float("nan")}
    return {"scale": le / lg}


def run_evo(est: Path, gt: Path, mode: str) -> dict[str, Any] | None:
    evo = shutil.which("evo_ape" if mode == "ate" else "evo_rpe")
    if not evo:
        return None
    cmd = [evo, "tum", str(gt), str(est), "-a", "--no_warnings"]
    try:
        out = subprocess.check_output(cmd, text=True, stderr=subprocess.STDOUT)
    except (subprocess.CalledProcessError, FileNotFoundError):
        return None
    return {"evo_raw": out}


def evaluate(est: Path, gt: Path) -> dict[str, Any]:
    result: dict[str, Any] = {}
    result.update(ate_rmse(est, gt))
    result.update(rpe_rmse(est, gt))
    result.update(scale_ratio(est, gt))
    evo_ate = run_evo(est, gt, "ate")
    if evo_ate:
        result["evo_ate"] = evo_ate
    return result


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--est", required=True, type=Path)
    p.add_argument("--gt", required=True, type=Path)
    p.add_argument("--out", type=Path, default=None)
    args = p.parse_args()
    if not args.est.is_file() or not args.gt.is_file():
        print("est/gt not found", file=sys.stderr)
        return 1
    result = evaluate(args.est, args.gt)
    text = json.dumps(result, indent=2)
    print(text)
    if args.out:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        args.out.write_text(text + "\n", encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
