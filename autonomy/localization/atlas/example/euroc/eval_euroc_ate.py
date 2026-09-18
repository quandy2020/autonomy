#!/usr/bin/env python3
"""Evaluate Atlas EuRoC trajectory (TUM) against ground-truth CSV.

Usage:
  python3 eval_euroc_ate.py \\
    --gt /path/to/MH_01_easy/mav0/state_groundtruth_estimate0/data.csv \\
    --est keyframe_trajectory_euroc.txt

Prints RMSE ATE after Umeyama Sim(3) alignment (monocular / VIO scale free).
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path

import numpy as np


def load_tum(path: Path) -> tuple[np.ndarray, np.ndarray]:
    ts, xyz = [], []
    for line in path.read_text().splitlines():
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        p = line.split()
        if len(p) < 8:
            continue
        ts.append(float(p[0]))
        xyz.append([float(p[1]), float(p[2]), float(p[3])])
    return np.asarray(ts), np.asarray(xyz)


def load_euroc_gt(path: Path) -> tuple[np.ndarray, np.ndarray]:
    """EuRoC GT CSV: timestamp[ns], p_RS_R_x/y/z, q_..."""
    ts, xyz = [], []
    with path.open() as f:
        header = f.readline()
        _ = header
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            p = line.replace(",", " ").split()
            if len(p) < 5:
                continue
            ts.append(float(p[0]) * 1e-9)
            xyz.append([float(p[1]), float(p[2]), float(p[3])])
    return np.asarray(ts), np.asarray(xyz)


def associate(ts_a: np.ndarray, ts_b: np.ndarray, max_diff: float = 0.02):
    """Greedy nearest-neighbor timestamp association."""
    matches = []
    j0 = 0
    for i, t in enumerate(ts_a):
        while j0 + 1 < len(ts_b) and abs(ts_b[j0 + 1] - t) <= abs(ts_b[j0] - t):
            j0 += 1
        if abs(ts_b[j0] - t) <= max_diff:
            matches.append((i, j0))
    return matches


def umeyama_sim3(src: np.ndarray, dst: np.ndarray) -> tuple[float, np.ndarray, np.ndarray]:
    """Return s, R, t such that dst ≈ s R src + t."""
    assert src.shape == dst.shape and src.shape[0] >= 3
    mu_s = src.mean(axis=0)
    mu_d = dst.mean(axis=0)
    src_c = src - mu_s
    dst_c = dst - mu_d
    cov = (dst_c.T @ src_c) / src.shape[0]
    U, S, Vt = np.linalg.svd(cov)
    R = U @ Vt
    if np.linalg.det(R) < 0:
        Vt[-1, :] *= -1
        R = U @ Vt
    var_s = (src_c**2).sum() / src.shape[0]
    s = float(S.sum() / max(var_s, 1e-12))
    t = mu_d - s * R @ mu_s
    return s, R, t


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--gt", required=True, type=Path)
    ap.add_argument("--est", required=True, type=Path)
    ap.add_argument("--max_diff", type=float, default=0.02)
    args = ap.parse_args()

    gt_t, gt_p = load_euroc_gt(args.gt)
    est_t, est_p = load_tum(args.est)
    matches = associate(est_t, gt_t, args.max_diff)
    if len(matches) < 10:
        raise SystemExit(f"too few matches: {len(matches)}")

    src = np.asarray([est_p[i] for i, _ in matches])
    dst = np.asarray([gt_p[j] for _, j in matches])
    s, R, t = umeyama_sim3(src, dst)
    aligned = (s * (R @ src.T)).T + t
    err = np.linalg.norm(aligned - dst, axis=1)
    rmse = float(math.sqrt((err**2).mean()))
    print(f"matches={len(matches)}  scale={s:.6f}  ATE_RMSE={rmse:.6f} m")
    print(f"ATE_mean={err.mean():.6f}  ATE_median={np.median(err):.6f}  ATE_max={err.max():.6f}")


if __name__ == "__main__":
    main()
