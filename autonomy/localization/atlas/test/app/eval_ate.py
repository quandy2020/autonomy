#!/usr/bin/env python3
"""Evaluate Atlas TUM-format trajectory against EuRoC / TUM / KITTI GT.

Usage:
  # EuRoC
  python3 eval_ate.py --gt_format=euroc \\
    --gt .../state_groundtruth_estimate0/data.csv --est kf_traj.txt

  # TUM RGB-D
  python3 eval_ate.py --gt_format=tum \\
    --gt .../groundtruth.txt --est kf_traj.txt

  # KITTI odometry (poses/XX.txt + sequences/XX/times.txt)
  python3 eval_ate.py --gt_format=kitti --fix_scale \\
    --gt /path/poses/00.txt --times /path/sequences/00/times.txt \\
    --est kf_traj.txt
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
    ts, xyz = [], []
    with path.open() as f:
        _ = f.readline()
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


def load_kitti_gt(poses_path: Path, times_path: Path | None):
    """KITTI poses: each line 12 floats of 3x4 [R|t] (camera→world)."""
    xyz = []
    for line in poses_path.read_text().splitlines():
        line = line.strip()
        if not line:
            continue
        v = [float(x) for x in line.split()]
        if len(v) < 12:
            continue
        xyz.append([v[3], v[7], v[11]])
    xyz = np.asarray(xyz)
    if times_path is not None and times_path.is_file():
        ts = []
        for line in times_path.read_text().splitlines():
            line = line.strip()
            if line:
                ts.append(float(line.split()[0]))
        ts = np.asarray(ts)
        n = min(len(ts), len(xyz))
        return ts[:n], xyz[:n]
    # Fallback: synthetic timestamps at 10 Hz
    ts = np.arange(len(xyz), dtype=float) * 0.1
    return ts, xyz


def associate(ts_a: np.ndarray, ts_b: np.ndarray, max_diff: float = 0.02):
    matches = []
    j0 = 0
    for i, t in enumerate(ts_a):
        while j0 + 1 < len(ts_b) and abs(ts_b[j0 + 1] - t) <= abs(ts_b[j0] - t):
            j0 += 1
        if abs(ts_b[j0] - t) <= max_diff:
            matches.append((i, j0))
    return matches


def umeyama_sim3(src: np.ndarray, dst: np.ndarray):
    assert src.shape == dst.shape and src.shape[0] >= 3
    mu_s = src.mean(axis=0)
    mu_d = dst.mean(axis=0)
    src_c = src - mu_s
    dst_c = dst - mu_d
    cov = (dst_c.T @ src_c) / src.shape[0]
    U, S, Vt = np.linalg.svd(cov)
    R = U @ Vt
    if np.linalg.det(R) < 0:
        Vt = Vt.copy()
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
    ap.add_argument(
        "--gt_format", choices=("euroc", "tum", "kitti"), default="euroc"
    )
    ap.add_argument(
        "--times",
        type=Path,
        default=None,
        help="KITTI sequences/XX/times.txt (required for accurate sync)",
    )
    ap.add_argument("--max_diff", type=float, default=0.02)
    ap.add_argument(
        "--fix_scale",
        action="store_true",
        help="SE3 alignment only (stereo/RGB-D/KITTI metric)",
    )
    args = ap.parse_args()

    if args.gt_format == "euroc":
        gt_t, gt_p = load_euroc_gt(args.gt)
    elif args.gt_format == "kitti":
        gt_t, gt_p = load_kitti_gt(args.gt, args.times)
        if args.max_diff < 0.05:
            args.max_diff = 0.05
    else:
        gt_t, gt_p = load_tum(args.gt)
    est_t, est_p = load_tum(args.est)
    matches = associate(est_t, gt_t, args.max_diff)
    if len(matches) < 10:
        raise SystemExit(f"too few matches: {len(matches)}")

    src = np.asarray([est_p[i] for i, _ in matches])
    dst = np.asarray([gt_p[j] for _, j in matches])
    s, R, t = umeyama_sim3(src, dst)
    if args.fix_scale:
        s = 1.0
        mu_s = src.mean(axis=0)
        mu_d = dst.mean(axis=0)
        t = mu_d - R @ mu_s
    aligned = (s * (R @ src.T)).T + t
    err = np.linalg.norm(aligned - dst, axis=1)
    rmse = float(math.sqrt((err**2).mean()))
    print(f"matches={len(matches)}  scale={s:.6f}  ATE_RMSE={rmse:.6f} m")
    print(
        f"ATE_mean={err.mean():.6f}  ATE_median={np.median(err):.6f}  "
        f"ATE_max={err.max():.6f}"
    )


if __name__ == "__main__":
    main()
