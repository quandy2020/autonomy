#!/usr/bin/env python3
"""Align estimate to GT (Umeyama SE3) and save a comparison figure + ATE stats.

Usage:
  python3 scripts/plot_traj_ate.py --gt gt.tum --est estimate.tum --out ate.png
"""

from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


def load_tum(path: Path) -> tuple[np.ndarray, np.ndarray]:
    ts, xyz = [], []
    with path.open() as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            p = line.split()
            if len(p) < 8:
                continue
            ts.append(float(p[0]))
            xyz.append([float(p[1]), float(p[2]), float(p[3])])
    return np.asarray(ts), np.asarray(xyz)


def associate(
    t_est: np.ndarray,
    xyz_est: np.ndarray,
    t_gt: np.ndarray,
    xyz_gt: np.ndarray,
    max_dt: float,
):
    j = 0
    pe, pg, te = [], [], []
    for i, t in enumerate(t_est):
        while j + 1 < len(t_gt) and abs(t_gt[j + 1] - t) <= abs(t_gt[j] - t):
            j += 1
        if abs(t_gt[j] - t) > max_dt:
            continue
        pe.append(xyz_est[i])
        pg.append(xyz_gt[j])
        te.append(t)
    return np.asarray(te), np.asarray(pe), np.asarray(pg)


def umeyama_se3(src: np.ndarray, dst: np.ndarray):
    """Find R,t minimizing ||R src + t - dst|| (no scale)."""
    mu_s = src.mean(axis=0)
    mu_d = dst.mean(axis=0)
    xs = src - mu_s
    xd = dst - mu_d
    W = xd.T @ xs / src.shape[0]
    U, _, Vt = np.linalg.svd(W)
    R = U @ Vt
    if np.linalg.det(R) < 0:
        U[:, -1] *= -1
        R = U @ Vt
    t = mu_d - R @ mu_s
    return R, t


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--gt", required=True, help="GT TUM trajectory")
    ap.add_argument("--est", required=True, help="Estimate TUM trajectory")
    ap.add_argument("--out", required=True, help="Output PNG path")
    ap.add_argument("--max-dt", type=float, default=0.5)
    ap.add_argument("--title", default="Argus LIVO vs Ground Truth")
    args = ap.parse_args()

    t_gt, xyz_gt = load_tum(Path(args.gt))
    t_est, xyz_est = load_tum(Path(args.est))
    if len(t_gt) < 2 or len(t_est) < 2:
        raise SystemExit(f"empty traj gt={len(t_gt)} est={len(t_est)}")

    te, pe, pg = associate(t_est, xyz_est, t_gt, xyz_gt, args.max_dt)
    if len(te) < 3:
        raise SystemExit(f"too few associations: {len(te)} (max_dt={args.max_dt})")

    R, t = umeyama_se3(pe, pg)
    pe_a = (R @ pe.T).T + t
    err = np.linalg.norm(pe_a - pg, axis=1)
    ate_rmse = float(np.sqrt(np.mean(err**2)))
    ate_mean = float(np.mean(err))
    ate_max = float(np.max(err))

    fig = plt.figure(figsize=(12, 5.2), dpi=160)
    ax0 = fig.add_subplot(1, 2, 1)
    ax0.plot(xyz_gt[:, 0], xyz_gt[:, 1], color="#1f77b4", lw=2.0, label="GT (VILENS)")
    ax0.plot(
        pe_a[:, 0],
        pe_a[:, 1],
        color="#d62728",
        lw=1.6,
        alpha=0.9,
        label="Argus (aligned)",
    )
    ax0.scatter(xyz_gt[0, 0], xyz_gt[0, 1], c="green", s=40, zorder=5, label="start")
    ax0.set_aspect("equal", adjustable="datalim")
    ax0.set_xlabel("x [m]")
    ax0.set_ylabel("y [m]")
    ax0.set_title("XY trajectory")
    ax0.grid(True, alpha=0.3)
    ax0.legend(loc="best", fontsize=8)

    ax1 = fig.add_subplot(1, 2, 2)
    ax1.plot(te - te[0], err, color="#ff7f0e", lw=1.2)
    ax1.set_xlabel("time since start [s]")
    ax1.set_ylabel("ATE [m]")
    ax1.set_title(
        f"ATE  RMSE={ate_rmse:.3f} m  mean={ate_mean:.3f}  max={ate_max:.3f}"
    )
    ax1.grid(True, alpha=0.3)

    fig.suptitle(
        f"{args.title}\nmatched={len(te)}  gt={len(t_gt)}  est={len(t_est)}",
        fontsize=11,
    )
    fig.tight_layout()
    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out)
    print(f"wrote {out}")
    print(
        f"ATE_RMSE_m={ate_rmse:.6f} ATE_mean_m={ate_mean:.6f} "
        f"ATE_max_m={ate_max:.6f} matched={len(te)}"
    )


if __name__ == "__main__":
    main()
