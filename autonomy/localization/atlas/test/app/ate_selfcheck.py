#!/usr/bin/env python3
"""Synthetic ATE self-check for eval_ate Umeyama (no dataset required).

Generates a known trajectory, applies Sim3 (s, R, t) + noise, then verifies
ATE RMSE recovers to ~noise level after alignment.
"""

from __future__ import annotations

import math
import tempfile
from pathlib import Path

import numpy as np

from eval_ate import associate, load_tum, umeyama_sim3


def _write_tum(path: Path, ts: np.ndarray, xyz: np.ndarray) -> None:
    lines = []
    for t, p in zip(ts, xyz):
        lines.append(f"{t:.9f} {p[0]:.9f} {p[1]:.9f} {p[2]:.9f} 0 0 0 1")
    path.write_text("\n".join(lines) + "\n")


def main() -> None:
    rng = np.random.default_rng(0)
    n = 200
    ts = np.arange(n, dtype=float) * 0.05
    # Helix ground truth (meters).
    gt = np.column_stack(
        [
            np.cos(ts),
            np.sin(ts),
            0.1 * ts,
        ]
    )
    # Known Sim3: scale 1.5, 30° about Z, translation.
    ang = math.radians(30.0)
    R = np.array(
        [
            [math.cos(ang), -math.sin(ang), 0.0],
            [math.sin(ang), math.cos(ang), 0.0],
            [0.0, 0.0, 1.0],
        ]
    )
    s_true = 1.5
    t_true = np.array([0.2, -0.1, 0.05])
    noise = 0.01  # 1 cm
    est = (s_true * (R @ gt.T)).T + t_true + rng.normal(0.0, noise, gt.shape)

    with tempfile.TemporaryDirectory() as td:
        td_path = Path(td)
        gt_path = td_path / "gt.txt"
        est_path = td_path / "est.txt"
        _write_tum(gt_path, ts, gt)
        _write_tum(est_path, ts, est)

        gt_t, gt_p = load_tum(gt_path)
        est_t, est_p = load_tum(est_path)
        matches = associate(est_t, gt_t, 0.02)
        assert len(matches) >= n - 2, f"associate failed: {len(matches)}"

        src = np.asarray([est_p[i] for i, _ in matches])
        dst = np.asarray([gt_p[j] for _, j in matches])
        s, R_hat, t_hat = umeyama_sim3(src, dst)
        aligned = (s * (R_hat @ src.T)).T + t_hat
        err = np.linalg.norm(aligned - dst, axis=1)
        rmse = float(math.sqrt((err**2).mean()))

    # est = s_true * R * gt + t  ⇒ Umeyama(est→gt) recovers ~1/s_true.
    s_expect = 1.0 / s_true
    print("=== ATE self-check (synthetic Sim3) ===")
    print(f"matches={len(matches)}")
    print(
        f"recovered_scale={s:.6f}  (expect≈{s_expect:.6f} = 1/{s_true:.3f})"
    )
    print(f"ATE_RMSE={rmse:.6f} m  (noise_sigma={noise:.3f} m)")
    print(
        f"ATE_mean={err.mean():.6f}  ATE_median={np.median(err):.6f}  "
        f"ATE_max={err.max():.6f}"
    )
    if abs(s - s_expect) > 0.02:
        raise SystemExit(f"scale recovery failed: {s} vs {s_expect}")
    if rmse > 3.0 * noise:
        raise SystemExit(f"ATE RMSE too large for noise: {rmse}")
    print("PASS")


if __name__ == "__main__":
    main()
