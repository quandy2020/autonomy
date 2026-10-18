#!/usr/bin/env python3
"""Compare multiple mode trajectories (LIVO/LIO/LO) vs GT on one figure.

Usage:
  python3 scripts/plot_modes_compare.py --gt gt.tum --modes livo=a.tum,lio=b.tum --out modes.png
"""

from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

# Reuse helpers from sibling module without package install.
import importlib.util

_spec = importlib.util.spec_from_file_location(
    "plot_traj_ate",
    Path(__file__).resolve().parent / "plot_traj_ate.py",
)
_mod = importlib.util.module_from_spec(_spec)
assert _spec.loader is not None
_spec.loader.exec_module(_mod)
load_tum = _mod.load_tum
associate = _mod.associate
umeyama_se3 = _mod.umeyama_se3

# High-contrast palette (colorblind-friendlier + distinct on white).
COLORS = {
    "gt": "#111111",  # black
    "livo": "#E41A1C",  # vivid red
    "lio": "#377EB8",  # strong blue
    "lo": "#984EA3",  # purple (diverges clearly from red/blue)
}
LINESTYLES = {
    "gt": (0, (5, 2)),  # dashed
    "livo": "solid",
    "lio": "solid",
    "lo": "solid",
}
LINEWIDTHS = {
    "gt": 2.8,
    "livo": 2.0,
    "lio": 2.0,
    "lo": 2.0,
}


def align_to_gt(t_est, xyz_est, t_gt, xyz_gt, max_dt: float):
    te, pe, pg = associate(t_est, xyz_est, t_gt, xyz_gt, max_dt)
    if len(te) < 3:
        return None
    R, t = umeyama_se3(pe, pg)
    pe_a = (R @ pe.T).T + t
    # also transform full estimate for continuous plot
    xyz_full = (R @ xyz_est.T).T + t
    err = np.linalg.norm(pe_a - pg, axis=1)
    stats = {
        "matched": len(te),
        "rmse": float(np.sqrt(np.mean(err**2))),
        "mean": float(np.mean(err)),
        "max": float(np.max(err)),
        "te": te,
        "err": err,
        "xy_aligned": pe_a,
        "xy_full": xyz_full,
    }
    return stats


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--gt", required=True)
    ap.add_argument(
        "--est",
        action="append",
        required=True,
        metavar="NAME=PATH",
        help="e.g. --est livo=/path/estimate.tum (repeatable)",
    )
    ap.add_argument("--out", required=True)
    ap.add_argument("--max-dt", type=float, default=1.0)
    ap.add_argument("--title", default="Argus modes vs VILENS")
    args = ap.parse_args()

    t_gt, xyz_gt = load_tum(Path(args.gt))
    if len(t_gt) < 2:
        raise SystemExit("empty GT")

    modes = []
    for item in args.est:
        if "=" not in item:
            raise SystemExit(f"--est needs NAME=PATH, got {item!r}")
        name, path = item.split("=", 1)
        name = name.strip().lower()
        t_est, xyz_est = load_tum(Path(path))
        st = align_to_gt(t_est, xyz_est, t_gt, xyz_gt, args.max_dt)
        if st is None:
            raise SystemExit(f"too few associations for {name}")
        modes.append((name, st, len(t_est)))

    fig = plt.figure(figsize=(13.5, 5.4), dpi=160)
    ax0 = fig.add_subplot(1, 2, 1)
    ax0.plot(
        xyz_gt[:, 0],
        xyz_gt[:, 1],
        color=COLORS["gt"],
        lw=LINEWIDTHS["gt"],
        ls=LINESTYLES["gt"],
        label="GT (VILENS)",
        zorder=1,
        alpha=0.95,
    )
    # Draw LO first (often diverges), then LIO/LIVO on top for readability.
    draw_order = sorted(
        modes,
        key=lambda x: {"lo": 0, "lio": 1, "livo": 2}.get(x[0], 1),
    )
    for name, st, _ in draw_order:
        ax0.plot(
            st["xy_full"][:, 0],
            st["xy_full"][:, 1],
            color=COLORS.get(name, "#333333"),
            lw=LINEWIDTHS.get(name, 1.8),
            ls=LINESTYLES.get(name, "solid"),
            alpha=0.92,
            label=f"{name.upper()} (aligned)",
            zorder=2 if name == "lo" else 3,
        )
    ax0.scatter(
        xyz_gt[0, 0], xyz_gt[0, 1], c="#00A651", s=48, zorder=5, label="start",
        edgecolors="white", linewidths=0.6,
    )
    ax0.set_aspect("equal", adjustable="datalim")
    ax0.set_xlabel("x [m]")
    ax0.set_ylabel("y [m]")
    ax0.set_title("XY trajectory (Umeyama SE3 → GT)")
    ax0.grid(True, alpha=0.3)
    ax0.legend(loc="best", fontsize=8, framealpha=0.92)

    ax1 = fig.add_subplot(1, 2, 2)
    ate_lines = []
    for name, st, n_est in modes:
        ax1.plot(
            st["te"] - st["te"][0],
            st["err"],
            color=COLORS.get(name, "#333333"),
            lw=1.6,
            ls=LINESTYLES.get(name, "solid"),
            label=f"{name.upper()} RMSE={st['rmse']:.3f} m",
        )
        ate_lines.append(
            f"{name.upper()}: RMSE={st['rmse']:.3f} mean={st['mean']:.3f} "
            f"max={st['max']:.3f} matched={st['matched']}/{n_est}"
        )
    ax1.set_xlabel("time since start [s]")
    ax1.set_ylabel("ATE [m]")
    ax1.set_title("ATE vs time")
    ax1.grid(True, alpha=0.3)
    ax1.legend(loc="best", fontsize=8, framealpha=0.92)

    fig.suptitle(f"{args.title}\n" + "  |  ".join(ate_lines), fontsize=10)
    fig.tight_layout()
    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out)
    print(f"wrote {out}")
    for line in ate_lines:
        print(line)


if __name__ == "__main__":
    main()
