#!/usr/bin/env python3
"""Score Argus Spires estimates vs VILENS GT → CSV + markdown table.

Usage:
  python3 scripts/score_spires_batch.py \\
    --root /path/to/Oxford \\
    --modes-subdir modes_mf400 \\
    --out /path/to/summary.csv
"""

from __future__ import annotations

import argparse
import csv
import importlib.util
from pathlib import Path

import numpy as np

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


def score_one(est: Path, gt: Path, max_dt: float) -> dict | None:
    t_gt, xyz_gt = load_tum(gt)
    t_est, xyz_est = load_tum(est)
    if len(t_est) < 3 or len(t_gt) < 3:
        return None
    te, pe, pg = associate(t_est, xyz_est, t_gt, xyz_gt, max_dt)
    if len(te) < 3:
        return {
            "frames": len(t_est),
            "matched": len(te),
            "rmse": None,
            "mean": None,
            "max": None,
            "status": "no_assoc",
        }
    R, t = umeyama_se3(pe, pg)
    pe_a = (R @ pe.T).T + t
    err = np.linalg.norm(pe_a - pg, axis=1)
    return {
        "frames": len(t_est),
        "matched": len(te),
        "rmse": float(np.sqrt(np.mean(err**2))),
        "mean": float(np.mean(err)),
        "max": float(np.max(err)),
        "status": "ok",
    }


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--root", type=Path, required=True, help="Oxford dataset root")
    ap.add_argument("--modes-subdir", default="modes_mf400")
    ap.add_argument("--modes", default="livo,lio,lo")
    ap.add_argument("--max-dt", type=float, default=1.0)
    ap.add_argument("--out", type=Path, required=True)
    args = ap.parse_args()

    modes = [m.strip() for m in args.modes.split(",") if m.strip()]
    seq_root = args.root / "sequences"
    rows = []
    for seq in sorted(seq_root.iterdir()):
        if not seq.is_dir():
            continue
        gt = seq / "processed" / "vilens-slam" / "gt.tum"
        if not gt.is_file():
            continue
        for mode in modes:
            est = seq / "argus_eval" / args.modes_subdir / mode / "estimate.tum"
            if not est.is_file():
                rows.append(
                    {
                        "mode": mode,
                        "sequence": seq.name,
                        "frames": 0,
                        "matched": 0,
                        "rmse_m": "",
                        "mean_m": "",
                        "max_m": "",
                        "status": "missing",
                    }
                )
                continue
            st = score_one(est, gt, args.max_dt)
            if st is None:
                rows.append(
                    {
                        "mode": mode,
                        "sequence": seq.name,
                        "frames": 0,
                        "matched": 0,
                        "rmse_m": "",
                        "mean_m": "",
                        "max_m": "",
                        "status": "empty",
                    }
                )
                continue
            rows.append(
                {
                    "mode": mode,
                    "sequence": seq.name,
                    "frames": st["frames"],
                    "matched": st["matched"],
                    "rmse_m": "" if st["rmse"] is None else f"{st['rmse']:.6f}",
                    "mean_m": "" if st["mean"] is None else f"{st['mean']:.6f}",
                    "max_m": "" if st["max"] is None else f"{st['max']:.6f}",
                    "status": st["status"],
                }
            )

    args.out.parent.mkdir(parents=True, exist_ok=True)
    fields = [
        "mode",
        "sequence",
        "frames",
        "matched",
        "rmse_m",
        "mean_m",
        "max_m",
        "status",
    ]
    with args.out.open("w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fields)
        w.writeheader()
        w.writerows(rows)

    # Markdown pivot: sequences × modes RMSE
    seqs = sorted({r["sequence"] for r in rows})
    print(f"wrote {args.out} rows={len(rows)}")
    print()
    header = "| sequence | " + " | ".join(m.upper() + " RMSE [m]" for m in modes) + " |"
    sep = "|---|" + "|".join(["---:"] * len(modes)) + "|"
    print(header)
    print(sep)
    for seq in seqs:
        cells = [seq]
        for mode in modes:
            r = next(
                (x for x in rows if x["sequence"] == seq and x["mode"] == mode),
                None,
            )
            if r is None or r["status"] != "ok":
                cells.append(r["status"] if r else "—")
            else:
                cells.append(f"{float(r['rmse_m']):.3f}")
        print("| " + " | ".join(cells) + " |")

    # Per-mode mean RMSE over ok rows
    print()
    for mode in modes:
        vals = [
            float(r["rmse_m"])
            for r in rows
            if r["mode"] == mode and r["status"] == "ok" and r["rmse_m"]
        ]
        if vals:
            print(
                f"{mode.upper()}: n={len(vals)}  mean_RMSE={np.mean(vals):.3f} m  "
                f"median={np.median(vals):.3f} m  max={np.max(vals):.3f} m"
            )


if __name__ == "__main__":
    main()
