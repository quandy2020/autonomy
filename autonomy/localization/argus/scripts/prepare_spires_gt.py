#!/usr/bin/env python3
"""Ensure sequences/<name>/processed/vilens-slam/gt.tum exists.

Usage:
  python3 scripts/prepare_spires_gt.py /path/to/sequence
"""

from __future__ import annotations

import argparse
from pathlib import Path


def csv_to_tum(csv_path: Path, out: Path) -> int:
    n = 0
    lines = []
    with csv_path.open() as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#") or line.lower().startswith("counter"):
                continue
            parts = [p.strip() for p in line.split(",")]
            if len(parts) < 10:
                continue
            # counter, sec, nsec, x, y, z, qx, qy, qz, qw
            t = float(parts[1]) + float(parts[2]) * 1e-9
            x, y, z = parts[3], parts[4], parts[5]
            qx, qy, qz, qw = parts[6], parts[7], parts[8], parts[9]
            lines.append(f"{t:.9f} {x} {y} {z} {qx} {qy} {qz} {qw}\n")
            n += 1
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text("".join(lines))
    return n


def _first_tum_time(path: Path) -> float | None:
    if not path.is_file():
        return None
    for line in path.read_text().splitlines():
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        try:
            return float(line.split()[0])
        except (ValueError, IndexError):
            return None
    return None


def prepare(seq_dir: Path) -> Path:
    gt = seq_dir / "processed" / "vilens-slam" / "gt.tum"
    csv_path = seq_dir / "processed" / "vilens-slam" / "slam-poses.csv"

    # Prefer slam-poses.csv (authoritative). Rebuild when gt.tum is missing or
    # its timeline does not match the CSV (seen: stale/wrong gt.tum).
    if csv_path.is_file():
        need = True
        if gt.is_file() and gt.stat().st_size > 0:
            t_gt = _first_tum_time(gt)
            t_csv = None
            for line in csv_path.open():
                line = line.strip()
                if not line or line.startswith("#") or line.lower().startswith(
                    "counter"
                ):
                    continue
                parts = [p.strip() for p in line.split(",")]
                if len(parts) >= 3:
                    t_csv = float(parts[1]) + float(parts[2]) * 1e-9
                    break
            if t_gt is not None and t_csv is not None and abs(t_gt - t_csv) < 1.0:
                need = False
        if need:
            n = csv_to_tum(csv_path, gt)
            if n < 2:
                raise SystemExit(f"too few poses in {csv_path}")
        return gt

    if gt.is_file() and gt.stat().st_size > 0:
        return gt

    tum_txt = seq_dir / "processed" / "trajectory" / "vilens-slam-tum.txt"
    if tum_txt.is_file():
        gt.parent.mkdir(parents=True, exist_ok=True)
        gt.write_text(tum_txt.read_text())
        return gt

    raise SystemExit(f"no VILENS GT under {seq_dir}/processed")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("seq_dir", type=Path)
    args = ap.parse_args()
    out = prepare(args.seq_dir.resolve())
    n = sum(1 for line in out.read_text().splitlines() if line.strip())
    print(f"gt={out} poses={n}")


if __name__ == "__main__":
    main()
