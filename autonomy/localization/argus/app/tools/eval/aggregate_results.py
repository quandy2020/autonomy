#!/usr/bin/env python3
"""Aggregate trajectory.tum files under a results directory into summary.yaml.

ATE is the RMSE of translation after a rigid alignment (no scale) when numpy
is available. Without numpy the RMSE is computed in the original frame.
Association uses the nearest ground-truth stamp within --max-dt seconds.
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path


def load_tum(path: Path) -> list[tuple[float, tuple[float, float, float]]]:
    poses = []
    for raw in path.read_text().splitlines():
        line = raw.strip()
        if not line or line.startswith("#"):
            continue
        parts = line.split()
        poses.append((float(parts[0]), (float(parts[1]), float(parts[2]), float(parts[3]))))
    return poses


def associate(estimate, ground_truth, max_dt: float):
    if not ground_truth:
        return []
    pairs = []
    index = 0
    for timestamp, position in estimate:
        while index + 1 < len(ground_truth) and abs(ground_truth[index + 1][0] - timestamp) <= abs(
            ground_truth[index][0] - timestamp
        ):
            index += 1
        if abs(ground_truth[index][0] - timestamp) <= max_dt:
            pairs.append((position, ground_truth[index][1]))
    return pairs


def _centroid(points):
    count = float(len(points))
    return tuple(sum(point[axis] for point in points) / count for axis in range(3))


def _sub(a, b):
    return tuple(a[axis] - b[axis] for axis in range(3))


def _add(a, b):
    return tuple(a[axis] + b[axis] for axis in range(3))


def _dot(a, b):
    return sum(a[axis] * b[axis] for axis in range(3))


def _matmul(matrix, vector):
    return tuple(_dot(row, vector) for row in matrix)


def align_rigid(pairs):
    try:
        import numpy as np
    except ImportError:
        return None
    estimate = np.array([pair[0] for pair in pairs], dtype=float)
    ground_truth = np.array([pair[1] for pair in pairs], dtype=float)
    mean_estimate = estimate.mean(axis=0)
    mean_ground_truth = ground_truth.mean(axis=0)
    centered_estimate = estimate - mean_estimate
    centered_ground_truth = ground_truth - mean_ground_truth
    covariance = centered_estimate.T @ centered_ground_truth
    u, _s, vt = np.linalg.svd(covariance)
    rotation = vt.T @ u.T
    if np.linalg.det(rotation) < 0.0:
        vt[-1, :] *= -1.0
        rotation = vt.T @ u.T
    translation = mean_ground_truth - rotation @ mean_estimate
    return rotation, translation


def rmse(pairs, aligned: bool) -> tuple[float, bool]:
    if len(pairs) < 2:
        raise SystemExit("need at least 2 associated poses to score a run")
    transform = align_rigid(pairs) if aligned else None
    errors = []
    if transform is None:
        for estimate, ground_truth in pairs:
            delta = _sub(estimate, ground_truth)
            errors.append(math.sqrt(_dot(delta, delta)))
        return math.sqrt(sum(error * error for error in errors) / len(errors)), False
    rotation, translation = transform
    rotation_rows = [tuple(rotation[row, col] for col in range(3)) for row in range(3)]
    shift = tuple(float(translation[axis]) for axis in range(3))
    for estimate, ground_truth in pairs:
        mapped = _add(_matmul(rotation_rows, estimate), shift)
        delta = _sub(mapped, ground_truth)
        errors.append(math.sqrt(_dot(delta, delta)))
    return math.sqrt(sum(error * error for error in errors) / len(errors)), True


def yaml_scalar(value) -> str:
    if isinstance(value, bool):
        return "true" if value else "false"
    if isinstance(value, float):
        return f"{value:.6f}"
    return str(value)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("results", type=Path, help="directory of per-config run folders")
    parser.add_argument("--ground-truth", type=Path, help="TUM file applied to every run")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--max-dt", type=float, default=0.05)
    parser.add_argument("--no-align", action="store_true")
    args = parser.parse_args(argv)

    runs = sorted(path for path in args.results.iterdir() if path.is_dir())
    if not runs:
        raise SystemExit(f"no run directories under {args.results}")
    shared_ground_truth = load_tum(args.ground_truth) if args.ground_truth else None
    blocks = []
    for run in runs:
        trajectory = run / "trajectory.tum"
        if not trajectory.is_file():
            raise SystemExit(f"missing {trajectory}")
        ground_truth_path = run / "groundtruth.tum"
        if shared_ground_truth is not None:
            ground_truth = shared_ground_truth
        elif ground_truth_path.is_file():
            ground_truth = load_tum(ground_truth_path)
        else:
            raise SystemExit(f"{run} has no ground truth (pass --ground-truth or groundtruth.tum)")
        pairs = associate(load_tum(trajectory), ground_truth, args.max_dt)
        error, did_align = rmse(pairs, aligned=not args.no_align)
        blocks.append(
            {
                "name": run.name,
                "poses": len(pairs),
                "ate_rmse_m": error,
                "aligned": did_align,
            }
        )
    lines = ["runs:"]
    for block in blocks:
        lines.append(f"  - name: {block['name']}")
        lines.append(f"    poses: {block['poses']}")
        lines.append(f"    ate_rmse_m: {yaml_scalar(block['ate_rmse_m'])}")
        lines.append(f"    aligned: {yaml_scalar(block['aligned'])}")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text("\n".join(lines) + "\n")
    print(f"wrote {args.output} ({len(blocks)} runs)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
