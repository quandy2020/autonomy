#!/usr/bin/env python3
"""Run isolated, reproducible 2D Cartographer parameter comparisons from a record."""
import argparse
import csv
import itertools
import json
import math
import os
from pathlib import Path
import shutil
import subprocess
import time


def values(text):
    result = [float(v) for v in text.split(",")]
    if not result or any(not math.isfinite(v) or v <= 0 for v in result):
        raise argparse.ArgumentTypeError("use comma-separated positive finite numbers")
    return list(dict.fromkeys(result))


def evaluate(estimate_path, truth_path, tolerance=0.05):
    """Time-associate TUM poses, align in SE(2), then measure XY ATE and 1 s RPE."""
    import numpy as np

    def load(path):
        data = np.loadtxt(path, comments="#", ndmin=2)
        if data.shape[1] != 8 or len(data) < 3 or not np.isfinite(data).all():
            raise ValueError("TUM requires at least three finite timestamp/x/y/z/qx/qy/qz/qw rows")
        if np.any(np.diff(data[:, 0]) <= 0):
            raise ValueError("TUM timestamps must be strictly increasing")
        norm = np.linalg.norm(data[:, 4:8], axis=1)
        if np.any(norm < 1e-9):
            raise ValueError("Invalid TUM quaternion")
        q = data[:, 4:8] / norm[:, None]
        yaw = np.arctan2(2 * (q[:, 3] * q[:, 2] + q[:, 0] * q[:, 1]),
                         1 - 2 * (q[:, 1] ** 2 + q[:, 2] ** 2))
        return data, yaw

    estimate, estimate_yaw = load(estimate_path)
    truth, truth_yaw = load(truth_path)
    right = np.clip(np.searchsorted(truth[:, 0], estimate[:, 0]), 0, len(truth) - 1)
    left = np.maximum(right - 1, 0)
    best = np.where(abs(truth[left, 0] - estimate[:, 0]) <= abs(truth[right, 0] - estimate[:, 0]), left, right)
    # Keep one estimate per truth sample, choosing the smallest timestamp error.
    associations = {}
    for i, j in enumerate(best):
        error = abs(estimate[i, 0] - truth[j, 0])
        if error <= tolerance and (j not in associations or error < associations[j][1]):
            associations[j] = (i, error)
    pairs = sorted((i, j) for j, (i, _) in associations.items())
    if len(pairs) < 3:
        raise ValueError("Fewer than three timestamp-matched poses; check clocks and body frames")
    ei, gi = np.array(pairs).T
    x, y = estimate[ei, 1:3], truth[gi, 1:3]
    if np.linalg.norm(x - x.mean(0)) < 1e-9 or np.linalg.norm(y - y.mean(0)) < 1e-9:
        raise ValueError("A stationary trajectory cannot determine SE(2) alignment")
    u, _, vt = np.linalg.svd((x - x.mean(0)).T @ (y - y.mean(0)))
    correction = np.diag([1., np.linalg.det(vt.T @ u.T)])
    rotation = vt.T @ correction @ u.T
    aligned = (x - x.mean(0)) @ rotation.T + y.mean(0)
    metrics = {"matched_poses": len(pairs), "ate_xy_rmse_m": float(np.sqrt(np.mean(np.sum((aligned - y) ** 2, axis=1))))}
    t = estimate[ei, 0]
    end = np.clip(np.searchsorted(t, t + 1.), 0, len(t) - 1)
    previous = np.maximum(end - 1, 0)
    end = np.where(abs(t[previous] - t - 1.) < abs(t[end] - t - 1.), previous, end)
    start = np.arange(len(t))
    valid = (end > start) & (abs(t[end] - t - 1.) <= tolerance)
    start, end = start[valid], end[valid]
    if len(start):
        def relative(points, yaw):
            delta = points[end] - points[start]
            c, s = np.cos(yaw[start]), np.sin(yaw[start])
            return np.column_stack((c * delta[:, 0] + s * delta[:, 1], -s * delta[:, 0] + c * delta[:, 1]))
        a, b = relative(x, estimate_yaw[ei]), relative(y, truth_yaw[gi])
        angle = ((estimate_yaw[ei][end] - estimate_yaw[ei][start]) -
                 (truth_yaw[gi][end] - truth_yaw[gi][start]))
        angle = np.arctan2(np.sin(angle), np.cos(angle))
        metrics.update(rpe_pairs=len(start), rpe_xy_1s_rmse_m=float(np.sqrt(np.mean(np.sum((a - b) ** 2, axis=1)))),
                       rpe_yaw_1s_rmse_rad=float(np.sqrt(np.mean(angle ** 2))))
    return metrics


def main():
    root = Path(__file__).resolve().parents[5]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--record", type=Path, required=True)
    parser.add_argument("--config", type=Path, default=Path(__file__).resolve().parents[1] / "conf/cartographer/robot_2d_ekf.lua")
    binary = shutil.which("autonomy.cartographer_benchmark") or root / "build/autonomy/bin/autonomy.cartographer_benchmark"
    parser.add_argument("--binary", type=Path, default=binary)
    parser.add_argument("--phase", choices=("frontend", "global"), default="frontend")
    parser.add_argument("--voxel-sizes", type=values, default=values("0.025,0.02,0.03"))
    parser.add_argument("--translation-weights", type=values, default=values("10,5,20"))
    parser.add_argument("--min-scores", type=values, default=values("0.55,0.6"))
    parser.add_argument("--sampling-ratios", type=values, default=values("0.3,0.1"))
    parser.add_argument("--seconds", type=float, default=60., help="0 = whole record")
    parser.add_argument("--timeout", type=float, default=600., help="Wall-clock limit per candidate")
    parser.add_argument("--static-transforms", type=Path)
    parser.add_argument("--ground-truth", type=Path, help="Independent TUM ground truth in the same sensor/body frame")
    parser.add_argument("--output", type=Path, help="New output directory; defaults to ~/.autonomy/artifacts/cartographer-tuning")
    args = parser.parse_args()
    for path in (args.record, args.config, args.binary, args.static_transforms, args.ground_truth):
        if path is not None and not path.is_file():
            parser.error(f"File not found: {path}")
    if not math.isfinite(args.seconds) or args.seconds < 0 or not math.isfinite(args.timeout) or args.timeout <= 0:
        parser.error("Invalid duration/timeout")
    if any(v > 1 for v in args.min_scores + args.sampling_ratios):
        parser.error("Scores and sampling ratios must be <= 1")
    name = time.strftime("%Y%m%d-%H%M%S") + f"-{os.getpid()}"
    home = Path.home() / ".autonomy"
    output = (args.output or home / "artifacts/cartographer-tuning" / name).resolve()
    output.mkdir(parents=True, exist_ok=False)
    logs = home / "log/cartographer-tuning" / name
    logs.mkdir(parents=True, exist_ok=False)
    config = output / "config"
    config.mkdir()
    for path in args.config.parent.glob("*.lua"):
        shutil.copy2(path, config / path.name)
    mount = args.static_transforms or args.config.with_name(args.config.stem + "_static_transform.yaml")
    if mount.exists():
        shutil.copy2(mount, config / "candidate_static_transform.yaml")
    if args.phase == "frontend":
        keys = ("TRAJECTORY_BUILDER_2D.voxel_filter_size", "TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight")
        grid = itertools.product(args.voxel_sizes, args.translation_weights)
    else:
        keys = ("POSE_GRAPH.constraint_builder.min_score", "POSE_GRAPH.constraint_builder.sampling_ratio")
        grid = itertools.product(args.min_scores, args.sampling_ratios)
    rows = []
    print(f"Results: {output}\nLogs: {logs}", flush=True)
    for index, parameters in enumerate(grid):
        run = f"{index:02d}"
        lua = f'include "{args.config.name}"\n'
        lua += "POSE_GRAPH.constraint_builder.log_matches = false\nPOSE_GRAPH.log_residual_histograms = false\n"
        if args.phase == "frontend":
            lua += "POSE_GRAPH.optimize_every_n_nodes = 0\n"
        for key, value in zip(keys, parameters):
            lua += f"{key} = {value!r}\n"
        lua += "return options\n"
        (config / "candidate.lua").write_text(lua)
        run_dir = output / run
        log = logs / f"{run}.log"
        command = [str(args.binary.resolve()), "--record", str(args.record.resolve()),
                   "--configuration_directory", str(config), "--configuration_basename", "candidate.lua",
                   "--output_dir", str(run_dir), "--seconds", str(args.seconds)]
        # Snapshot the effective candidate beside its results before reusing candidate.lua.
        run_dir.mkdir()
        snapshot = run_dir / "candidate.lua"
        snapshot.write_text(lua)
        # The native runner requires an empty destination.
        native_output = run_dir / "result"
        command[command.index("--output_dir") + 1] = str(native_output)
        row = {"run": run, "phase": args.phase, "parameter_1": keys[0], "value_1": parameters[0],
               "parameter_2": keys[1], "value_2": parameters[1], "log": str(log)}
        try:
            with log.open("w") as stream:
                subprocess.run(command, stdout=stream, stderr=subprocess.STDOUT, timeout=args.timeout, check=True)
            row.update(json.loads((native_output / "summary.json").read_text()), status="ok")
            if args.phase == "global" and not row["global_optimization"]:
                raise ValueError("Global comparison requires optimize_every_n_nodes > 0 in the base config")
            if args.ground_truth:
                trajectory = "local.tum" if args.phase == "frontend" else "trajectory.tum"
                row.update(evaluate(native_output / trajectory, args.ground_truth))
        except (OSError, ValueError, subprocess.SubprocessError) as error:
            row.update(status="failed", error=str(error))
        rows.append(row)
        (output / "results.json").write_text(json.dumps(rows, indent=2, allow_nan=False) + "\n")
        fields = list(dict.fromkeys(key for entry in rows for key in entry))
        with (output / "results.csv").open("w", newline="") as stream:
            writer = csv.DictWriter(stream, fields); writer.writeheader(); writer.writerows(rows)
        print(f"{run}: {parameters}, status={row['status']}, realtime_factor={row.get('realtime_factor', 'n/a')}", flush=True)
    if not args.ground_truth:
        print("No independent ground truth: accuracy is not ranked; compare latency and inspect saved maps.")
    return int(any(row["status"] != "ok" for row in rows))


if __name__ == "__main__":
    raise SystemExit(main())
