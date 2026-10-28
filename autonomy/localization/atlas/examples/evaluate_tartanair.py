#!/usr/bin/env python3
"""Evaluate an Atlas TartanAir trajectory with fixed-scale SE(3) alignment.

TartanAir V2 stores each camera pose as 'x y z qx qy qz qw' in the
AirSim NED world frame. Camera timestamps live in 'imu/cam_time.txt'.
Atlas writes TUM 'timestamp tx ty tz qx qy qz qw' camera poses. Both inputs
therefore represent world-from-left-camera poses; only a rigid world-frame
alignment is applied. Scale is never fitted.
"""

import argparse
import csv
import json
import math
from decimal import ROUND_HALF_EVEN, Decimal, InvalidOperation
from pathlib import Path

import numpy as np
from evaluate_euroc import quaternion_xyzw_to_matrix
from evaluate_kitti import create_plot, rigid_alignment

NANOSECONDS_PER_SECOND = Decimal(1_000_000_000)


def parse_seconds_ns(value: str, path: Path, line_number: int) -> int:
    """Convert a decimal seconds timestamp to integer nanoseconds."""
    try:
        seconds = Decimal(value.strip())
    except InvalidOperation as error:
        raise ValueError(
            f"{path}:{line_number}: invalid timestamp {value!r}"
        ) from error
    if not seconds.is_finite():
        raise ValueError(f"{path}:{line_number}: timestamp is not finite")
    timestamp_ns = int(
        (seconds * NANOSECONDS_PER_SECOND).to_integral_value(rounding=ROUND_HALF_EVEN)
    )
    if not -(2**63) <= timestamp_ns < 2**63:
        raise ValueError(f"{path}:{line_number}: timestamp is outside int64 range")
    return timestamp_ns


def read_camera_times(path: Path) -> np.ndarray:
    """Read the strictly increasing TartanAir camera timestamp vector."""
    timestamps = []
    with path.open("r", encoding="utf-8") as stream:
        for line_number, raw_line in enumerate(stream, 1):
            line = raw_line.strip()
            if not line or line.startswith("#"):
                continue
            fields = line.split()
            if len(fields) != 1:
                raise ValueError(f"{path}:{line_number}: expected one timestamp field")
            timestamps.append(parse_seconds_ns(fields[0], path, line_number))
    if not timestamps:
        raise ValueError(f"{path}: no camera timestamps")
    result = np.asarray(timestamps, dtype=np.int64)
    if np.any(np.diff(result) <= 0):
        raise ValueError(f"{path}: camera timestamps must be strictly increasing")
    return result


def read_ground_truth(path: Path, camera_times: np.ndarray) -> np.ndarray:
    """Read TartanAir x y z qx qy qz qw left-camera poses."""
    positions = []
    with path.open("r", encoding="utf-8") as stream:
        for line_number, raw_line in enumerate(stream, 1):
            line = raw_line.strip()
            if not line or line.startswith("#"):
                continue
            fields = line.replace(",", " ").split()
            if len(fields) != 7:
                raise ValueError(
                    f"{path}:{line_number}: expected 7 fields (x y z qx qy qz qw)"
                )
            values = np.asarray([float(value) for value in fields], dtype=float)
            if not np.all(np.isfinite(values)):
                raise ValueError(f"{path}:{line_number}: pose is not finite")
            quaternion_xyzw_to_matrix(values[3:7])
            positions.append(values[:3])
    if not positions:
        raise ValueError(f"{path}: no ground-truth poses")
    if len(positions) != len(camera_times):
        raise ValueError(
            f"ground-truth pose/time counts differ "
            f"(poses={len(positions)}, times={len(camera_times)})"
        )
    return np.asarray(positions, dtype=float)


def read_estimate(path: Path):
    """Read a TUM seconds tx ty tz qx qy qz qw trajectory."""
    poses = {}
    raw_count = 0
    duplicate_count = 0
    with path.open("r", encoding="utf-8") as stream:
        for line_number, raw_line in enumerate(stream, 1):
            line = raw_line.strip()
            if not line or line.startswith("#"):
                continue
            fields = line.replace(",", " ").split()
            if len(fields) != 8:
                raise ValueError(
                    f"{path}:{line_number}: expected 8 TUM fields, got {len(fields)}"
                )
            timestamp_ns = parse_seconds_ns(fields[0], path, line_number)
            values = np.asarray([float(value) for value in fields[1:]], dtype=float)
            if not np.all(np.isfinite(values)):
                raise ValueError(f"{path}:{line_number}: pose is not finite")
            quaternion_xyzw_to_matrix(values[3:7])
            raw_count += 1
            if timestamp_ns in poses:
                duplicate_count += 1
                continue
            poses[timestamp_ns] = values[:3]
    if not poses:
        raise ValueError(f"{path}: no estimated poses")
    timestamps = np.asarray(sorted(poses), dtype=np.int64)
    positions = np.asarray([poses[int(timestamp)] for timestamp in timestamps])
    return raw_count, duplicate_count, timestamps, positions


def associate_nearest(
    ground_truth_times: np.ndarray,
    ground_truth_positions: np.ndarray,
    estimate_times: np.ndarray,
    maximum_delta_ns: int,
):
    """Associate each estimate with one nearest ground-truth timestamp."""
    matched_times = []
    reference_positions = []
    estimate_indices = []
    deltas_ns = []
    last_ground_truth_index = -1
    for estimate_index, timestamp in enumerate(estimate_times):
        right = int(np.searchsorted(ground_truth_times, timestamp, side="left"))
        candidates = []
        if right < len(ground_truth_times):
            candidates.append(right)
        if right > 0:
            candidates.append(right - 1)
        if not candidates:
            continue
        ground_truth_index = min(
            candidates,
            key=lambda index: abs(int(ground_truth_times[index]) - int(timestamp)),
        )
        delta_ns = abs(int(ground_truth_times[ground_truth_index]) - int(timestamp))
        if delta_ns > maximum_delta_ns or ground_truth_index <= last_ground_truth_index:
            continue
        last_ground_truth_index = ground_truth_index
        matched_times.append(int(timestamp))
        reference_positions.append(ground_truth_positions[ground_truth_index])
        estimate_indices.append(estimate_index)
        deltas_ns.append(delta_ns)
    return (
        np.asarray(matched_times, dtype=np.int64),
        np.asarray(reference_positions, dtype=float),
        np.asarray(estimate_indices, dtype=np.int64),
        np.asarray(deltas_ns, dtype=np.int64),
    )


def write_aligned_trajectory(
    path: Path,
    matched_times: np.ndarray,
    association_deltas_ns: np.ndarray,
    reference: np.ndarray,
    aligned: np.ndarray,
    errors: np.ndarray,
) -> None:
    """Write associated fixed-scale positions and per-frame ATE."""
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(
            [
                "timestamp_ns",
                "elapsed_s",
                "association_delta_ns",
                "gt_x_m",
                "gt_y_m",
                "gt_z_m",
                "estimate_aligned_x_m",
                "estimate_aligned_y_m",
                "estimate_aligned_z_m",
                "ate_m",
            ]
        )
        origin = int(matched_times[0])
        for timestamp, delta_ns, ground_truth, estimate, error in zip(
            matched_times, association_deltas_ns, reference, aligned, errors
        ):
            writer.writerow(
                [
                    int(timestamp),
                    f"{(int(timestamp) - origin) * 1.0e-9:.9f}",
                    int(delta_ns),
                    *[f"{value:.9f}" for value in ground_truth],
                    *[f"{value:.9f}" for value in estimate],
                    f"{error:.9f}",
                ]
            )


def path_length(positions: np.ndarray) -> float:
    """Return translation path length without fitting scale."""
    if len(positions) < 2:
        return 0.0
    return float(np.linalg.norm(np.diff(positions, axis=0), axis=1).sum())


def main() -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Evaluate Atlas TUM output against TartanAir left-camera poses "
            "using fixed-scale SE(3) alignment."
        )
    )
    parser.add_argument(
        "ground_truth_pose",
        type=Path,
        help="TartanAir pose_lcam_front.txt (x y z qx qy qz qw)",
    )
    parser.add_argument(
        "camera_times",
        type=Path,
        help="TartanAir imu/cam_time.txt in seconds",
    )
    parser.add_argument("estimate", type=Path, help="Atlas TUM trajectory")
    parser.add_argument("output_dir", type=Path)
    parser.add_argument(
        "--max-time-delta-ms",
        type=float,
        default=1.0,
        help="maximum nearest-timestamp association delta (default: 1)",
    )
    args = parser.parse_args()
    if not math.isfinite(args.max_time_delta_ms) or args.max_time_delta_ms <= 0.0:
        parser.error("--max-time-delta-ms must be finite and positive")

    camera_times = read_camera_times(args.camera_times)
    ground_truth_positions = read_ground_truth(args.ground_truth_pose, camera_times)
    (
        raw_estimated_poses,
        duplicate_estimated_timestamps,
        estimate_times,
        estimate_positions,
    ) = read_estimate(args.estimate)

    maximum_delta_ns = round(args.max_time_delta_ms * 1.0e6)
    matched_times, reference, estimate_indices, association_deltas_ns = (
        associate_nearest(
            camera_times,
            ground_truth_positions,
            estimate_times,
            maximum_delta_ns,
        )
    )
    if len(matched_times) < 3:
        raise ValueError(
            "fewer than three poses were associated; check timestamp origins "
            "and --max-time-delta-ms"
        )

    matched_estimate = estimate_positions[estimate_indices]
    alignment_rotation, alignment_translation = rigid_alignment(
        matched_estimate, reference
    )
    aligned = (alignment_rotation @ matched_estimate.T).T + alignment_translation
    errors = np.linalg.norm(aligned - reference, axis=1)
    ground_truth_length = path_length(reference)
    estimate_length = path_length(matched_estimate)
    length_ratio = (
        estimate_length / ground_truth_length if ground_truth_length > 0.0 else None
    )

    metrics = {
        "alignment": "SE3",
        "scale_fixed": True,
        "scale_fitted": False,
        "alignment_rotation": alignment_rotation.tolist(),
        "alignment_translation_m": alignment_translation.tolist(),
        "association": "nearest_timestamp",
        "maximum_time_delta_ms": float(args.max_time_delta_ms),
        "association_mean_delta_ms": float(np.mean(association_deltas_ns)) * 1.0e-6,
        "association_max_delta_ms": float(np.max(association_deltas_ns)) * 1.0e-6,
        "ground_truth_pose_order": "x y z qx qy qz qw",
        "ground_truth_world_frame": "AirSim NED",
        "ground_truth_pose_file": str(args.ground_truth_pose),
        "camera_times_file": str(args.camera_times),
        "estimate_file": str(args.estimate),
        "ground_truth_poses": len(camera_times),
        "raw_estimated_poses": int(raw_estimated_poses),
        "estimated_poses": len(estimate_times),
        "duplicate_estimated_timestamps": int(duplicate_estimated_timestamps),
        "matched_poses": len(matched_times),
        "unmatched_estimated_poses": int(len(estimate_times) - len(matched_times)),
        "estimate_coverage_percent": float(
            100.0 * len(matched_times) / len(estimate_times)
        ),
        "ground_truth_coverage_percent": float(
            100.0 * len(matched_times) / len(camera_times)
        ),
        "matched_duration_s": float(int(matched_times[-1]) - int(matched_times[0]))
        * 1.0e-9,
        "ground_truth_path_length_m": ground_truth_length,
        "estimate_path_length_m_before_alignment": estimate_length,
        "path_length_ratio_no_scale_fit": length_ratio,
        "ate_rmse_m": float(np.sqrt(np.mean(errors**2))),
        "ate_mean_m": float(np.mean(errors)),
        "ate_median_m": float(np.median(errors)),
        "ate_std_m": float(np.std(errors)),
        "ate_min_m": float(np.min(errors)),
        "ate_max_m": float(np.max(errors)),
    }

    args.output_dir.mkdir(parents=True, exist_ok=True)
    with (args.output_dir / "metrics.json").open("w", encoding="utf-8") as stream:
        json.dump(metrics, stream, indent=2, sort_keys=True)
        stream.write("\n")
    write_aligned_trajectory(
        args.output_dir / "aligned_trajectory.csv",
        matched_times,
        association_deltas_ns,
        reference,
        aligned,
        errors,
    )
    create_plot(
        args.output_dir / "trajectory_and_ate.png",
        reference,
        aligned,
        errors,
        metrics["ate_rmse_m"],
        axes=(0, 1),
    )
    print(json.dumps(metrics, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
