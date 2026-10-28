#!/usr/bin/env python3
"""Evaluate an Atlas EuRoC trajectory with fixed-scale SE(3) alignment."""

import argparse
import csv
import json
import math
import re
from decimal import Decimal, InvalidOperation
from pathlib import Path

import numpy as np
from evaluate_kitti import create_plot, rigid_alignment


def parse_timestamp_ns(value: str, path: Path, line_number: int) -> int:
    """Parse an integral nanosecond timestamp without losing float precision."""
    try:
        timestamp = Decimal(value.strip())
    except InvalidOperation as error:
        raise ValueError(
            f"{path}:{line_number}: invalid timestamp {value!r}"
        ) from error
    if not timestamp.is_finite() or timestamp != timestamp.to_integral_value():
        raise ValueError(
            f"{path}:{line_number}: expected an integral nanosecond timestamp"
        )
    timestamp_ns = int(timestamp)
    if not -(2**63) <= timestamp_ns < 2**63:
        raise ValueError(f"{path}:{line_number}: timestamp is outside int64 range")
    return timestamp_ns


def quaternion_xyzw_to_matrix(quaternion: np.ndarray) -> np.ndarray:
    """Return the rotation matrix for an xyzw quaternion."""
    norm = float(np.linalg.norm(quaternion))
    if not math.isfinite(norm) or norm <= 0.0:
        raise ValueError("invalid zero or non-finite quaternion")
    x, y, z, w = quaternion / norm
    return np.asarray(
        [
            [
                1.0 - 2.0 * (y * y + z * z),
                2.0 * (x * y - z * w),
                2.0 * (x * z + y * w),
            ],
            [
                2.0 * (x * y + z * w),
                1.0 - 2.0 * (x * x + z * z),
                2.0 * (y * z - x * w),
            ],
            [
                2.0 * (x * z - y * w),
                2.0 * (y * z + x * w),
                1.0 - 2.0 * (x * x + y * y),
            ],
        ],
        dtype=float,
    )


def read_estimate(path: Path):
    """Read Atlas EuRoC rows: timestamp_ns tx ty tz qx qy qz qw."""
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
                    f"{path}:{line_number}: expected 8 fields, got {len(fields)}"
                )
            timestamp_ns = parse_timestamp_ns(fields[0], path, line_number)
            values = np.asarray([float(value) for value in fields[1:]], dtype=float)
            if not np.all(np.isfinite(values)):
                raise ValueError(f"{path}:{line_number}: pose contains non-finite data")
            rotation = quaternion_xyzw_to_matrix(values[3:7])
            raw_count += 1
            if timestamp_ns in poses:
                duplicate_count += 1
                continue
            poses[timestamp_ns] = (values[:3], rotation)

    if not poses:
        raise ValueError(f"{path}: no estimated poses")
    timestamps = np.asarray(sorted(poses), dtype=np.int64)
    positions = np.asarray([poses[int(timestamp)][0] for timestamp in timestamps])
    rotations = np.asarray([poses[int(timestamp)][1] for timestamp in timestamps])
    return raw_count, duplicate_count, timestamps, positions, rotations


def read_ground_truth(path: Path):
    """Read EuRoC state ground truth and return body positions."""
    poses = {}
    raw_count = 0
    duplicate_count = 0
    with path.open("r", encoding="utf-8") as stream:
        reader = csv.reader(stream)
        for line_number, row in enumerate(reader, 1):
            if not row or row[0].lstrip().startswith("#"):
                continue
            if len(row) < 8:
                raise ValueError(
                    f"{path}:{line_number}: expected at least 8 CSV fields"
                )
            timestamp_ns = parse_timestamp_ns(row[0], path, line_number)
            position = np.asarray([float(value) for value in row[1:4]], dtype=float)
            if not np.all(np.isfinite(position)):
                raise ValueError(
                    f"{path}:{line_number}: position contains non-finite data"
                )
            raw_count += 1
            if timestamp_ns in poses:
                duplicate_count += 1
                continue
            poses[timestamp_ns] = position

    if not poses:
        raise ValueError(f"{path}: no ground-truth poses")
    timestamps = np.asarray(sorted(poses), dtype=np.int64)
    positions = np.asarray([poses[int(timestamp)] for timestamp in timestamps])
    return raw_count, duplicate_count, timestamps, positions


def read_body_from_sensor(path: Path) -> np.ndarray:
    """Read the EuRoC T_BS sensor-to-body transform without a YAML dependency."""
    text = path.read_text(encoding="utf-8")
    match = re.search(r"T_BS:\s*.*?data:\s*\[([^\]]+)\]", text, flags=re.DOTALL)
    if match is None:
        raise ValueError(f"{path}: T_BS matrix not found")
    values = [
        float(value) for value in re.split(r"[\s,]+", match.group(1).strip()) if value
    ]
    if len(values) != 16 or not np.all(np.isfinite(values)):
        raise ValueError(f"{path}: T_BS must contain 16 finite values")
    body_from_sensor = np.asarray(values, dtype=float).reshape(4, 4)
    if not np.allclose(body_from_sensor[3], [0.0, 0.0, 0.0, 1.0], atol=1.0e-8):
        raise ValueError(f"{path}: T_BS has an invalid homogeneous bottom row")
    rotation = body_from_sensor[:3, :3]
    if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1.0e-5):
        raise ValueError(f"{path}: T_BS rotation is not orthonormal")
    if not math.isclose(float(np.linalg.det(rotation)), 1.0, abs_tol=1.0e-5):
        raise ValueError(f"{path}: T_BS rotation determinant is not one")
    return body_from_sensor


def camera_positions_to_body(
    camera_positions: np.ndarray,
    camera_rotations: np.ndarray,
    body_from_camera: np.ndarray,
) -> np.ndarray:
    """Convert T_world_camera translations to T_world_body translations."""
    camera_from_body = np.linalg.inv(body_from_camera)
    translation_camera_body = camera_from_body[:3, 3]
    return camera_positions + np.einsum(
        "nij,j->ni", camera_rotations, translation_camera_body
    )


def interpolate_ground_truth(
    ground_truth_times: np.ndarray,
    ground_truth_positions: np.ndarray,
    estimate_times: np.ndarray,
    maximum_gap_ns: int,
):
    """Linearly interpolate ground-truth positions at estimate timestamps."""
    matched_times = []
    reference_positions = []
    estimate_indices = []
    for estimate_index, timestamp in enumerate(estimate_times):
        right = int(np.searchsorted(ground_truth_times, timestamp, side="left"))
        if right < len(ground_truth_times) and ground_truth_times[right] == timestamp:
            matched_times.append(int(timestamp))
            reference_positions.append(ground_truth_positions[right])
            estimate_indices.append(estimate_index)
            continue
        if right == 0 or right == len(ground_truth_times):
            continue

        left = right - 1
        left_gap = int(timestamp - ground_truth_times[left])
        right_gap = int(ground_truth_times[right] - timestamp)
        if left_gap > maximum_gap_ns or right_gap > maximum_gap_ns:
            continue
        interval = int(ground_truth_times[right] - ground_truth_times[left])
        if interval <= 0:
            continue
        weight = left_gap / interval
        position = (1.0 - weight) * ground_truth_positions[
            left
        ] + weight * ground_truth_positions[right]
        matched_times.append(int(timestamp))
        reference_positions.append(position)
        estimate_indices.append(estimate_index)

    return (
        np.asarray(matched_times, dtype=np.int64),
        np.asarray(reference_positions, dtype=float),
        np.asarray(estimate_indices, dtype=np.int64),
    )


def write_aligned_trajectory(
    path: Path,
    matched_times: np.ndarray,
    reference: np.ndarray,
    aligned: np.ndarray,
    errors: np.ndarray,
) -> None:
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(
            [
                "timestamp_ns",
                "elapsed_s",
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
        for timestamp, ground_truth, estimate, error in zip(
            matched_times, reference, aligned, errors
        ):
            writer.writerow(
                [
                    int(timestamp),
                    f"{(int(timestamp) - origin) * 1.0e-9:.9f}",
                    *[f"{value:.9f}" for value in ground_truth],
                    *[f"{value:.9f}" for value in estimate],
                    f"{error:.9f}",
                ]
            )


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Evaluate an Atlas trajectory against EuRoC body ground truth."
    )
    parser.add_argument("ground_truth_csv", type=Path)
    parser.add_argument("estimate", type=Path)
    parser.add_argument("output_dir", type=Path)
    parser.add_argument(
        "--estimate-frame",
        choices=("body", "camera"),
        default="body",
        help="frame represented by the estimated poses (default: body)",
    )
    parser.add_argument(
        "--calibration-yaml",
        type=Path,
        help="EuRoC camera sensor.yaml containing T_BS; required for camera poses",
    )
    parser.add_argument(
        "--max-interval-ms",
        type=float,
        default=20.0,
        help="maximum gap from an estimate to each interpolation endpoint",
    )
    args = parser.parse_args()

    if not math.isfinite(args.max_interval_ms) or args.max_interval_ms <= 0.0:
        parser.error("--max-interval-ms must be finite and positive")
    if args.estimate_frame == "camera" and args.calibration_yaml is None:
        parser.error("--calibration-yaml is required for --estimate-frame camera")

    (
        raw_estimated_poses,
        duplicate_estimated_timestamps,
        estimate_times,
        estimate_positions,
        estimate_rotations,
    ) = read_estimate(args.estimate)
    (
        raw_ground_truth_poses,
        duplicate_ground_truth_timestamps,
        ground_truth_times,
        ground_truth_positions,
    ) = read_ground_truth(args.ground_truth_csv)

    if args.estimate_frame == "camera":
        body_from_camera = read_body_from_sensor(args.calibration_yaml)
        estimate_positions = camera_positions_to_body(
            estimate_positions, estimate_rotations, body_from_camera
        )

    maximum_gap_ns = round(args.max_interval_ms * 1.0e6)
    matched_times, reference, estimate_indices = interpolate_ground_truth(
        ground_truth_times,
        ground_truth_positions,
        estimate_times,
        maximum_gap_ns,
    )
    if len(matched_times) < 3:
        raise ValueError(
            "fewer than three poses were associated; check timestamps and "
            "--max-interval-ms"
        )

    matched_estimate = estimate_positions[estimate_indices]
    alignment_rotation, alignment_translation = rigid_alignment(
        matched_estimate, reference
    )
    aligned = (alignment_rotation @ matched_estimate.T).T + alignment_translation
    errors = np.linalg.norm(aligned - reference, axis=1)
    duration_seconds = float(int(matched_times[-1]) - int(matched_times[0])) * 1.0e-9

    metrics = {
        "alignment": "SE3",
        "alignment_rotation": alignment_rotation.tolist(),
        "alignment_translation_m": alignment_translation.tolist(),
        "association": "linear_interpolation",
        "maximum_interval_ms": float(args.max_interval_ms),
        "scale_fixed": True,
        "estimate_frame": args.estimate_frame,
        "calibration_yaml": (
            str(args.calibration_yaml) if args.calibration_yaml is not None else None
        ),
        "raw_ground_truth_poses": int(raw_ground_truth_poses),
        "ground_truth_poses": len(ground_truth_times),
        "duplicate_ground_truth_timestamps": int(duplicate_ground_truth_timestamps),
        "raw_estimated_poses": int(raw_estimated_poses),
        "estimated_poses": len(estimate_times),
        "duplicate_estimated_timestamps": int(duplicate_estimated_timestamps),
        "matched_poses": len(matched_times),
        "unmatched_estimated_poses": int(len(estimate_times) - len(matched_times)),
        "coverage_percent": float(100.0 * len(matched_times) / len(estimate_times)),
        "first_matched_timestamp_ns": int(matched_times[0]),
        "last_matched_timestamp_ns": int(matched_times[-1]),
        "matched_duration_s": duration_seconds,
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
