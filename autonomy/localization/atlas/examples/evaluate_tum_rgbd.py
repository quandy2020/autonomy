#!/usr/bin/env python3
"""Evaluate a metric TUM RGB-D trajectory with fixed-scale SE(3) alignment."""

from __future__ import annotations

import argparse
import bisect
import csv
import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Sequence

import numpy as np

from evaluate_kitti import create_plot, rigid_alignment


@dataclass(frozen=True)
class TimedPose:
    timestamp: float
    translation: np.ndarray


def read_trajectory(path: Path) -> tuple[list[TimedPose], int]:
    """Read timestamp tx ty tz qx qy qz qw rows in timestamp order."""
    poses: dict[float, TimedPose] = {}
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
            values = np.asarray([float(value) for value in fields], dtype=float)
            if not np.all(np.isfinite(values)):
                raise ValueError(f"{path}:{line_number}: pose contains non-finite data")
            if float(np.linalg.norm(values[4:8])) <= 1.0e-12:
                raise ValueError(f"{path}:{line_number}: quaternion has zero norm")
            timestamp = float(values[0])
            if timestamp in poses:
                duplicate_count += 1
                continue
            poses[timestamp] = TimedPose(timestamp, values[1:4])
    if not poses:
        raise ValueError(f"{path}: no poses")
    return [poses[timestamp] for timestamp in sorted(poses)], duplicate_count


def read_expected_timestamps(path: Path) -> list[float]:
    """Read unique RGB timestamps from a TUM association file."""
    timestamps = set()
    with path.open("r", encoding="utf-8") as stream:
        for line_number, raw_line in enumerate(stream, 1):
            line = raw_line.strip()
            if not line or line.startswith("#"):
                continue
            fields = line.replace(",", " ").split()
            if len(fields) < 1:
                continue
            timestamp = float(fields[0])
            if not math.isfinite(timestamp):
                raise ValueError(
                    f"{path}:{line_number}: timestamp must be finite"
                )
            timestamps.add(timestamp)
    if not timestamps:
        raise ValueError(f"{path}: no expected timestamps")
    return sorted(timestamps)


def supported_timestamp_indices(
    reference_times: Sequence[float],
    timestamps: Sequence[float],
    maximum_difference: float,
) -> set[int]:
    """Return input indices with nearby ground truth, without consuming it."""
    supported = set()
    for timestamp_index, timestamp in enumerate(timestamps):
        index = bisect.bisect_left(reference_times, timestamp)
        candidates = []
        if index < len(reference_times):
            candidates.append(reference_times[index])
        if index > 0:
            candidates.append(reference_times[index - 1])
        nearest_difference = min(
            (abs(value - timestamp) for value in candidates), default=math.inf
        )
        if nearest_difference <= maximum_difference:
            supported.add(timestamp_index)
    return supported


def associate_timestamps(
    reference_times: Sequence[float],
    estimate_times: Sequence[float],
    maximum_difference: float,
) -> list[tuple[int, int]]:
    """Return deterministic one-to-one nearest timestamp associations."""
    if not math.isfinite(maximum_difference) or maximum_difference <= 0.0:
        raise ValueError("maximum_difference must be finite and positive")
    candidates = []
    for estimate_index, timestamp in enumerate(estimate_times):
        begin = bisect.bisect_left(
            reference_times, timestamp - maximum_difference
        )
        end = bisect.bisect_right(
            reference_times, timestamp + maximum_difference
        )
        for reference_index in range(begin, end):
            candidates.append(
                (
                    abs(reference_times[reference_index] - timestamp),
                    reference_index,
                    estimate_index,
                )
            )
    candidates.sort()
    used_reference = set()
    used_estimate = set()
    matches = []
    for _, reference_index, estimate_index in candidates:
        if reference_index in used_reference or estimate_index in used_estimate:
            continue
        used_reference.add(reference_index)
        used_estimate.add(estimate_index)
        matches.append((reference_index, estimate_index))
    matches.sort(key=lambda pair: estimate_times[pair[1]])
    return matches


def associate_poses(
    reference: Sequence[TimedPose],
    estimate: Sequence[TimedPose],
    maximum_difference: float,
) -> list[tuple[int, int]]:
    return associate_timestamps(
        [pose.timestamp for pose in reference],
        [pose.timestamp for pose in estimate],
        maximum_difference,
    )


def write_aligned_trajectory(
    path: Path,
    reference: np.ndarray,
    aligned: np.ndarray,
    errors: np.ndarray,
    timestamps: Sequence[float],
) -> None:
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(
            [
                "timestamp_s",
                "gt_x_m",
                "gt_y_m",
                "gt_z_m",
                "estimate_aligned_x_m",
                "estimate_aligned_y_m",
                "estimate_aligned_z_m",
                "ate_m",
            ]
        )
        for timestamp, ground_truth, estimate, error in zip(
            timestamps, reference, aligned, errors
        ):
            writer.writerow(
                [
                    f"{timestamp:.9f}",
                    *[f"{value:.9f}" for value in ground_truth],
                    *[f"{value:.9f}" for value in estimate],
                    f"{error:.9f}",
                ]
            )


def evaluate(
    ground_truth_path: Path,
    estimate_path: Path,
    maximum_difference: float,
    expected_timestamps_path: Path | None = None,
) -> tuple[
    dict[str, float | int | bool | str],
    np.ndarray,
    np.ndarray,
    np.ndarray,
    list[float],
]:
    ground_truth, duplicate_ground_truth = read_trajectory(ground_truth_path)
    estimate, duplicate_estimate = read_trajectory(estimate_path)
    matches = associate_poses(ground_truth, estimate, maximum_difference)
    if len(matches) < 3:
        raise ValueError("at least three associated poses are required")

    reference_xyz = np.asarray(
        [ground_truth[reference_index].translation for reference_index, _ in matches]
    )
    estimate_xyz = np.asarray(
        [estimate[estimate_index].translation for _, estimate_index in matches]
    )
    rotation, translation = rigid_alignment(estimate_xyz, reference_xyz)
    aligned_xyz = (rotation @ estimate_xyz.T).T + translation
    errors = np.linalg.norm(aligned_xyz - reference_xyz, axis=1)
    matched_timestamps = [estimate[index].timestamp for _, index in matches]

    expected_timestamps = (
        [pose.timestamp for pose in estimate]
        if expected_timestamps_path is None
        else read_expected_timestamps(expected_timestamps_path)
    )
    expected_pose_count = len(expected_timestamps)
    estimate_timestamps = [pose.timestamp for pose in estimate]
    tracked_expected_matches = associate_timestamps(
        expected_timestamps, estimate_timestamps, maximum_difference
    )
    supported_expected_indices = supported_timestamp_indices(
        [pose.timestamp for pose in ground_truth],
        expected_timestamps,
        maximum_difference,
    )
    matched_estimate_indices = {estimate_index for _, estimate_index in matches}
    associated_supported_pose_count = sum(
        expected_index in supported_expected_indices
        and estimate_index in matched_estimate_indices
        for expected_index, estimate_index in tracked_expected_matches
    )
    supported_expected_pose_count = len(supported_expected_indices)
    tracking_output_coverage = min(
        100.0, 100.0 * len(tracked_expected_matches) / expected_pose_count
    )
    ground_truth_support_fraction = (
        100.0 * supported_expected_pose_count / expected_pose_count
    )
    association_coverage = (
        100.0 * associated_supported_pose_count / supported_expected_pose_count
        if supported_expected_pose_count > 0
        else 0.0
    )
    coverage = min(tracking_output_coverage, association_coverage)
    estimate_coverage = 100.0 * len(matches) / len(estimate)
    stats: dict[str, float | int | bool | str] = {
        "alignment": "SE3",
        "scale_fixed": True,
        "raw_ground_truth_poses": len(ground_truth) + duplicate_ground_truth,
        "ground_truth_poses": len(ground_truth),
        "duplicate_ground_truth_timestamps": duplicate_ground_truth,
        "raw_estimated_poses": len(estimate) + duplicate_estimate,
        "estimated_poses": len(estimate),
        "duplicate_estimated_timestamps": duplicate_estimate,
        "expected_poses": expected_pose_count,
        "associated_poses": len(matches),
        "ground_truth_supported_expected_poses": supported_expected_pose_count,
        "tracked_expected_poses": len(tracked_expected_matches),
        "associated_supported_expected_poses": associated_supported_pose_count,
        "tracking_output_coverage_percent": float(tracking_output_coverage),
        "gt_support_fraction_percent": float(ground_truth_support_fraction),
        "association_coverage_percent": float(association_coverage),
        "coverage_percent": float(coverage),
        "estimate_coverage_percent": float(min(100.0, estimate_coverage)),
        "ate_rmse_m": float(np.sqrt(np.mean(errors**2))),
        "ate_mean_m": float(np.mean(errors)),
        "ate_median_m": float(np.median(errors)),
        "ate_std_m": float(np.std(errors)),
        "ate_min_m": float(np.min(errors)),
        "ate_max_m": float(np.max(errors)),
        "maximum_timestamp_difference_s": float(maximum_difference),
    }
    return stats, reference_xyz, aligned_xyz, errors, matched_timestamps


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("ground_truth", type=Path)
    parser.add_argument("estimate", type=Path)
    parser.add_argument("output_dir", type=Path)
    parser.add_argument(
        "--expected-timestamps",
        type=Path,
        help="TUM association file used to compute end-to-end pose coverage",
    )
    parser.add_argument(
        "--max-difference",
        type=float,
        default=0.02,
        help="maximum timestamp association difference in seconds (default: 0.02)",
    )
    args = parser.parse_args()

    stats, reference, aligned, errors, timestamps = evaluate(
        args.ground_truth,
        args.estimate,
        args.max_difference,
        args.expected_timestamps,
    )
    args.output_dir.mkdir(parents=True, exist_ok=True)
    metrics_path = args.output_dir / "metrics.json"
    metrics_path.write_text(
        json.dumps(stats, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    write_aligned_trajectory(
        args.output_dir / "aligned_trajectory.csv",
        reference,
        aligned,
        errors,
        timestamps,
    )
    create_plot(
        args.output_dir / "trajectory_and_ate.png",
        reference,
        aligned,
        errors,
        float(stats["ate_rmse_m"]),
        axes=(0, 1),
    )
    print(json.dumps(stats, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
