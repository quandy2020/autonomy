#!/usr/bin/env python3
"""Evaluate an Atlas trajectory against AutoSim ground truth.

Both inputs use the TUM trajectory layout::

    timestamp tx ty tz qx qy qz qw

Timestamps are associated monotonically by nearest neighbour.  The estimate
is aligned to ground truth with the orientation-preserving Umeyama solution
with scale fixed to one.  This is important for metric RGB-D, stereo and
visual-inertial validation: scale errors remain visible in ATE and path ratio.

The tool is deliberately independent of Autolink.  The companion recorder
dumps the full-run ``/atlas/odometry`` and extrinsic-corrected
``/gt/odometry`` streams to TUM files for evaluation after an AutoSim run.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from dataclasses import dataclass
from decimal import ROUND_HALF_EVEN, Decimal, InvalidOperation
from pathlib import Path
from typing import Sequence

import numpy as np


NANOSECONDS_PER_SECOND = Decimal(1_000_000_000)


@dataclass(frozen=True)
class Trajectory:
    """A timestamp-sorted TUM trajectory and input-order diagnostics."""

    timestamps_ns: np.ndarray
    positions: np.ndarray
    rotations: np.ndarray
    raw_pose_count: int
    duplicate_timestamp_count: int
    timestamp_regression_count: int


@dataclass(frozen=True)
class Association:
    """One-to-one, monotonically associated trajectory indexes."""

    estimate_indices: np.ndarray
    ground_truth_indices: np.ndarray
    deltas_ns: np.ndarray


def parse_seconds_ns(value: str, path: Path, line_number: int) -> int:
    """Parse decimal seconds into integer nanoseconds without float loss."""
    try:
        seconds = Decimal(value.strip())
    except InvalidOperation as error:
        raise ValueError(
            f"{path}:{line_number}: invalid timestamp {value!r}"
        ) from error
    if not seconds.is_finite():
        raise ValueError(f"{path}:{line_number}: timestamp is not finite")
    timestamp_ns = int(
        (seconds * NANOSECONDS_PER_SECOND).to_integral_value(
            rounding=ROUND_HALF_EVEN
        )
    )
    if not -(2**63) <= timestamp_ns < 2**63:
        raise ValueError(f"{path}:{line_number}: timestamp is outside int64 range")
    return timestamp_ns


def quaternion_xyzw_to_matrix(quaternion: Sequence[float]) -> np.ndarray:
    """Return a rotation matrix for one finite non-zero xyzw quaternion."""
    value = np.asarray(quaternion, dtype=float)
    if value.shape != (4,) or not np.all(np.isfinite(value)):
        raise ValueError("quaternion must contain four finite values")
    norm = float(np.linalg.norm(value))
    if norm <= 1.0e-12:
        raise ValueError("quaternion norm is zero")
    x, y, z, w = value / norm
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


def read_tum(path: Path) -> Trajectory:
    """Read TUM poses, keeping the first occurrence of duplicate timestamps."""
    poses: dict[int, tuple[np.ndarray, np.ndarray]] = {}
    raw_pose_count = 0
    duplicate_timestamp_count = 0
    timestamp_regression_count = 0
    previous_timestamp: int | None = None

    with path.open("r", encoding="utf-8") as stream:
        for line_number, raw_line in enumerate(stream, 1):
            line = raw_line.strip()
            if not line or line.startswith("#"):
                continue
            fields = line.replace(",", " ").split()
            if len(fields) != 8:
                raise ValueError(
                    f"{path}:{line_number}: expected 8 TUM fields, "
                    f"got {len(fields)}"
                )
            timestamp_ns = parse_seconds_ns(fields[0], path, line_number)
            try:
                values = np.asarray(
                    [float(field) for field in fields[1:]], dtype=float
                )
            except ValueError as error:
                raise ValueError(
                    f"{path}:{line_number}: pose contains a non-number"
                ) from error
            if not np.all(np.isfinite(values)):
                raise ValueError(f"{path}:{line_number}: pose is not finite")
            try:
                rotation = quaternion_xyzw_to_matrix(values[3:7])
            except ValueError as error:
                raise ValueError(f"{path}:{line_number}: {error}") from error

            raw_pose_count += 1
            if previous_timestamp is not None and timestamp_ns < previous_timestamp:
                timestamp_regression_count += 1
            previous_timestamp = timestamp_ns
            if timestamp_ns in poses:
                duplicate_timestamp_count += 1
                continue
            poses[timestamp_ns] = (values[:3], rotation)

    if not poses:
        raise ValueError(f"{path}: no trajectory poses")
    timestamps_ns = np.asarray(sorted(poses), dtype=np.int64)
    positions = np.asarray(
        [poses[int(timestamp)][0] for timestamp in timestamps_ns], dtype=float
    )
    rotations = np.asarray(
        [poses[int(timestamp)][1] for timestamp in timestamps_ns], dtype=float
    )
    return Trajectory(
        timestamps_ns=timestamps_ns,
        positions=positions,
        rotations=rotations,
        raw_pose_count=raw_pose_count,
        duplicate_timestamp_count=duplicate_timestamp_count,
        timestamp_regression_count=timestamp_regression_count,
    )


def associate_nearest(
    ground_truth_times_ns: np.ndarray,
    estimate_times_ns: np.ndarray,
    maximum_delta_ns: int,
) -> Association:
    """Associate estimates to unique, increasing nearest GT timestamps."""
    if maximum_delta_ns < 0:
        raise ValueError("maximum association delta must be nonnegative")
    estimate_indices: list[int] = []
    ground_truth_indices: list[int] = []
    deltas_ns: list[int] = []
    last_ground_truth_index = -1

    for estimate_index, timestamp in enumerate(estimate_times_ns):
        right = int(np.searchsorted(ground_truth_times_ns, timestamp, side="left"))
        candidates = []
        if right < len(ground_truth_times_ns):
            candidates.append(right)
        if right > 0:
            candidates.append(right - 1)
        # If a previous estimate consumed both neighbours around this
        # timestamp, the first unused GT sample is still the nearest valid
        # monotonic candidate.  Considering it avoids unnecessarily dropping
        # dense estimates near a sparse GT sample.
        first_unused = last_ground_truth_index + 1
        if first_unused < len(ground_truth_times_ns):
            candidates.append(first_unused)
        candidates = [
            index for index in candidates if index > last_ground_truth_index
        ]
        candidates = list(set(candidates))
        if not candidates:
            continue
        ground_truth_index = min(
            candidates,
            key=lambda index: (
                abs(int(ground_truth_times_ns[index]) - int(timestamp)),
                index,
            ),
        )
        delta_ns = abs(
            int(ground_truth_times_ns[ground_truth_index]) - int(timestamp)
        )
        if delta_ns > maximum_delta_ns:
            continue
        last_ground_truth_index = ground_truth_index
        estimate_indices.append(estimate_index)
        ground_truth_indices.append(ground_truth_index)
        deltas_ns.append(delta_ns)

    return Association(
        estimate_indices=np.asarray(estimate_indices, dtype=np.int64),
        ground_truth_indices=np.asarray(ground_truth_indices, dtype=np.int64),
        deltas_ns=np.asarray(deltas_ns, dtype=np.int64),
    )


def fixed_scale_umeyama(
    source: np.ndarray, target: np.ndarray
) -> tuple[np.ndarray, np.ndarray]:
    """Return the rigid transform mapping source points onto target points.

    This is the orientation-preserving Umeyama/Kabsch solution with scale
    fixed to one.  At least three poses and non-zero translation variance are
    required so a missing or collapsed trajectory cannot report zero error.
    """
    source = np.asarray(source, dtype=float)
    target = np.asarray(target, dtype=float)
    if (
        source.ndim != 2
        or source.shape[1:] != (3,)
        or source.shape != target.shape
        or len(source) < 3
        or not np.all(np.isfinite(source))
        or not np.all(np.isfinite(target))
    ):
        raise ValueError("alignment requires matching finite Nx3 arrays (N >= 3)")

    source_center = source.mean(axis=0)
    target_center = target.mean(axis=0)
    source_zero = source - source_center
    target_zero = target - target_center
    source_variance = float(np.mean(np.sum(source_zero * source_zero, axis=1)))
    target_variance = float(np.mean(np.sum(target_zero * target_zero, axis=1)))
    if source_variance <= 1.0e-15 or target_variance <= 1.0e-15:
        raise ValueError("alignment trajectory is translation-degenerate")

    covariance = target_zero.T @ source_zero / len(source)
    left, _, right_transpose = np.linalg.svd(covariance)
    signs = np.ones(3, dtype=float)
    if np.linalg.det(left) * np.linalg.det(right_transpose) < 0.0:
        signs[-1] = -1.0
    rotation = left @ np.diag(signs) @ right_transpose
    if not np.all(np.isfinite(rotation)) or not math.isclose(
        float(np.linalg.det(rotation)), 1.0, rel_tol=0.0, abs_tol=1.0e-8
    ):
        raise ValueError("alignment did not produce a proper rotation")
    translation = target_center - rotation @ source_center
    return rotation, translation


def path_length(positions: np.ndarray) -> float:
    """Return translation path length in metres."""
    if len(positions) < 2:
        return 0.0
    return float(np.linalg.norm(np.diff(positions, axis=0), axis=1).sum())


def rotation_angle_radians(rotation: np.ndarray) -> float:
    """Return the stable principal angle of a rotation matrix."""
    cosine = float((np.trace(rotation) - 1.0) * 0.5)
    return math.acos(max(-1.0, min(1.0, cosine)))


def relative_pose_errors(
    reference_positions: np.ndarray,
    reference_rotations: np.ndarray,
    estimate_positions: np.ndarray,
    estimate_rotations: np.ndarray,
    delta: int,
) -> tuple[np.ndarray, np.ndarray]:
    """Compute SE(3) relative-pose translation and rotation errors."""
    count = len(reference_positions)
    if delta <= 0 or count <= delta:
        raise ValueError("RPE delta must be positive and smaller than matches")
    translation_errors = []
    rotation_errors = []
    for first in range(count - delta):
        second = first + delta
        reference_relative_rotation = (
            reference_rotations[first].T @ reference_rotations[second]
        )
        estimate_relative_rotation = (
            estimate_rotations[first].T @ estimate_rotations[second]
        )
        reference_relative_translation = reference_rotations[first].T @ (
            reference_positions[second] - reference_positions[first]
        )
        estimate_relative_translation = estimate_rotations[first].T @ (
            estimate_positions[second] - estimate_positions[first]
        )
        translation_errors.append(
            float(
                np.linalg.norm(
                    estimate_relative_translation
                    - reference_relative_translation
                )
            )
        )
        rotation_error = (
            reference_relative_rotation.T @ estimate_relative_rotation
        )
        rotation_errors.append(rotation_angle_radians(rotation_error))
    return (
        np.asarray(translation_errors, dtype=float),
        np.asarray(rotation_errors, dtype=float),
    )


def evaluate(
    ground_truth: Trajectory,
    estimate: Trajectory,
    *,
    maximum_delta_ns: int,
    rpe_delta: int = 1,
    continuity_max_gap_ns: int = 500_000_000,
    continuity_jump_m: float = 2.0,
    continuity_jump_ratio: float = 5.0,
) -> dict[str, object]:
    """Evaluate one estimate and return JSON-serializable metrics."""
    if continuity_max_gap_ns <= 0:
        raise ValueError("continuity maximum gap must be positive")
    if not math.isfinite(continuity_jump_m) or continuity_jump_m <= 0.0:
        raise ValueError("continuity jump distance must be finite and positive")
    if (
        not math.isfinite(continuity_jump_ratio)
        or continuity_jump_ratio <= 1.0
    ):
        raise ValueError("continuity jump ratio must be finite and greater than one")

    association = associate_nearest(
        ground_truth.timestamps_ns,
        estimate.timestamps_ns,
        maximum_delta_ns,
    )
    matched_pose_count = len(association.estimate_indices)
    if matched_pose_count < 3:
        raise ValueError(
            "fewer than three trajectory poses were associated "
            f"(matches={matched_pose_count})"
        )
    if rpe_delta <= 0 or rpe_delta >= matched_pose_count:
        raise ValueError(
            "RPE delta must be positive and smaller than the associated pose count"
        )

    reference_positions = ground_truth.positions[
        association.ground_truth_indices
    ]
    reference_rotations = ground_truth.rotations[
        association.ground_truth_indices
    ]
    estimate_positions = estimate.positions[association.estimate_indices]
    estimate_rotations = estimate.rotations[association.estimate_indices]
    matched_times_ns = estimate.timestamps_ns[association.estimate_indices]

    alignment_rotation, alignment_translation = fixed_scale_umeyama(
        estimate_positions, reference_positions
    )
    aligned_positions = (
        alignment_rotation @ estimate_positions.T
    ).T + alignment_translation
    aligned_rotations = np.einsum(
        "ij,njk->nik", alignment_rotation, estimate_rotations
    )

    ate_errors = np.linalg.norm(
        aligned_positions - reference_positions, axis=1
    )
    rpe_translation, rpe_rotation = relative_pose_errors(
        reference_positions,
        reference_rotations,
        aligned_positions,
        aligned_rotations,
        rpe_delta,
    )

    reference_length = path_length(reference_positions)
    estimate_length = path_length(estimate_positions)
    path_ratio = (
        estimate_length / reference_length if reference_length > 1.0e-12 else None
    )

    timestamp_gaps = np.diff(matched_times_ns)
    time_gap_count = int(np.count_nonzero(timestamp_gaps > continuity_max_gap_ns))
    reference_steps = np.linalg.norm(
        np.diff(reference_positions, axis=0), axis=1
    )
    estimate_steps = np.linalg.norm(
        np.diff(estimate_positions, axis=0), axis=1
    )
    position_jumps = (estimate_steps >= continuity_jump_m) & (
        estimate_steps
        > np.maximum(reference_steps * continuity_jump_ratio, continuity_jump_m)
    )
    position_jump_count = int(np.count_nonzero(position_jumps))
    reset_count = estimate.timestamp_regression_count + position_jump_count
    continuity_break_count = reset_count + time_gap_count
    edge_count = max(1, matched_pose_count - 1)
    continuity_percent = 100.0 * max(
        0.0, 1.0 - continuity_break_count / edge_count
    )

    estimate_pose_count = len(estimate.timestamps_ns)
    ground_truth_pose_count = len(ground_truth.timestamps_ns)
    matched_ground_truth_count = len(
        set(int(index) for index in association.ground_truth_indices)
    )
    ground_truth_duration_ns = int(
        ground_truth.timestamps_ns[-1] - ground_truth.timestamps_ns[0]
    )
    matched_ground_truth_times_ns = ground_truth.timestamps_ns[
        association.ground_truth_indices
    ]
    matched_ground_truth_span_ns = int(
        matched_ground_truth_times_ns[-1] - matched_ground_truth_times_ns[0]
    )

    return {
        "alignment": "SE3_UMEYAMA_FIXED_SCALE",
        "scale_fixed": True,
        "scale_fitted": False,
        "alignment_scale": 1.0,
        "alignment_rotation": alignment_rotation.tolist(),
        "alignment_translation_m": alignment_translation.tolist(),
        "association": "nearest_timestamp_monotonic_one_to_one",
        "maximum_time_delta_ms": maximum_delta_ns * 1.0e-6,
        "association_mean_delta_ms": float(np.mean(association.deltas_ns))
        * 1.0e-6,
        "association_max_delta_ms": float(np.max(association.deltas_ns))
        * 1.0e-6,
        "ground_truth_pose_count": ground_truth_pose_count,
        "estimate_raw_pose_count": estimate.raw_pose_count,
        "estimate_pose_count": estimate_pose_count,
        "matched_pose_count": matched_pose_count,
        "unmatched_estimate_pose_count": estimate_pose_count - matched_pose_count,
        "estimate_duplicate_timestamp_count": estimate.duplicate_timestamp_count,
        "estimate_timestamp_regression_count": estimate.timestamp_regression_count,
        "coverage_percent": 100.0 * matched_pose_count / estimate_pose_count,
        "estimate_coverage_percent": 100.0
        * matched_pose_count
        / estimate_pose_count,
        "ground_truth_coverage_percent": 100.0
        * matched_ground_truth_count
        / ground_truth_pose_count,
        "ground_truth_time_span_coverage_percent": (
            min(
                100.0,
                max(
                    0.0,
                    100.0
                    * matched_ground_truth_span_ns
                    / ground_truth_duration_ns,
                ),
            )
            if ground_truth_duration_ns > 0
            else 100.0
        ),
        "ground_truth_path_length_m": reference_length,
        "estimate_path_length_m_before_alignment": estimate_length,
        "path_length_ratio_no_scale_fit": path_ratio,
        "ate_rmse_m": float(np.sqrt(np.mean(ate_errors**2))),
        "ate_mean_m": float(np.mean(ate_errors)),
        "ate_median_m": float(np.median(ate_errors)),
        "ate_std_m": float(np.std(ate_errors)),
        "ate_min_m": float(np.min(ate_errors)),
        "ate_max_m": float(np.max(ate_errors)),
        "rpe_delta_frames": rpe_delta,
        "rpe_pair_count": len(rpe_translation),
        "rpe_translation_rmse_m": float(
            np.sqrt(np.mean(rpe_translation**2))
        ),
        "rpe_translation_mean_m": float(np.mean(rpe_translation)),
        "rpe_translation_max_m": float(np.max(rpe_translation)),
        "rpe_rotation_rmse_deg": math.degrees(
            float(np.sqrt(np.mean(rpe_rotation**2)))
        ),
        "rpe_rotation_mean_deg": math.degrees(float(np.mean(rpe_rotation))),
        "rpe_rotation_max_deg": math.degrees(float(np.max(rpe_rotation))),
        "continuous": continuity_break_count == 0,
        "continuity_percent": continuity_percent,
        "continuity_break_count": continuity_break_count,
        "continuity_segment_count": 1 + continuity_break_count,
        "reset_count": reset_count,
        "position_jump_count": position_jump_count,
        "time_gap_count": time_gap_count,
        "continuity_max_gap_ms": continuity_max_gap_ns * 1.0e-6,
        "continuity_jump_m": continuity_jump_m,
        "continuity_jump_ratio": continuity_jump_ratio,
    }


def build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Evaluate Atlas TUM output against AutoSim TUM ground truth with "
            "fixed-scale SE(3) alignment."
        )
    )
    parser.add_argument("ground_truth", type=Path, help="AutoSim ground-truth TUM file")
    parser.add_argument("estimate", type=Path, help="Atlas estimated TUM file")
    parser.add_argument(
        "--output",
        type=Path,
        help="optional metrics JSON path (metrics are always printed)",
    )
    parser.add_argument(
        "--max-time-delta-ms",
        type=float,
        default=50.0,
        help="maximum nearest-timestamp association delta (default: 50)",
    )
    parser.add_argument(
        "--rpe-delta",
        type=int,
        default=1,
        help="RPE interval in associated frames (default: 1)",
    )
    parser.add_argument(
        "--continuity-max-gap-ms",
        type=float,
        default=500.0,
        help="timestamp gap considered a continuity break (default: 500)",
    )
    parser.add_argument(
        "--continuity-jump-m",
        type=float,
        default=2.0,
        help="minimum estimate step considered a reset candidate (default: 2)",
    )
    parser.add_argument(
        "--continuity-jump-ratio",
        type=float,
        default=5.0,
        help="estimate/GT step ratio considered a reset (default: 5)",
    )
    return parser


def finite_positive(value: float, name: str) -> None:
    if not math.isfinite(value) or value <= 0.0:
        raise ValueError(f"{name} must be finite and positive")


def finite_nonnegative(value: float, name: str) -> None:
    if not math.isfinite(value) or value < 0.0:
        raise ValueError(f"{name} must be finite and nonnegative")


def main(argv: Sequence[str] | None = None) -> int:
    parser = build_argument_parser()
    args = parser.parse_args(argv)
    try:
        finite_nonnegative(args.max_time_delta_ms, "--max-time-delta-ms")
        finite_positive(
            args.continuity_max_gap_ms, "--continuity-max-gap-ms"
        )
        finite_positive(args.continuity_jump_m, "--continuity-jump-m")
        if (
            not math.isfinite(args.continuity_jump_ratio)
            or args.continuity_jump_ratio <= 1.0
        ):
            raise ValueError(
                "--continuity-jump-ratio must be finite and greater than one"
            )
        if args.rpe_delta <= 0:
            raise ValueError("--rpe-delta must be positive")

        ground_truth = read_tum(args.ground_truth)
        estimate = read_tum(args.estimate)
        metrics = evaluate(
            ground_truth,
            estimate,
            maximum_delta_ns=round(args.max_time_delta_ms * 1.0e6),
            rpe_delta=args.rpe_delta,
            continuity_max_gap_ns=round(
                args.continuity_max_gap_ms * 1.0e6
            ),
            continuity_jump_m=args.continuity_jump_m,
            continuity_jump_ratio=args.continuity_jump_ratio,
        )
        encoded = json.dumps(
            metrics, indent=2, sort_keys=True, allow_nan=False
        )
        if args.output is not None:
            args.output.parent.mkdir(parents=True, exist_ok=True)
            args.output.write_text(encoded + "\n", encoding="utf-8")
        print(encoded)
        return 0
    except (OSError, ValueError) as error:
        print(f"evaluate_autosim: error: {error}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
