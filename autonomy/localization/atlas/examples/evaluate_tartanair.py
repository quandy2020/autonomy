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


def read_segment_manifest(estimate: Path):
    """Return independently evaluable Atlas coordinate-frame records."""
    manifest = Path(str(estimate) + ".segments.csv")
    if not manifest.is_file():
        return [
            {
                "index": 0,
                "path": estimate,
                "declared_poses": None,
                "map_id": None,
                "map_id_valid": False,
                "coordinate_epoch": None,
                "destructive_epoch": None,
                "source_kind": "conventional",
            }
        ]

    raw_records = []
    optimized_replacements = []
    with manifest.open("r", encoding="utf-8", newline="") as stream:
        reader = csv.DictReader(stream)
        required = {"kind", "index", "path", "poses"}
        if reader.fieldnames is None or not required.issubset(reader.fieldnames):
            raise ValueError(f"{manifest}: invalid segment manifest header")
        has_replace_active = "replace_active" in reader.fieldnames
        has_coordinate_epoch = "coordinate_epoch" in reader.fieldnames
        has_destructive_epoch = "destructive_epoch" in reader.fieldnames
        for row_number, row in enumerate(reader, 2):
            kind = row["kind"]
            if kind not in {"raw", "optimized_active"}:
                continue
            try:
                index = int(row["index"])
                declared_poses = int(row["poses"])
                map_id_valid = row.get("map_id_valid", "false") == "true"
                map_id = int(row.get("map_id", "0")) if map_id_valid else None
                coordinate_epoch = (
                    int(row["coordinate_epoch"])
                    if has_coordinate_epoch and row["coordinate_epoch"]
                    else None
                )
                destructive_epoch = (
                    int(row["destructive_epoch"])
                    if has_destructive_epoch and row["destructive_epoch"]
                    else None
                )
            except (TypeError, ValueError) as error:
                raise ValueError(
                    f"{manifest}:{row_number}: invalid segment metadata"
                ) from error
            path = Path(row["path"])
            if not path.is_absolute():
                path = manifest.parent / path
            record = {
                "index": index,
                "path": path,
                "declared_poses": declared_poses,
                "map_id": map_id,
                "map_id_valid": map_id_valid,
                "coordinate_epoch": coordinate_epoch,
                "destructive_epoch": destructive_epoch,
                "source_kind": kind,
            }
            if kind == "raw":
                raw_records.append(record)
                continue
            replace_active = (
                row.get("replace_active", "false") == "true"
                if has_replace_active
                else row.get("selected", "false") == "true"
            )
            if replace_active:
                optimized_replacements.append(record)

    raw_records.sort(key=lambda record: record["index"])
    indexes = [record["index"] for record in raw_records]
    if not raw_records or len(set(indexes)) != len(indexes):
        raise ValueError(f"{manifest}: raw segment indexes must be unique")
    if len(optimized_replacements) > 1:
        raise ValueError(f"{manifest}: multiple active optimized replacements")
    if not optimized_replacements:
        return raw_records

    optimized = optimized_replacements[0]
    matching_indexes = []
    for index, raw in enumerate(raw_records):
        if raw["map_id_valid"] != optimized["map_id_valid"]:
            continue
        if raw["map_id_valid"] and raw["map_id"] != optimized["map_id"]:
            continue
        if optimized["destructive_epoch"] is not None:
            if raw["destructive_epoch"] != optimized["destructive_epoch"]:
                continue
        matching_indexes.append(index)
    if not matching_indexes:
        raise ValueError(
            f"{manifest}: active optimized trajectory has no matching raw epoch"
        )
    if optimized["destructive_epoch"] is None:
        matching_indexes = [matching_indexes[-1]]

    optimized["index"] = raw_records[matching_indexes[0]]["index"]
    matching = set(matching_indexes)
    records = [
        raw for index, raw in enumerate(raw_records) if index not in matching
    ]
    records.append(optimized)
    records.sort(key=lambda record: record["index"])
    return records


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
    ground_truth_indices = []
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
        ground_truth_indices.append(ground_truth_index)
    return (
        np.asarray(matched_times, dtype=np.int64),
        np.asarray(reference_positions, dtype=float),
        np.asarray(estimate_indices, dtype=np.int64),
        np.asarray(deltas_ns, dtype=np.int64),
        np.asarray(ground_truth_indices, dtype=np.int64),
    )


def write_aligned_trajectory(
    path: Path,
    matched_times: np.ndarray,
    association_deltas_ns: np.ndarray,
    reference: np.ndarray,
    aligned: np.ndarray,
    errors: np.ndarray,
    segment_indices: np.ndarray,
) -> None:
    """Write associated fixed-scale positions and per-frame ATE."""
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(
            [
                "segment_index",
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
        for segment, timestamp, delta_ns, ground_truth, estimate, error in zip(
            segment_indices, matched_times, association_deltas_ns,
            reference, aligned, errors
        ):
            writer.writerow(
                [
                    int(segment),
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


def interval_union_duration_ns(intervals) -> int:
    """Return the union duration of timestamp intervals."""
    if not intervals:
        return 0
    ordered = sorted((int(first), int(last)) for first, last in intervals)
    first, last = ordered[0]
    duration = 0
    for next_first, next_last in ordered[1:]:
        if next_first <= last:
            last = max(last, next_last)
        else:
            duration += max(0, last - first)
            first, last = next_first, next_last
    return duration + max(0, last - first)


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
    segment_records = read_segment_manifest(args.estimate)
    maximum_delta_ns = round(args.max_time_delta_ms * 1.0e6)
    segment_metrics = []
    aligned_times = []
    aligned_deltas = []
    aligned_references = []
    aligned_estimates = []
    aligned_errors = []
    aligned_segment_indices = []
    all_association_deltas = []
    matched_ground_truth_indices = set()
    evaluated_ground_truth_indices = set()
    matched_intervals = []
    raw_estimated_poses = 0
    duplicate_estimated_timestamps = 0
    estimated_poses = 0
    matched_poses = 0
    longest_segment_matches = 0
    total_ground_truth_length = 0.0
    total_estimate_length = 0.0
    plot_payload = None

    for record in segment_records:
        raw_count, duplicate_count, estimate_times, estimate_positions = (
            read_estimate(record["path"])
        )
        if (
            record["declared_poses"] is not None
            and raw_count != record["declared_poses"]
        ):
            raise ValueError(
                f"{record['path']}: manifest declares "
                f"{record['declared_poses']} poses, file contains {raw_count}"
            )
        (
            matched_times,
            reference,
            estimate_indices,
            association_deltas_ns,
            ground_truth_indices,
        ) = associate_nearest(
            camera_times,
            ground_truth_positions,
            estimate_times,
            maximum_delta_ns,
        )
        raw_estimated_poses += raw_count
        duplicate_estimated_timestamps += duplicate_count
        estimated_poses += len(estimate_times)
        matched_poses += len(matched_times)
        all_association_deltas.append(association_deltas_ns)
        matched_ground_truth_indices.update(
            int(index) for index in ground_truth_indices
        )
        longest_segment_matches = max(
            longest_segment_matches, len(set(ground_truth_indices.tolist()))
        )
        if len(ground_truth_indices) > 0:
            matched_intervals.append(
                (
                    int(camera_times[int(ground_truth_indices[0])]),
                    int(camera_times[int(ground_truth_indices[-1])]),
                )
            )

        details = {
            "segment_index": record["index"],
            "source_kind": record["source_kind"],
            "map_id": record["map_id"],
            "map_id_valid": record["map_id_valid"],
            "coordinate_epoch": record["coordinate_epoch"],
            "destructive_epoch": record["destructive_epoch"],
            "path": str(record["path"]),
            "raw_estimated_poses": raw_count,
            "estimated_poses": len(estimate_times),
            "duplicate_estimated_timestamps": duplicate_count,
            "matched_poses": len(matched_times),
            "ground_truth_coverage_percent": float(
                100.0 * len(set(ground_truth_indices.tolist()))
                / len(camera_times)
            ),
            "evaluable": len(matched_times) >= 3,
        }
        if len(matched_times) < 3:
            segment_metrics.append(details)
            continue
        evaluated_ground_truth_indices.update(
            int(index) for index in ground_truth_indices
        )

        matched_estimate = estimate_positions[estimate_indices]
        alignment_rotation, alignment_translation = rigid_alignment(
            matched_estimate, reference
        )
        aligned = (
            alignment_rotation @ matched_estimate.T
        ).T + alignment_translation
        errors = np.linalg.norm(aligned - reference, axis=1)
        ground_truth_length = path_length(reference)
        estimate_length = path_length(matched_estimate)
        total_ground_truth_length += ground_truth_length
        total_estimate_length += estimate_length
        details.update(
            {
                "alignment": "SE3",
                "alignment_rotation": alignment_rotation.tolist(),
                "alignment_translation_m": alignment_translation.tolist(),
                "ground_truth_path_length_m": ground_truth_length,
                "estimate_path_length_m_before_alignment": estimate_length,
                "ate_rmse_m": float(np.sqrt(np.mean(errors**2))),
                "ate_mean_m": float(np.mean(errors)),
                "ate_max_m": float(np.max(errors)),
            }
        )
        segment_metrics.append(details)
        aligned_times.append(matched_times)
        aligned_deltas.append(association_deltas_ns)
        aligned_references.append(reference)
        aligned_estimates.append(aligned)
        aligned_errors.append(errors)
        aligned_segment_indices.append(
            np.full(len(matched_times), record["index"], dtype=np.int64)
        )
        if plot_payload is None or len(matched_times) > len(plot_payload[0]):
            plot_payload = (
                matched_times,
                reference,
                aligned,
                errors,
                record["index"],
            )

    if not aligned_errors:
        raise ValueError(
            "no Atlas-map segment has at least three associated poses"
        )

    matched_times = np.concatenate(aligned_times)
    association_deltas_ns = np.concatenate(aligned_deltas)
    reference = np.concatenate(aligned_references)
    aligned = np.concatenate(aligned_estimates)
    errors = np.concatenate(aligned_errors)
    segment_indices = np.concatenate(aligned_segment_indices)
    order = np.argsort(matched_times, kind="stable")
    matched_times = matched_times[order]
    association_deltas_ns = association_deltas_ns[order]
    reference = reference[order]
    aligned = aligned[order]
    errors = errors[order]
    segment_indices = segment_indices[order]
    nonempty_deltas = [values for values in all_association_deltas if len(values)]
    all_deltas = np.concatenate(nonempty_deltas)
    matched_duration_ns = interval_union_duration_ns(matched_intervals)
    ground_truth_duration_ns = int(camera_times[-1]) - int(camera_times[0])
    length_ratio = (
        total_estimate_length / total_ground_truth_length
        if total_ground_truth_length > 0.0
        else None
    )

    metrics = {
        "alignment": (
            "SE3" if len(segment_records) == 1 else "SE3_PER_ATLAS_MAP"
        ),
        "scale_fixed": True,
        "scale_fitted": False,
        "global_continuity": len(segment_records) == 1,
        "segment_manifest_used": len(segment_records) > 1 or Path(
            str(args.estimate) + ".segments.csv"
        ).is_file(),
        "segment_count": len(segment_records),
        "evaluable_segment_count": len(aligned_errors),
        "segments": segment_metrics,
        "plot_segment_index": int(plot_payload[4]),
        "association": "nearest_timestamp",
        "maximum_time_delta_ms": float(args.max_time_delta_ms),
        "association_mean_delta_ms": float(np.mean(all_deltas)) * 1.0e-6,
        "association_max_delta_ms": float(np.max(all_deltas)) * 1.0e-6,
        "ground_truth_pose_order": "x y z qx qy qz qw",
        "ground_truth_world_frame": "AirSim NED",
        "ground_truth_pose_file": str(args.ground_truth_pose),
        "camera_times_file": str(args.camera_times),
        "estimate_file": str(args.estimate),
        "ground_truth_poses": len(camera_times),
        "raw_estimated_poses": int(raw_estimated_poses),
        "estimated_poses": int(estimated_poses),
        "duplicate_estimated_timestamps": int(duplicate_estimated_timestamps),
        "matched_poses": int(matched_poses),
        "ate_matched_poses": int(len(errors)),
        "unmatched_estimated_poses": int(estimated_poses - matched_poses),
        "estimate_coverage_percent": float(
            100.0 * matched_poses / estimated_poses
        ),
        "ground_truth_coverage_percent": float(
            100.0 * len(matched_ground_truth_indices) / len(camera_times)
        ),
        "ate_ground_truth_coverage_percent": float(
            100.0 * len(evaluated_ground_truth_indices) / len(camera_times)
        ),
        "ground_truth_time_coverage_percent": (
            float(100.0 * matched_duration_ns / ground_truth_duration_ns)
            if ground_truth_duration_ns > 0
            else 100.0
        ),
        "longest_segment_coverage_percent": float(
            100.0 * longest_segment_matches / len(camera_times)
        ),
        "matched_duration_s": float(matched_duration_ns) * 1.0e-9,
        "ground_truth_path_length_m": total_ground_truth_length,
        "estimate_path_length_m_before_alignment": total_estimate_length,
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
        segment_indices,
    )
    _, plot_reference, plot_aligned, plot_errors, _ = plot_payload
    create_plot(
        args.output_dir / "trajectory_and_ate.png",
        plot_reference,
        plot_aligned,
        plot_errors,
        float(np.sqrt(np.mean(plot_errors**2))),
        axes=(0, 1),
    )
    print(json.dumps(metrics, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
