#!/usr/bin/env python3
"""Regression tests for segmented TartanAir trajectory evaluation."""

import csv
import json
import math
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path


SCRIPT = Path(__file__).with_name("evaluate_tartanair.py")


def write_reference(root: Path, count: int):
    times = root / "cam_time.txt"
    poses = root / "pose_lcam_front.txt"
    times.write_text(
        "".join(f"{index}.0\n" for index in range(count)),
        encoding="utf-8",
    )
    poses.write_text(
        "".join(
            f"{index}.0 0 0 0 0 0 1\n" for index in range(count)
        ),
        encoding="utf-8",
    )
    return times, poses


def write_tum(path: Path, rows):
    path.write_text(
        "".join(
            f"{time:.9f} {x:.9f} {y:.9f} 0 0 0 0 1\n"
            for time, x, y in rows
        ),
        encoding="utf-8",
    )


def write_manifest(estimate: Path, segments, optimized=None):
    with Path(str(estimate) + ".segments.csv").open(
        "w", encoding="utf-8", newline=""
    ) as stream:
        writer = csv.writer(stream)
        writer.writerow(
            [
                "kind",
                "index",
                "path",
                "poses",
                "first_time_s",
                "last_time_s",
                "selected",
                "replace_active",
                "map_id",
                "map_id_valid",
                "coordinate_epoch",
                "destructive_epoch",
                "coordinate_frame_scope",
            ]
        )
        for segment in segments:
            index, path, poses, map_id = segment[:4]
            coordinate_epoch = segment[4] if len(segment) > 4 else index
            destructive_epoch = segment[5] if len(segment) > 5 else index
            writer.writerow(
                [
                    "raw",
                    index,
                    path,
                    poses,
                    0,
                    0,
                    "false",
                    "false",
                    map_id,
                    "true",
                    coordinate_epoch,
                    destructive_epoch,
                    "independent_atlas_map",
                ]
            )
        if optimized is not None:
            path, poses, map_id, coordinate_epoch, destructive_epoch, replace = (
                optimized
            )
            writer.writerow(
                [
                    "optimized_active",
                    -1,
                    path,
                    poses,
                    0,
                    0,
                    "false",
                    "true" if replace else "false",
                    map_id,
                    "true",
                    coordinate_epoch,
                    destructive_epoch,
                    "active_map_only",
                ]
            )


def evaluate(
    poses: Path,
    times: Path,
    estimate: Path,
    output: Path,
    cwd=None,
):
    completed = subprocess.run(
        [sys.executable, str(SCRIPT), str(poses), str(times),
         str(estimate), str(output)],
        check=False,
        capture_output=True,
        cwd=cwd,
        text=True,
    )
    if completed.returncode != 0:
        raise AssertionError(completed.stdout + completed.stderr)
    return json.loads((output / "metrics.json").read_text(encoding="utf-8"))


class SegmentedEvaluationTest(unittest.TestCase):
    def test_replaces_active_raw_epochs_with_complete_optimized_trajectory(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            times, poses = write_reference(root, 6)
            estimate = root / "trajectory.tum"
            first = Path(str(estimate) + ".segment-000.tum")
            active_before_scale = Path(str(estimate) + ".segment-001.tum")
            active_after_scale = Path(str(estimate) + ".segment-002.tum")
            optimized = Path(str(estimate) + ".active.tum")
            first_rows = [(time, time + 10.0, 0.0) for time in range(3)]
            wrong_rows = [(3, 3.0, 0.0), (4, 7.0, 0.0)]
            final_wrong_rows = [(5, 12.0, 0.0)]
            optimized_rows = [
                (time, time + 100.0, 0.0) for time in range(3, 6)
            ]
            write_tum(first, first_rows)
            write_tum(active_before_scale, wrong_rows)
            write_tum(active_after_scale, final_wrong_rows)
            write_tum(optimized, optimized_rows)
            write_tum(estimate, first_rows)
            write_manifest(
                estimate,
                [
                    (0, first, len(first_rows), 4),
                    (1, active_before_scale, len(wrong_rows), 5, 1, 1),
                    (2, active_after_scale, len(final_wrong_rows), 5, 2, 1),
                ],
                optimized=(optimized, len(optimized_rows), 5, 2, 1, True),
            )

            metrics = evaluate(poses, times, estimate, root / "evaluation")
            self.assertEqual(metrics["segment_count"], 2)
            self.assertEqual(metrics["estimated_poses"], 6)
            self.assertEqual(metrics["matched_poses"], 6)
            self.assertLess(metrics["ate_rmse_m"], 1.0e-9)
            self.assertEqual(metrics["segments"][1]["source_kind"],
                             "optimized_active")
            self.assertEqual(metrics["segments"][1]["path"], str(optimized))

    def test_incomplete_optimized_trajectory_does_not_replace_raw_epoch(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            times, poses = write_reference(root, 4)
            estimate = root / "trajectory.tum"
            raw = Path(str(estimate) + ".segment-000.tum")
            optimized = Path(str(estimate) + ".active.tum")
            raw_rows = [(0, 0.0, 0.0), (1, 1.0, 0.0),
                        (2, 4.0, 0.0), (3, 9.0, 0.0)]
            optimized_rows = [(0, 0.0, 0.0), (1, 1.0, 0.0)]
            write_tum(raw, raw_rows)
            write_tum(optimized, optimized_rows)
            write_tum(estimate, raw_rows)
            write_manifest(
                estimate,
                [(0, raw, len(raw_rows), 4)],
                optimized=(optimized, len(optimized_rows), 4, 0, 0, False),
            )

            metrics = evaluate(poses, times, estimate, root / "evaluation")
            self.assertEqual(metrics["segment_count"], 1)
            self.assertEqual(metrics["segments"][0]["source_kind"], "raw")
            self.assertGreater(metrics["ate_rmse_m"], 0.1)

    def test_aligns_maps_independently_and_weights_pose_errors(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            times, poses = write_reference(root, 9)
            estimate = root / "trajectory.tum"
            first = Path(str(estimate) + ".segment-000.tum")
            second = Path(str(estimate) + ".segment-001.tum")
            first_rows = [(time, time + 10.0, 0.0) for time in range(3)]
            second_rows = [
                (time, time - 7.0, float(time % 2)) for time in range(3, 9)
            ]
            write_tum(first, first_rows)
            write_tum(second, second_rows)
            write_tum(estimate, first_rows)
            write_manifest(
                estimate,
                [(0, first, len(first_rows), 4),
                 (1, second, len(second_rows), 5)],
            )

            metrics = evaluate(poses, times, estimate, root / "evaluation")
            self.assertEqual(metrics["segment_count"], 2)
            self.assertFalse(metrics["global_continuity"])
            self.assertAlmostEqual(metrics["ground_truth_coverage_percent"], 100.0)
            self.assertAlmostEqual(
                metrics["longest_segment_coverage_percent"], 200.0 / 3.0
            )
            evaluable = [
                segment for segment in metrics["segments"]
                if segment["evaluable"]
            ]
            squared_error_sum = sum(
                segment["matched_poses"] * segment["ate_rmse_m"] ** 2
                for segment in evaluable
            )
            weighted = math.sqrt(
                squared_error_sum
                / sum(segment["matched_poses"] for segment in evaluable)
            )
            self.assertAlmostEqual(metrics["ate_rmse_m"], weighted, places=12)

    def test_conventional_single_trajectory_remains_supported(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            times, poses = write_reference(root, 3)
            estimate = root / "trajectory.tum"
            write_tum(
                estimate,
                [(time, time + 3.0, 0.0) for time in range(3)],
            )
            metrics = evaluate(poses, times, estimate, root / "evaluation")
            self.assertEqual(metrics["segment_count"], 1)
            self.assertTrue(metrics["global_continuity"])
            self.assertLess(metrics["ate_rmse_m"], 1.0e-9)

    def test_resolves_relative_segment_paths_from_manifest_directory(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            times, poses = write_reference(root, 3)
            estimate = root / "trajectory.tum"
            segment = root / "segments" / "part.tum"
            segment.parent.mkdir()
            rows = [(time, time + 3.0, 0.0) for time in range(3)]
            write_tum(segment, rows)
            write_tum(estimate, rows)
            write_manifest(
                estimate,
                [(0, Path("segments/part.tum"), len(rows), 4)],
            )

            working = root / "working"
            decoy = working / "segments" / "part.tum"
            decoy.parent.mkdir(parents=True)
            write_tum(decoy, rows[:1])
            metrics = evaluate(
                poses,
                times,
                estimate,
                root / "evaluation",
                cwd=working,
            )
            self.assertEqual(metrics["ate_matched_poses"], 3)


if __name__ == "__main__":
    unittest.main()
