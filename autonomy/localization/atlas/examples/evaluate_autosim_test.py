#!/usr/bin/env python3
"""Standard-library tests for the standalone AutoSim trajectory evaluator."""

from __future__ import annotations

import json
import math
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np

import evaluate_autosim as evaluator


SCRIPT = Path(__file__).with_name("evaluate_autosim.py")


def yaw_quaternion(yaw: float) -> tuple[float, float, float, float]:
    return 0.0, 0.0, math.sin(0.5 * yaw), math.cos(0.5 * yaw)


def write_tum(path: Path, rows) -> None:
    lines = []
    for timestamp, position, yaw in rows:
        qx, qy, qz, qw = yaw_quaternion(yaw)
        lines.append(
            f"{timestamp:.9f} "
            f"{position[0]:.12f} {position[1]:.12f} {position[2]:.12f} "
            f"{qx:.12f} {qy:.12f} {qz:.12f} {qw:.12f}\n"
        )
    path.write_text("".join(lines), encoding="utf-8")


def z_rotation(yaw: float) -> np.ndarray:
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    return np.asarray(
        [[cosine, -sine, 0.0], [sine, cosine, 0.0], [0.0, 0.0, 1.0]]
    )


class EvaluateAutosimTest(unittest.TestCase):
    def setUp(self) -> None:
        self.temporary_directory = tempfile.TemporaryDirectory(
            prefix="atlas_evaluate_autosim_"
        )
        self.output_directory = Path(self.temporary_directory.name)

    def tearDown(self) -> None:
        self.temporary_directory.cleanup()

    def rigid_pair(self, scale: float = 1.0) -> tuple[Path, Path]:
        positions = np.asarray(
            [
                [0.0, 0.0, 0.0],
                [0.7, 0.2, 0.1],
                [1.5, 0.8, 0.15],
                [2.0, 1.7, 0.3],
                [2.9, 2.5, 0.5],
            ]
        )
        yaws = np.asarray([0.0, 0.1, 0.25, 0.45, 0.7])
        alignment_yaw = 0.63
        rotation = z_rotation(alignment_yaw)
        translation = np.asarray([4.0, -2.5, 0.8])
        estimate_positions = np.asarray(
            [scale * (rotation.T @ (point - translation)) for point in positions]
        )

        ground_truth_path = self.output_directory / "ground_truth.tum"
        estimate_path = self.output_directory / "estimate.tum"
        write_tum(
            ground_truth_path,
            [(float(index), positions[index], yaws[index]) for index in range(5)],
        )
        write_tum(
            estimate_path,
            [
                (
                    float(index) + 0.005,
                    estimate_positions[index],
                    yaws[index] - alignment_yaw,
                )
                for index in range(5)
            ],
        )
        return ground_truth_path, estimate_path

    def evaluate_paths(
        self,
        ground_truth_path: Path,
        estimate_path: Path,
        **kwargs,
    ) -> dict[str, object]:
        options = {
            "maximum_delta_ns": 10_000_000,
            "continuity_max_gap_ns": 1_500_000_000,
        }
        options.update(kwargs)
        return evaluator.evaluate(
            evaluator.read_tum(ground_truth_path),
            evaluator.read_tum(estimate_path),
            **options,
        )

    def test_fixed_scale_se3_alignment_recovers_rigid_transform(self) -> None:
        ground_truth_path, estimate_path = self.rigid_pair()
        metrics = self.evaluate_paths(ground_truth_path, estimate_path)

        self.assertEqual(metrics["alignment"], "SE3_UMEYAMA_FIXED_SCALE")
        self.assertIs(metrics["scale_fixed"], True)
        self.assertIs(metrics["scale_fitted"], False)
        self.assertEqual(metrics["matched_pose_count"], 5)
        self.assertAlmostEqual(metrics["coverage_percent"], 100.0)
        self.assertLess(metrics["ate_rmse_m"], 1.0e-9)
        self.assertLess(metrics["rpe_translation_rmse_m"], 1.0e-9)
        self.assertLess(metrics["rpe_rotation_rmse_deg"], 1.0e-8)
        self.assertAlmostEqual(metrics["path_length_ratio_no_scale_fit"], 1.0)
        self.assertIs(metrics["continuous"], True)

    def test_fixed_scale_alignment_does_not_hide_scale_error(self) -> None:
        ground_truth_path, estimate_path = self.rigid_pair(scale=2.0)
        metrics = self.evaluate_paths(ground_truth_path, estimate_path)

        self.assertEqual(metrics["alignment_scale"], 1.0)
        self.assertAlmostEqual(metrics["path_length_ratio_no_scale_fit"], 2.0)
        self.assertGreater(metrics["ate_rmse_m"], 0.5)
        self.assertGreater(metrics["rpe_translation_rmse_m"], 0.5)

    def test_timestamp_association_reports_both_coverages(self) -> None:
        ground_truth_path = self.output_directory / "ground_truth.tum"
        estimate_path = self.output_directory / "estimate.tum"
        ground_truth_rows = [
            (0.1 * index, (0.1 * index, 0.02 * index * index, 0.0), 0.0)
            for index in range(6)
        ]
        estimate_rows = [
            (
                0.1 * index + 0.004,
                (0.1 * index + 2.0, 0.02 * index * index - 1.0, 0.0),
                0.0,
            )
            for index in (0, 1, 3, 4, 5)
        ]
        write_tum(ground_truth_path, ground_truth_rows)
        write_tum(estimate_path, estimate_rows)

        metrics = self.evaluate_paths(
            ground_truth_path,
            estimate_path,
            continuity_max_gap_ns=250_000_000,
        )

        self.assertEqual(metrics["matched_pose_count"], 5)
        self.assertAlmostEqual(metrics["estimate_coverage_percent"], 100.0)
        self.assertAlmostEqual(
            metrics["ground_truth_coverage_percent"], 500.0 / 6.0
        )
        self.assertAlmostEqual(metrics["association_max_delta_ms"], 4.0)
        self.assertEqual(metrics["time_gap_count"], 0)

    def test_association_uses_next_monotonic_ground_truth_candidate(self) -> None:
        association = evaluator.associate_nearest(
            np.asarray([0, 100, 110], dtype=np.int64),
            np.asarray([95, 96], dtype=np.int64),
            maximum_delta_ns=20,
        )

        self.assertEqual(association.estimate_indices.tolist(), [0, 1])
        self.assertEqual(association.ground_truth_indices.tolist(), [1, 2])
        self.assertEqual(association.deltas_ns.tolist(), [5, 14])

    def test_ground_truth_time_span_coverage_is_bounded(self) -> None:
        ground_truth_path = self.output_directory / "ground_truth.tum"
        estimate_path = self.output_directory / "estimate.tum"
        positions = [
            (0.0, 0.0, 0.0),
            (0.8, 0.2, 0.0),
            (1.7, 0.7, 0.1),
        ]
        write_tum(
            ground_truth_path,
            [
                (0.1 * index, position, 0.0)
                for index, position in enumerate(positions)
            ],
        )
        write_tum(
            estimate_path,
            [
                (-0.05, positions[0], 0.0),
                (0.10, positions[1], 0.0),
                (0.25, positions[2], 0.0),
            ],
        )

        metrics = self.evaluate_paths(
            ground_truth_path,
            estimate_path,
            maximum_delta_ns=50_000_000,
        )

        self.assertAlmostEqual(
            metrics["ground_truth_time_span_coverage_percent"], 100.0
        )

    def test_spatial_jump_is_reported_as_reset(self) -> None:
        ground_truth_path = self.output_directory / "ground_truth.tum"
        estimate_path = self.output_directory / "estimate.tum"
        ground_truth_rows = []
        estimate_rows = []
        for index in range(7):
            timestamp = 0.1 * index
            ground_truth_position = np.asarray(
                [0.2 * index, 0.03 * index * index, 0.0]
            )
            estimate_position = ground_truth_position.copy()
            if index >= 3:
                estimate_position += np.asarray([10.0, -4.0, 0.0])
            ground_truth_rows.append(
                (timestamp, ground_truth_position, 0.02 * index)
            )
            estimate_rows.append((timestamp, estimate_position, 0.02 * index))
        write_tum(ground_truth_path, ground_truth_rows)
        write_tum(estimate_path, estimate_rows)

        metrics = self.evaluate_paths(
            ground_truth_path,
            estimate_path,
            continuity_max_gap_ns=200_000_000,
            continuity_jump_m=2.0,
            continuity_jump_ratio=5.0,
        )

        self.assertEqual(metrics["position_jump_count"], 1)
        self.assertEqual(metrics["reset_count"], 1)
        self.assertEqual(metrics["continuity_break_count"], 1)
        self.assertEqual(metrics["continuity_segment_count"], 2)
        self.assertIs(metrics["continuous"], False)

    def test_timestamp_regression_is_reported_as_reset(self) -> None:
        ground_truth_path = self.output_directory / "ground_truth.tum"
        estimate_path = self.output_directory / "estimate.tum"
        positions = {
            0.0: (0.0, 0.0, 0.0),
            0.1: (0.1, 0.01, 0.0),
            0.2: (0.2, 0.04, 0.0),
            0.3: (0.3, 0.09, 0.0),
        }
        write_tum(
            ground_truth_path,
            [(stamp, position, 0.0) for stamp, position in positions.items()],
        )
        write_tum(
            estimate_path,
            [
                (stamp, positions[stamp], 0.0)
                for stamp in (0.0, 0.2, 0.1, 0.3)
            ],
        )

        metrics = self.evaluate_paths(
            ground_truth_path,
            estimate_path,
            continuity_max_gap_ns=200_000_000,
        )

        self.assertEqual(metrics["estimate_timestamp_regression_count"], 1)
        self.assertEqual(metrics["reset_count"], 1)
        self.assertIs(metrics["continuous"], False)

    def test_reader_rejects_zero_quaternion(self) -> None:
        path = self.output_directory / "bad.tum"
        path.write_text("0.0 0 0 0 0 0 0 0\n", encoding="utf-8")

        with self.assertRaisesRegex(ValueError, "quaternion norm is zero"):
            evaluator.read_tum(path)

    def test_cli_writes_json_metrics(self) -> None:
        ground_truth_path, estimate_path = self.rigid_pair()
        output = self.output_directory / "results" / "metrics.json"

        completed = subprocess.run(
            [
                sys.executable,
                str(SCRIPT),
                str(ground_truth_path),
                str(estimate_path),
                "--max-time-delta-ms",
                "10",
                "--continuity-max-gap-ms",
                "1500",
                "--output",
                str(output),
            ],
            check=False,
            capture_output=True,
            text=True,
        )

        self.assertEqual(completed.returncode, 0, completed.stderr)
        metrics = json.loads(output.read_text(encoding="utf-8"))
        self.assertEqual(metrics["matched_pose_count"], 5)
        self.assertLess(json.loads(completed.stdout)["ate_rmse_m"], 1.0e-9)

    def test_cli_returns_nonzero_when_estimate_has_no_data(self) -> None:
        ground_truth_path, _ = self.rigid_pair()
        estimate_path = self.output_directory / "empty.tum"
        estimate_path.write_text("# no estimate\n", encoding="utf-8")

        completed = subprocess.run(
            [sys.executable, str(SCRIPT), str(ground_truth_path), str(estimate_path)],
            check=False,
            capture_output=True,
            text=True,
        )

        self.assertNotEqual(completed.returncode, 0)
        self.assertIn("no trajectory poses", completed.stderr)

    def test_cli_returns_nonzero_for_insufficient_associations(self) -> None:
        ground_truth_path, estimate_path = self.rigid_pair()

        completed = subprocess.run(
            [
                sys.executable,
                str(SCRIPT),
                str(ground_truth_path),
                str(estimate_path),
                "--max-time-delta-ms",
                "1",
            ],
            check=False,
            capture_output=True,
            text=True,
        )

        self.assertNotEqual(completed.returncode, 0)
        self.assertIn("fewer than three", completed.stderr)


if __name__ == "__main__":
    unittest.main()
