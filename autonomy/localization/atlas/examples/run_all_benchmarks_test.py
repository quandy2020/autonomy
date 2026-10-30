#!/usr/bin/env python3
"""Unit tests for the Atlas multi-dataset benchmark runner."""

import json
import struct
import tempfile
import unittest
from pathlib import Path

from run_all_benchmarks import (
    TUM_RGBD_CALIBRATIONS,
    Thresholds,
    assess_metrics,
    build_argument_parser,
    discover_tum_rgbd_sequences,
    execute_job,
    inventory_tum_rgbd,
    make_jobs,
    read_kitti_calibration,
    write_kitti_config,
    write_tum_rgbd_associations,
    write_tum_rgbd_config,
)


class KittiConfigurationTest(unittest.TestCase):
    def test_reads_baseline_with_nonzero_primary_projection_offset(self):
        with tempfile.TemporaryDirectory() as temporary:
            calibration = Path(temporary) / "calib.txt"
            calibration.write_text(
                "P0: 700 0 600 -70 0 710 180 0 0 0 1 0\n"
                "P1: 705 0 605 -423 0 715 182 0 0 0 1 0\n",
                encoding="utf-8",
            )
            fx, fy, cx, cy, baseline = read_kitti_calibration(calibration)
            self.assertEqual((fx, fy, cx, cy), (700.0, 710.0, 600.0, 180.0))
            self.assertAlmostEqual(baseline, 0.5)

    def test_generates_sequence_specific_config(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            sequence = root / "sequence"
            image_dir = sequence / "image_0"
            image_dir.mkdir(parents=True)
            (sequence / "calib.txt").write_text(
                "P0: 700 0 600 0 0 710 180 0 0 0 1 0\n"
                "P1: 700 0 600 -350 0 710 180 0 0 0 1 0\n",
                encoding="utf-8",
            )
            (sequence / "times.txt").write_text(
                "0.0\n0.19\n0.38\n", encoding="utf-8"
            )
            png_header = (
                b"\x89PNG\r\n\x1a\n"
                + struct.pack(">I", 13)
                + b"IHDR"
                + struct.pack(">II", 1280, 720)
            )
            (image_dir / "000000.png").write_bytes(png_header)
            base = root / "base.yaml"
            base.write_text(
                "%YAML:1.0\n\n"
                "Camera1.fx: 1\n"
                "Camera1.fy: 1\n"
                "Camera1.cx: 1\n"
                "Camera1.cy: 1\n"
                "Camera.width: 1\n"
                "Camera.height: 1\n"
                "Camera.fps: 1\n"
                "Stereo.b: 1\n",
                encoding="utf-8",
            )
            output = root / "generated.yaml"
            write_kitti_config(base, sequence, output)
            text = output.read_text(encoding="utf-8")
            self.assertTrue(text.startswith("%YAML:1.0\n"))
            self.assertIn("Camera1.fx: 700", text)
            self.assertIn("Camera1.fy: 710", text)
            self.assertIn("Camera.width: 1280", text)
            self.assertIn("Camera.height: 720", text)
            self.assertIn("Camera.fps: 5", text)
            self.assertIn("Stereo.b: 0.5", text)


class MetricsTest(unittest.TestCase):
    def test_flags_ate_coverage_and_kitti_drift(self):
        violations, ate, coverage, translation, rotation = assess_metrics(
            {
                "ate_rmse_m": 2.0,
                "associated_poses": 10,
                "kitti_translation_drift_percent": 6.0,
                "kitti_rotation_drift_deg_per_100m": 3.0,
            },
            Thresholds(1.0, 100.0, 5.0, 2.0),
        )
        self.assertEqual((ate, coverage, translation, rotation), (2.0, 100.0, 6.0, 3.0))
        self.assertEqual(len(violations), 3)

    def test_prefers_tartanair_ground_truth_coverage(self):
        violations, _, coverage, _, _ = assess_metrics(
            {
                "ate_rmse_m": 0.02,
                "estimate_coverage_percent": 100.0,
                "ground_truth_coverage_percent": 61.0,
            },
            Thresholds(0.25, 90.0),
        )
        self.assertEqual(coverage, 61.0)
        self.assertEqual(len(violations), 1)

    def test_prefers_ate_evaluable_coverage_for_segmented_trajectory(self):
        violations, _, coverage, _, _ = assess_metrics(
            {
                "ate_rmse_m": 0.02,
                "ground_truth_coverage_percent": 100.0,
                "ate_ground_truth_coverage_percent": 61.0,
            },
            Thresholds(0.25, 90.0),
        )
        self.assertEqual(coverage, 61.0)
        self.assertEqual(len(violations), 1)


class TumInventoryTest(unittest.TestCase):
    def test_inventory_does_not_extract_archives(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            tum = root / "TUM"
            tum.mkdir()
            archive = tum / "rgbd_dataset_freiburg1_desk.tgz"
            archive.write_bytes(b"not extracted")
            output = root / "results"
            records = inventory_tum_rgbd(root, output)
            self.assertEqual(len(records), 1)
            self.assertFalse(records[0]["extracted_directory_present"])
            saved = json.loads(
                (output / "tum_rgbd_archive_inventory.json").read_text(
                    encoding="utf-8"
                )
            )
            self.assertEqual(saved, records)
            self.assertEqual(archive.read_bytes(), b"not extracted")

    def test_discovers_archive_and_extracted_sequence_union(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            tum = root / "TUM"
            tum.mkdir()
            (tum / "rgbd_dataset_freiburg1_desk.tgz").touch()
            (tum / "rgbd_dataset_freiburg3_room").mkdir()
            self.assertEqual(
                discover_tum_rgbd_sequences(root),
                (
                    "rgbd_dataset_freiburg1_desk",
                    "rgbd_dataset_freiburg3_room",
                ),
            )


class TumRgbdConfigurationTest(unittest.TestCase):
    @staticmethod
    def write_base_config(path):
        keys = (
            "Camera1.fx",
            "Camera1.fy",
            "Camera1.cx",
            "Camera1.cy",
            "Camera1.k1",
            "Camera1.k2",
            "Camera1.p1",
            "Camera1.p2",
            "Camera1.k3",
            "Stereo.b",
            "RGBD.DepthMapFactor",
        )
        path.write_text(
            "%YAML:1.0\n\n" + "".join(f"{key}: 1\n" for key in keys),
            encoding="utf-8",
        )

    def test_generates_official_freiburg_calibrations(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            base = root / "base.yaml"
            self.write_base_config(base)
            for family, expected in TUM_RGBD_CALIBRATIONS.items():
                output = root / f"{family}.yaml"
                write_tum_rgbd_config(
                    base, f"rgbd_dataset_{family}_test", output
                )
                text = output.read_text(encoding="utf-8")
                self.assertTrue(text.startswith("%YAML:1.0\n"))
                for key, value in expected.items():
                    self.assertIn(f"{key}: {value:.12g}\n", text)

    def test_writes_deterministic_one_to_one_associations(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            dataset = root / "rgbd_dataset_freiburg1_test"
            dataset.mkdir()
            (dataset / "rgb.txt").write_text(
                "# rgb\n1.000 rgb/1.png\n1.030 rgb/2.png\n",
                encoding="utf-8",
            )
            (dataset / "depth.txt").write_text(
                "1.009 depth/1.png\n1.011 depth/2.png\n1.031 depth/3.png\n",
                encoding="utf-8",
            )
            output = root / "association.txt"
            self.assertEqual(write_tum_rgbd_associations(dataset, output), 2)
            self.assertEqual(
                output.read_text(encoding="utf-8"),
                "1.000 rgb/1.png 1.009 depth/1.png\n"
                "1.030 rgb/2.png 1.031 depth/3.png\n",
            )


class TumRgbdResumeTest(unittest.TestCase):
    def test_unextracted_is_missing_data_and_resume_recovers(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            workspace = root / "workspace"
            atlas = workspace / "src/autonomy/autonomy/localization/atlas"
            config = atlas / "config"
            examples = atlas / "examples"
            binaries = workspace / "build/autonomy/bin"
            config.mkdir(parents=True)
            examples.mkdir(parents=True)
            binaries.mkdir(parents=True)
            TumRgbdConfigurationTest.write_base_config(config / "tum_rgbd.yaml")
            (config / "orb_vocab.fbow").touch()
            (examples / "evaluate_tum_rgbd.py").touch()
            (binaries / "atlas_rgbd_tum").touch()

            data_root = root / "data"
            tum = data_root / "TUM"
            tum.mkdir(parents=True)
            sequence = "rgbd_dataset_freiburg1_test"
            (tum / f"{sequence}.tgz").touch()
            output = root / "results"
            args = build_argument_parser().parse_args(
                [
                    "--workspace",
                    str(workspace),
                    "--data-root",
                    str(data_root),
                    "--output-dir",
                    str(output),
                    "--suites",
                    "tum_rgbd",
                    "--resume",
                ]
            )
            job = make_jobs(args)[0]
            missing = execute_job(job, resume=True, environment={})
            self.assertEqual(missing.status, "missing_data")

            dataset = tum / sequence
            (dataset / "rgb").mkdir(parents=True)
            (dataset / "depth").mkdir()
            (dataset / "rgb.txt").write_text(
                "1.000 rgb/1.png\n", encoding="utf-8"
            )
            (dataset / "depth.txt").write_text(
                "1.001 depth/1.png\n", encoding="utf-8"
            )
            (dataset / "groundtruth.txt").write_text(
                "1.000 0 0 0 0 0 0 1\n", encoding="utf-8"
            )
            job = make_jobs(args)[0]
            job.evaluation_dir.mkdir(parents=True)
            (job.evaluation_dir / "metrics.json").write_text(
                json.dumps({"ate_rmse_m": 0.01, "coverage_percent": 100.0}),
                encoding="utf-8",
            )
            resumed = execute_job(job, resume=True, environment={})
            self.assertEqual(resumed.status, "passed")
            self.assertTrue(resumed.resumed)


if __name__ == "__main__":
    unittest.main()
