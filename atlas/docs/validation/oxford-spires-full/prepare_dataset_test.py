#!/usr/bin/env python3
"""Regression tests for staging native Oxford sensor measurements."""

import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np


SCRIPT_DIRECTORY = Path(__file__).resolve().parent


class PrepareDatasetTest(unittest.TestCase):
    def test_preserves_native_lidar_and_imu_frames(self):
        with tempfile.TemporaryDirectory() as temporary_directory:
            temporary_path = Path(temporary_directory)
            input_directory = temporary_path / "input"
            output_directory = temporary_path / "output"
            (input_directory / "clouds").mkdir(parents=True)
            source_cloud = np.array(
                [[1.25, -2.5, 0.75, 42.0, 0.03], [-3.0, 4.5, 1.0, 8.0, 0.08]]
            )
            np.savetxt(input_directory / "clouds" / "scan.txt", source_cloud)
            (input_directory / "scans.txt").write_text(
                "1700000000.0 clouds/scan.txt\n"
            )
            source_imu = np.array(
                [[1700000000.0, 0.1, -0.2, 0.3, 0.4, -0.5, 9.7],
                 [1700000000.01, 0.2, -0.1, 0.25, 0.35, -0.45, 9.6]]
            )
            np.savetxt(
                input_directory / "imu.txt",
                source_imu,
                header="timestamp wx wy wz ax ay az; native IMU sensor axes",
                comments="# ",
            )

            subprocess.run(
                [
                    sys.executable,
                    str(SCRIPT_DIRECTORY / "prepare_dataset.py"),
                    "--input-directory",
                    str(input_directory),
                    "--output-directory",
                    str(output_directory),
                ],
                check=True,
                capture_output=True,
                text=True,
            )

            staged_cloud = np.loadtxt(output_directory / "clouds" / "000000.txt")
            staged_imu = np.loadtxt(output_directory / "imu.txt", comments="#")
            np.testing.assert_allclose(staged_cloud, source_cloud, atol=1e-10)
            np.testing.assert_allclose(staged_imu, source_imu, atol=1e-10)

    def test_rejects_base_frame_imu_input(self):
        with tempfile.TemporaryDirectory() as temporary_directory:
            temporary_path = Path(temporary_directory)
            input_directory = temporary_path / "input"
            output_directory = temporary_path / "output"
            input_directory.mkdir()
            (input_directory / "scans.txt").write_text("")
            (input_directory / "imu.txt").write_text(
                "# timestamp wx wy wz ax ay az; transformed into base frame\n"
            )
            result = subprocess.run(
                [
                    sys.executable,
                    str(SCRIPT_DIRECTORY / "prepare_dataset.py"),
                    "--input-directory",
                    str(input_directory),
                    "--output-directory",
                    str(output_directory),
                ],
                capture_output=True,
                text=True,
            )
            self.assertNotEqual(result.returncode, 0)
            self.assertIn("native IMU sensor axes", result.stderr)


if __name__ == "__main__":
    unittest.main()
