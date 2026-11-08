"""Regression checks for raw Oxford Spires frame and point-time conversion."""

import importlib.util
import tempfile
import unittest
from pathlib import Path

import numpy as np


SCRIPT_PATH = Path(__file__).with_name("prepare_oxford_spires.py")
SPEC = importlib.util.spec_from_file_location("prepare_oxford_spires", SCRIPT_PATH)
PREPARER = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(PREPARER)


class OxfordSpiresPreparationTest(unittest.TestCase):
    def test_imu_conversion_preserves_native_sensor_values(self):
        with tempfile.TemporaryDirectory() as temporary_directory:
            directory = Path(temporary_directory)
            source = directory / "imu.csv"
            output = directory / "imu.txt"
            source.write_text(
                "secs,nsecs,acc_x,acc_y,acc_z,ang_vel_x,ang_vel_y,ang_vel_z\n"
                "10,250000000,1,2,3,4,5,6\n"
            )

            PREPARER.convert_imu(source, output)

            values = np.loadtxt(output, comments="#")
            np.testing.assert_allclose(values, [10.25, 4, 5, 6, 1, 2, 3])
            self.assertIn("native IMU sensor axes", output.read_text().splitlines()[0])

    def test_scan_conversion_uses_relative_seconds_and_native_coordinates(self):
        with tempfile.TemporaryDirectory() as temporary_directory:
            directory = Path(temporary_directory)
            source = directory / "10.000000000.pcd"
            output = directory / "scan.txt"
            point_type = np.dtype(
                {
                    "names": ["x", "y", "z", "intensity", "timestamp"],
                    "formats": ["<f4", "<f4", "<f4", "<f4", "<f8"],
                }
            )
            points = np.array([(1, 2, 3, 4, 10.025), (5, 6, 7, 8, 10.075)],
                              dtype=point_type)
            with source.open("wb") as stream:
                stream.write(
                    b"FIELDS x y z intensity timestamp\n"
                    b"SIZE 4 4 4 4 8\n"
                    b"TYPE F F F F F\n"
                    b"COUNT 1 1 1 1 1\n"
                    b"WIDTH 2\nHEIGHT 1\nPOINTS 2\nDATA binary\n"
                )
                stream.write(points.tobytes())

            PREPARER.read_scan(source, output)

            staged = np.loadtxt(output)
            np.testing.assert_allclose(
                staged,
                [[1, 2, 3, 4, 0.025], [5, 6, 7, 8, 0.075]],
                atol=1e-8,
            )


if __name__ == "__main__":
    unittest.main()
