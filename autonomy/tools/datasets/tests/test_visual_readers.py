# Copyright 2026 The Openbot Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from __future__ import annotations

import tempfile
import unittest
from pathlib import Path

import numpy as np
from autonomy.tools.datasets.readers.euroc import EurocReader, TumViReader
from autonomy.tools.datasets.readers.kitti import KittiReader
from autonomy.tools.datasets.readers.tum_rgbd import TumRgbdReader
from autonomy.tools.datasets.readers.visual import (
    nanoseconds_text_to_ns,
    seconds_text_to_ns,
)
from PIL import Image as PilImage


def _write_gray(path: Path, value: int) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    pixels = np.full((3, 4), value, dtype=np.uint8)
    PilImage.fromarray(pixels, mode="L").save(path)


def _write_rgb(path: Path, value: int) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    pixels = np.full((3, 4, 3), value, dtype=np.uint8)
    PilImage.fromarray(pixels, mode="RGB").save(path)


def _write_depth(path: Path, value: int) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    pixels = np.full((3, 4), value, dtype=np.uint16)
    PilImage.fromarray(pixels, mode="I;16").save(path)


def _make_euroc(root: Path, *, stereo_offset_ns: int = 0) -> None:
    timestamps = [1_403_636_579_763_555_584, 1_403_636_579_813_555_456]
    for camera, offset in (("cam0", 0), ("cam1", stereo_offset_ns)):
        rows = ["#timestamp [ns],filename"]
        for index, timestamp in enumerate(timestamps):
            image_timestamp = timestamp + offset
            filename = f"{image_timestamp}.png"
            rows.append(f"{image_timestamp},{filename}")
            _write_gray(root / "mav0" / camera / "data" / filename, index + 1)
        csv_path = root / "mav0" / camera / "data.csv"
        csv_path.parent.mkdir(parents=True, exist_ok=True)
        csv_path.write_text("\n".join(rows) + "\n", encoding="utf-8")

    imu_path = root / "mav0" / "imu0" / "data.csv"
    imu_path.parent.mkdir(parents=True, exist_ok=True)
    imu_path.write_text(
        "#timestamp [ns],gyro,gyro,gyro,acc,acc,acc\n"
        "1403636579758555392,0.1,0.2,0.3,1.0,2.0,3.0\n"
        "1403636579763555584,0.4,0.5,0.6,4.0,5.0,6.0\n",
        encoding="utf-8",
    )


class TimestampTest(unittest.TestCase):
    def test_decimal_seconds_preserve_nanoseconds(self) -> None:
        self.assertEqual(seconds_text_to_ns("1.000000001"), 1_000_000_001)
        self.assertEqual(seconds_text_to_ns("1.037359e-01"), 103_735_900)
        self.assertEqual(
            nanoseconds_text_to_ns("1403636579763555584"),
            1_403_636_579_763_555_584,
        )


class EurocReaderTest(unittest.TestCase):
    def test_monocular_inertial_preserves_timestamp_and_imu(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            _make_euroc(root)
            reader = EurocReader(root, sensor_mode="monocular_inertial")
            self.assertEqual(len(reader), 2)
            self.assertEqual(reader.frame_timestamps_ns[0], 1_403_636_579_763_555_584)
            self.assertEqual(len(reader.imu), 2)
            self.assertEqual(reader.imu[0].angular_velocity, (0.1, 0.2, 0.3))
            frame = reader.read(0)
            self.assertEqual(frame.primary.encoding, "mono8")
            self.assertEqual(frame.primary.pixels.dtype, np.uint8)
            self.assertIsNone(frame.secondary)

    def test_stereo_association_uses_each_camera_timestamp(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            _make_euroc(root, stereo_offset_ns=5_000_000)
            reader = EurocReader(root, sensor_mode="stereo")
            frame = reader.read(0)
            self.assertEqual(frame.secondary_role, "right")
            self.assertIsNotNone(frame.secondary)
            assert frame.secondary is not None
            self.assertEqual(
                frame.secondary.timestamp_ns - frame.primary.timestamp_ns,
                5_000_000,
            )

    def test_tum_vi_uses_stereo_inertial_mode(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            _make_euroc(root)
            reader = TumViReader(root, load_images=False)
            self.assertEqual(reader.info()["dataset"], "tum-vi")
            self.assertEqual(reader.sensor_mode, "stereo_inertial")
            self.assertEqual(len(reader.imu), 2)


class TumRgbdReaderTest(unittest.TestCase):
    def test_auto_association_preserves_uint16_depth(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            _write_rgb(root / "rgb" / "0.png", 7)
            _write_rgb(root / "rgb" / "1.png", 8)
            _write_depth(root / "depth" / "0.png", 5000)
            _write_depth(root / "depth" / "1.png", 6000)
            (root / "rgb.txt").write_text(
                "# rgb\n1.000000 rgb/0.png\n1.100000 rgb/1.png\n",
                encoding="utf-8",
            )
            (root / "depth.txt").write_text(
                "# depth\n1.005000 depth/0.png\n1.095000 depth/1.png\n",
                encoding="utf-8",
            )
            reader = TumRgbdReader(root)
            self.assertEqual(len(reader), 2)
            frame = reader.read(0)
            self.assertEqual(frame.primary.encoding, "rgb8")
            self.assertEqual(frame.secondary_role, "depth")
            self.assertIsNotNone(frame.secondary)
            assert frame.secondary is not None
            self.assertEqual(frame.secondary.encoding, "mono16")
            self.assertEqual(frame.secondary.pixels.dtype, np.uint16)
            self.assertEqual(int(frame.secondary.pixels[0, 0]), 5000)


class KittiReaderTest(unittest.TestCase):
    def test_stereo_frames_use_decimal_times(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / "image_0").mkdir()
            (root / "image_1").mkdir()
            (root / "times.txt").write_text(
                "0.000000e+00\n1.037359e-01\n", encoding="utf-8"
            )
            (root / "poses.txt").write_text(
                "1 0 0 0 0 1 0 0 0 0 1 0\n"
                "0 -1 0 1 1 0 0 2 0 0 1 3\n",
                encoding="utf-8",
            )
            for index in range(2):
                filename = f"{index:06d}.png"
                _write_gray(root / "image_0" / filename, index + 1)
                _write_gray(root / "image_1" / filename, index + 2)
            reader = KittiReader(root)
            self.assertEqual(reader.frame_timestamps_ns, [0, 103_735_900])
            frame = reader.read(1)
            self.assertEqual(frame.secondary_role, "right")
            self.assertEqual(frame.primary.encoding, "mono8")
            self.assertEqual(frame.secondary.encoding, "mono8")
            self.assertIsNotNone(frame.ground_truth)
            assert frame.ground_truth is not None
            self.assertEqual(frame.ground_truth.timestamp_ns, 103_735_900)
            self.assertEqual(frame.ground_truth.position, (1.0, 2.0, 3.0))
            self.assertAlmostEqual(
                frame.ground_truth.orientation_xyzw[2], 2.0**-0.5
            )
            self.assertAlmostEqual(
                frame.ground_truth.orientation_xyzw[3], 2.0**-0.5
            )
            self.assertEqual(reader.info()["ground_truth_poses"], 2)


if __name__ == "__main__":
    unittest.main()
