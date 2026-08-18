# Copyright 2026 The Openbot Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Tests for Sensors Gaussian noise."""

import numpy as np

from autosim.sensors import Sensors


class FakeSim:
    def laser_ranges(self, angle_min, angle_max, num_beams, range_max):
        return np.full((num_beams,), 5.0, dtype=np.float32)

    def lidar_points(self, *args):
        return np.zeros((0, 3), dtype=np.float32)

    def color_depth(self):
        color = np.zeros((4, 4, 3), dtype=np.uint8)
        depth = np.full((4, 4), 2.0, dtype=np.float32)
        return color, depth


def test_depth_noise_zero_keeps_depth():
    sensors = Sensors(
        angle_min=-1.0,
        angle_max=1.0,
        num_beams=3,
        range_min=0.1,
        range_max=30.0,
        depth_noise=0.0,
        seed=0,
    )
    _, depth = sensors.sample_camera(FakeSim())
    assert np.allclose(depth, 2.0)


def test_depth_noise_perturbs_and_clips():
    sensors = Sensors(
        angle_min=-1.0,
        angle_max=1.0,
        num_beams=3,
        range_min=0.1,
        range_max=30.0,
        depth_noise=0.5,
        seed=0,
    )
    _, depth = sensors.sample_camera(FakeSim())
    assert depth.shape == (4, 4)
    assert np.all(depth >= 0.0)
    assert not np.allclose(depth, 2.0)


def test_laser_noise_perturbs_ranges():
    sensors = Sensors(
        angle_min=-1.0,
        angle_max=1.0,
        num_beams=8,
        range_min=0.1,
        range_max=30.0,
        noise=0.2,
        seed=2,
    )
    ranges = sensors.sample_laser(FakeSim())
    assert ranges.shape == (8,)
    assert not np.allclose(ranges, 5.0)
