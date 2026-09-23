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

import math

import numpy as np

from autosim.cameras import SurroundCamera
from autosim.stitch import stitch_cylindrical


def _camera(name: str, yaw: float, color: tuple[int, int, int]) -> tuple[SurroundCamera, np.ndarray]:
    image = np.zeros((40, 80, 3), dtype=np.uint8)
    image[:] = color
    camera = SurroundCamera(
        name=name,
        model="pinhole",
        yaw=yaw,
        hfov_deg=120.0,
        vfov_deg=90.0,
        width=80,
        height=40,
        x=0.0,
        y=0.0,
        z=1.0,
        rate_hz=10.0,
        frame=f"cam_{name}",
        rgb_channel=f"/{name}",
        info_channel=f"/{name}/info",
        uuid=f"surround_{name}",
    )
    return camera, image


def _column(width: int, yaw: float) -> int:
    """Panorama column whose ray has ROS yaw ``yaw`` (0 = forward)."""
    u = (math.pi - yaw) * width / (2.0 * math.pi) - 0.5
    return int(round(u))


def test_forward_pixel_uses_front_camera():
    front, front_image = _camera("front", 0.0, (200, 0, 0))
    left, left_image = _camera("front_left", math.radians(60.0), (0, 200, 0))
    images = {front.uuid: front_image, left.uuid: left_image}
    pano = stitch_cylindrical(images, (front, left), 360, 80)
    center = pano[40, _column(360, 0.0)]
    assert center[0] > 150
    assert center[1] < 30


def test_overlap_blends_both_cameras():
    front, front_image = _camera("front", 0.0, (200, 0, 0))
    left, left_image = _camera("front_left", math.radians(60.0), (0, 200, 0))
    images = {front.uuid: front_image, left.uuid: left_image}
    pano = stitch_cylindrical(images, (front, left), 360, 80)
    pixel = pano[40, _column(360, math.radians(30.0))]
    assert pixel[0] > 70 and pixel[1] > 70
    assert abs(int(pixel[0]) - int(pixel[1])) < 40


def test_outside_fov_stays_black():
    front, front_image = _camera("front", 0.0, (200, 0, 0))
    pano = stitch_cylindrical({front.uuid: front_image}, (front,), 360, 80)
    rear = pano[40, _column(360, math.pi)]
    assert int(rear.sum()) == 0
