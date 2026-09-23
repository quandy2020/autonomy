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
from pathlib import Path

import pytest

from autosim.cameras import (
    COMMODITY_HFOV_DEG,
    covers_full_circle,
    layout_names,
    projection_height,
    resolve_surround,
    ros_to_habitat_mount,
)
from autosim.config import Config


def test_four_and_six_cover_360_with_commodity_fov():
    for count in (4, 6):
        cameras = resolve_surround(
            {"enabled": True, "count": count, "hfov_deg": COMMODITY_HFOV_DEG}
        )
        assert len(cameras) == count
        assert all(abs(cam.hfov_deg - 120.0) < 1e-9 for cam in cameras)
        assert covers_full_circle(cameras)
        # Even spacing.
        yaws = sorted(cam.yaw % (2.0 * math.pi) for cam in cameras)
        step = 2.0 * math.pi / count
        for index in range(1, count):
            assert abs((yaws[index] - yaws[index - 1]) - step) < 1e-9


def test_six_names_match_fastbev():
    names = [name for name, _ in layout_names(6)]
    assert names == [
        "front",
        "front_left",
        "rear_left",
        "rear",
        "rear_right",
        "front_right",
    ]


def test_explicit_channels_override_prefix():
    cameras = resolve_surround(
        {
            "enabled": True,
            "count": 4,
            "hfov_deg": COMMODITY_HFOV_DEG,
            "cameras": {
                "front": {
                    "rgb_channel": "/bev/front/image",
                    "info_channel": "/bev/front/info",
                },
                "left": {
                    "rgb_channel": "/bev/left/image",
                    "info_channel": "/bev/left/info",
                },
                "rear": {
                    "rgb_channel": "/bev/rear/image",
                    "info_channel": "/bev/rear/info",
                },
                "right": {
                    "rgb_channel": "/bev/right/image",
                    "info_channel": "/bev/right/info",
                },
            },
        }
    )
    by_name = {cam.name: cam for cam in cameras}
    assert by_name["front"].rgb_channel == "/bev/front/image"
    assert by_name["right"].info_channel == "/bev/right/info"


def test_each_camera_keeps_its_own_fov_and_base_link_pose():
    cameras = resolve_surround(
        {
            "enabled": True,
            "cameras": {
                "front": {
                    "hfov_deg": 100.0,
                    "width": 640,
                    "height": 480,
                    "rate_hz": 5.0,
                    "rgb_channel": "/bev/front/image",
                    "info_channel": "/bev/front/info",
                    "xyz": [0.5, 0.1, 1.1],
                    "yaw_deg": 5.0,
                },
                "left": {
                    "hfov_deg": 130.0,
                    "rgb_channel": "/bev/left/image",
                    "info_channel": "/bev/left/info",
                    "xyz": [0.0, 0.5, 1.2],
                    "yaw_deg": 90.0,
                },
                "rear": {
                    "hfov_deg": 130.0,
                    "rgb_channel": "/bev/rear/image",
                    "info_channel": "/bev/rear/info",
                    "xyz": [-0.5, 0.0, 1.2],
                    "yaw_deg": 180.0,
                },
                "right": {
                    "hfov_deg": 130.0,
                    "rgb_channel": "/bev/right/image",
                    "info_channel": "/bev/right/info",
                    "xyz": [0.0, -0.5, 1.2],
                    "yaw_deg": 270.0,
                },
            },
        }
    )
    front = {cam.name: cam for cam in cameras}["front"]
    assert front.hfov_deg == 100.0
    assert front.vfov_deg > 0.0
    assert front.width == 640
    assert front.rate_hz == 5.0
    assert front.x == 0.5 and front.y == 0.1 and front.z == 1.1
    assert abs(front.yaw - math.radians(5.0)) < 1e-9
    assert covers_full_circle(cameras)


def test_camera_type_fisheye_and_unknown():
    block = {
        "enabled": True,
        "count": 4,
        "hfov_deg": COMMODITY_HFOV_DEG,
        "cameras": {
            "front": {
                "type": "fisheye",
                "rgb_channel": "/bev/front/image",
                "info_channel": "/bev/front/info",
            },
            "left": {
                "rgb_channel": "/bev/left/image",
                "info_channel": "/bev/left/info",
            },
            "rear": {
                "rgb_channel": "/bev/rear/image",
                "info_channel": "/bev/rear/info",
            },
            "right": {
                "type": "not-a-camera",
                "rgb_channel": "/bev/right/image",
                "info_channel": "/bev/right/info",
            },
        },
    }
    with pytest.raises(ValueError, match="type"):
        resolve_surround(block)
    block["cameras"]["right"]["type"] = "equirectangular"
    cameras = resolve_surround(block)
    by_name = {cam.name: cam for cam in cameras}
    assert by_name["front"].model == "fisheye"
    assert by_name["left"].model == "pinhole"
    assert by_name["right"].model == "equirectangular"


def test_vertical_fov_is_independent_of_aspect():
    cameras = resolve_surround(
        {
            "enabled": True,
            "count": 4,
            "hfov_deg": 120.0,
            "cameras": {
                name: {
                    "vfov_deg": 70.0,
                    "rgb_channel": f"/bev/{name}/image",
                    "info_channel": f"/bev/{name}/info",
                }
                for name in ("front", "left", "rear", "right")
            },
        }
    )
    assert all(abs(cam.vfov_deg - 70.0) < 1e-9 for cam in cameras)
    assert projection_height(960, 540, 120.0, 70.0) < 540
    assert projection_height(960, 540, 120.0, 88.53) == 540


def test_disabled_camera_is_omitted():
    cameras = {
        "front": {"enabled": True, "rgb_channel": "/a", "info_channel": "/b"},
        "left": {"enabled": False, "rgb_channel": "/c", "info_channel": "/d"},
        "rear": {"enabled": True, "rgb_channel": "/e", "info_channel": "/f"},
        "right": {"enabled": True, "rgb_channel": "/g", "info_channel": "/h"},
    }
    with pytest.raises(ValueError, match="enabled cameras"):
        resolve_surround(cameras)


def test_narrow_fov_rejected():
    with pytest.raises(ValueError, match="360"):
        Config.validate_surround({"enabled": True, "count": 4, "hfov_deg": 60.0})


def test_ros_mount_forward_is_habitat_minus_z():
    ax, ay, az = ros_to_habitat_mount(0.4, 0.0, 1.2)
    assert abs(ax) < 1e-12
    assert abs(ay - 1.2) < 1e-12
    assert abs(az + 0.4) < 1e-12


def test_bev_yaml_loads_six_cameras():
    root = Path(__file__).resolve().parents[1]
    settings = Config.load(root / "config" / "bev.yaml")
    cameras = resolve_surround(settings["habitat"]["sensors"]["cameras"])
    channels = settings.channel_map()
    assert len(cameras) == 6
    front = {cam.name: cam for cam in cameras}["front"]
    assert front.model == "pinhole"
    assert abs(front.hfov_deg - 120.0) < 1e-9
    assert abs(front.vfov_deg - 88.53) < 1e-9
    assert front.frame == "cam_front"
    assert abs(front.x - 0.40) < 1e-6 and abs(front.z - 1.20) < 1e-6
    assert abs(front.yaw) < 1e-9
    left = {cam.name: cam for cam in cameras}["front_left"]
    assert abs(left.x - 0.20) < 1e-6
    assert abs(left.yaw - math.radians(60.0)) < 1e-6
    assert channels["surround_rgb_front"] == "/surround/front/image_raw"
    assert channels["surround_info_rear_left"] == "/surround/rear_left/camera_info"
    assert channels["surround_panorama"] == "/surround/panorama"
    assert covers_full_circle(cameras)
    assert settings["habitat"]["sensors"]["imu"]["enabled"] is False
    assert settings["habitat"]["sensors"]["lidar_3d"]["enabled"] is False
