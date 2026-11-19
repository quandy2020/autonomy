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

"""Stereo head camera parsing and configuration validation."""

from pathlib import Path

import pytest

from autosim.bridge.stereo import stereo_cameras
from autosim.runtime import Config

ROOT = Path(__file__).resolve().parents[1]


def google_config():
    return Config.load(ROOT / "config/robots/google_robot.yaml").data


def test_disabled_block_yields_no_cameras():
    assert stereo_cameras({}) == []
    assert stereo_cameras({"stereo": {"enabled": False}}) == []
    assert stereo_cameras({"stereo": None}) == []


def test_defaults_fill_frames_and_channels():
    cameras = stereo_cameras({"stereo": {"enabled": True}})
    assert [camera["side"] for camera in cameras] == ["left", "right"]
    assert cameras[0]["frame"] == "camera_left_optical_frame"
    assert cameras[1]["frame"] == "camera_right_optical_frame"
    assert cameras[0]["rgb_channel"] == "/camera/left/image_raw"
    assert cameras[1]["info_channel"] == "/camera/right/camera_info"
    assert cameras[0]["baseline_m"] == pytest.approx(0.10)
    assert cameras[0]["uuid"] == "stereo_left"


@pytest.mark.parametrize(
    "block",
    [
        {"enabled": True, "width": 0},
        {"enabled": True, "height": -1},
        {"enabled": True, "hfov_deg": 180.0},
        {"enabled": True, "hfov_deg": 0.0},
        {"enabled": True, "rate_hz": 0.0},
        {"enabled": True, "baseline_m": 0.0},
        {"enabled": True, "baseline_m": -0.1},
    ],
)
def test_rejects_invalid_geometry(block):
    with pytest.raises(ValueError):
        stereo_cameras({"stereo": block})


def test_google_robot_config_matches_the_head_rig():
    data = google_config()
    cameras = stereo_cameras(data["habitat"]["sensors"])
    assert [camera["frame"] for camera in cameras] == [
        "camera_left_optical_frame",
        "camera_right_optical_frame",
    ]
    assert cameras[0]["baseline_m"] == pytest.approx(0.124)
    assert cameras[0]["width"] == 640 and cameras[0]["height"] == 480
    assert cameras[0]["hfov_deg"] == pytest.approx(60.0)
    camera = data["habitat"]["sensors"]["camera"]
    assert camera["enabled"] is True
    assert camera["follow_urdf"] is True
    assert camera["frame"] == "camera_center_optical_frame"


def test_stereo_requires_the_habitat_backend():
    data = google_config()
    data["habitat"]["backend"] = "fake"
    data["habitat"]["robot"]["joint_command"]["enabled"] = False
    with pytest.raises(ValueError, match="Habitat"):
        Config.load(data)


def test_stereo_rejects_unknown_frames():
    data = google_config()
    data["habitat"]["sensors"]["stereo"]["left_frame"] = "camera_missing_optical_frame"
    with pytest.raises(ValueError):
        Config.load(data)


def test_stereo_channels_must_be_unique():
    data = google_config()
    data["habitat"]["sensors"]["stereo"]["left_channel"] = "/camera/right/image_raw"
    with pytest.raises(ValueError):
        Config.load(data)
