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

"""ToF / cliff range sensor parsing, validation and message encoding."""

import math
from pathlib import Path

import pytest

from autosim.bridge.messages import Messages
from autosim.bridge.range import range_sensors
from autosim.runtime import Config

ROOT = Path(__file__).resolve().parents[1]
GOOGLE_YAML = ROOT / "config/robots/google_robot.yaml"


def google_config():
    return Config.load(GOOGLE_YAML).data


def test_disabled_block_yields_no_beams():
    assert range_sensors({}) == []
    assert range_sensors({"range": {"enabled": False}}) == []
    assert range_sensors({"range": None}) == []


def test_defaults_fill_channel_and_limits():
    beams = range_sensors({"range": {"enabled": True, "sensors": {"tof.front": None}}})
    assert len(beams) == 1
    beam = beams[0]
    assert beam["frame"] == "tof.front"
    assert beam["channel"] == "/range/tof.front"
    assert beam["min_range"] == pytest.approx(0.02)
    assert beam["max_range"] == pytest.approx(4.0)
    assert beam["fov"] == pytest.approx(0.0349)
    assert beam["rate_hz"] == pytest.approx(10.0)
    assert beam["radiation_type"] == 1


def test_list_form_and_string_channels():
    beams = range_sensors(
        {
            "range": {
                "enabled": True,
                "sensors": [
                    {"frame": "tof.left", "channel": "/tof/left"},
                    {"frame": "tof.right", "channel": "/tof/right", "max_range": 1.5},
                ],
            }
        }
    )
    assert [beam["channel"] for beam in beams] == ["/tof/left", "/tof/right"]
    assert beams[1]["max_range"] == pytest.approx(1.5)
    assert beams[0]["max_range"] == pytest.approx(4.0)


@pytest.mark.parametrize(
    "block",
    [
        {"enabled": True, "min_range": -0.1},
        {"enabled": True, "min_range": 1.0, "max_range": 1.0},
        {"enabled": True, "max_range": 0.0},
        {"enabled": True, "fov": 0.0},
        {"enabled": True, "fov": math.pi},
        {"enabled": True, "rate_hz": 0.0},
        {"enabled": True, "radiation_type": 7},
        {"enabled": True, "sensors": {"": "/range/blank"}},
        {"enabled": True, "sensors": [{"frame": "tof.a"}, {"frame": "tof.a"}]},
        {"enabled": True, "sensors": [{"channel": "/range/a"}]},
        {"enabled": True, "sensors": 3},
    ],
)
def test_rejects_invalid_blocks(block):
    with pytest.raises(ValueError):
        range_sensors({"range": block})


def test_google_robot_config_matches_the_urdf_suite():
    data = google_config()
    beams = range_sensors(data["habitat"]["sensors"])
    frames = [beam["frame"] for beam in beams]
    assert len(beams) == 10
    assert frames.count("cliff.rear_left") == 1
    assert frames.count("cliff.rear_right") == 1
    assert sum(frame.startswith("time_of_flight.") for frame in frames) == 8
    cliff = [beam for beam in beams if beam["frame"].startswith("cliff.")]
    assert all(beam["max_range"] == pytest.approx(0.5) for beam in cliff)
    assert all(beam["min_range"] == pytest.approx(0.01) for beam in cliff)
    channels = [beam["channel"] for beam in beams]
    assert len(set(channels)) == len(channels)
    assert "/tof/right_1" in channels and "/cliff/rear_left" in channels


def test_range_requires_the_habitat_backend():
    data = google_config()
    data["habitat"]["backend"] = "fake"
    data["habitat"]["robot"]["joint_command"]["enabled"] = False
    with pytest.raises(ValueError, match="Habitat"):
        Config.load(data)


def test_range_rejects_unknown_frames():
    data = google_config()
    data["habitat"]["sensors"]["range"]["sensors"]["tof.missing"] = "/tof/missing"
    with pytest.raises(ValueError):
        Config.load(data)


def test_range_channels_must_be_unique():
    data = google_config()
    data["habitat"]["sensors"]["range"]["sensors"]["cliff.rear_left"] = "/tof/right_1"
    with pytest.raises(ValueError):
        Config.load(data)


def test_encode_range_reports_inf_outside_the_limits():
    message = Messages.encode_range((1, 2), "tof.front", 1.25, 0.02, 4.0, 0.0349, 1)
    assert message.header.frame_id == "tof.front"
    assert message.range == pytest.approx(1.25)
    assert message.radiation_type == 1
    assert message.field_of_view == pytest.approx(0.0349)
    assert message.min_range == pytest.approx(0.02)
    assert message.max_range == pytest.approx(4.0)
    assert math.isinf(Messages.encode_range((0, 0), "tof.front", float("inf"), 0.02, 4.0).range)
    assert math.isinf(Messages.encode_range((0, 0), "tof.front", 0.001, 0.02, 4.0).range)
    assert math.isinf(Messages.encode_range((0, 0), "tof.front", 9.0, 0.02, 4.0).range)


def test_cone_widens_the_spot_and_sets_the_reported_fov():
    beams = range_sensors(
        {
            "range": {
                "enabled": True,
                "cone_deg": 10.0,
                "samples": 9,
                "sensors": {"tof.front": None},
            }
        }
    )
    assert beams[0]["cone_deg"] == pytest.approx(10.0)
    assert beams[0]["samples"] == 9
    assert beams[0]["fov"] == pytest.approx(math.radians(10.0))


def test_explicit_fov_wins_over_the_cone_width():
    beams = range_sensors(
        {
            "range": {
                "enabled": True,
                "cone_deg": 10.0,
                "fov": 0.5,
                "sensors": {"tof.front": None},
            }
        }
    )
    assert beams[0]["fov"] == pytest.approx(0.5)


def test_zero_cone_collapses_to_a_single_ray():
    beams = range_sensors(
        {
            "range": {
                "enabled": True,
                "cone_deg": 0.0,
                "samples": 16,
                "sensors": {"tof.front": None},
            }
        }
    )
    assert beams[0]["samples"] == 1


@pytest.mark.parametrize(
    "block",
    [
        {"enabled": True, "cone_deg": -1.0},
        {"enabled": True, "cone_deg": 150.0},
        {"enabled": True, "cone_deg": 10.0, "samples": 0},
        {"enabled": True, "cone_deg": 10.0, "samples": 100},
    ],
)
def test_cone_limits_are_validated(block):
    block["sensors"] = {"tof.front": None}
    with pytest.raises(ValueError):
        range_sensors({"range": block})
