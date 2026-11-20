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

"""Strict config keys, capability matrix, and asset preflight."""

import copy
from pathlib import Path

import pytest
import yaml

from autosim.robot.profiles import apply_profile, resolve_path
from autosim.robot.urdf import UrdfModel, missing_meshes
from autosim.runtime import Config
from autosim.runtime.capabilities import support_matrix
from autosim.runtime.presets import compose_config

ROOT = UrdfModel.package_root()
ROBOT_CONFIGS = sorted(
    path
    for path in (ROOT / "config" / "robots").glob("*.yaml")
    if not path.name.startswith("_")
)


def _base_mapping() -> dict:
    """A minimal valid config to mutate in each test."""
    data = Config.read(ROOT / "config" / "robots" / "husky.yaml")
    Config.fill_omitted_sensors(data)
    return data


def test_unknown_sensor_key_names_the_path_and_suggests_the_field():
    data = _base_mapping()
    data["habitat"]["sensors"]["camera"]["hfov"] = 90.0
    with pytest.raises(ValueError) as error:
        Config.validate(data)
    message = str(error.value)
    assert "habitat.sensors.camera.hfov" in message
    assert "hfov_deg" in message


def test_unknown_robot_key_is_rejected():
    data = _base_mapping()
    data["habitat"]["robot"]["wheel_radius_m"] = 0.1
    with pytest.raises(ValueError, match="habitat.robot.wheel_radius_m"):
        Config.validate(data)


def test_unknown_top_level_habitat_key_is_rejected():
    data = _base_mapping()
    data["habitat"]["renderer"] = "rtx"
    with pytest.raises(ValueError, match="habitat.renderer"):
        Config.validate(data)


def test_dynamic_tables_accept_any_name():
    data = _base_mapping()
    data["habitat"]["robot"]["joint_positions"] = {"any_joint_name": 0.1}
    # Unknown joint names are a joint-table problem, not a schema problem.
    with pytest.raises(ValueError) as error:
        Config.validate(data)
    assert "unknown config key" not in str(error.value)


@pytest.mark.parametrize("rate", [0.0, -1.0, "fast", float("inf")])
def test_control_hz_must_be_a_positive_finite_rate(rate):
    data = _base_mapping()
    data["habitat"]["robot"]["control_hz"] = rate
    with pytest.raises(ValueError, match="control_hz"):
        Config.validate(data)


def test_watchdog_must_be_non_negative():
    data = _base_mapping()
    data["habitat"]["robot"]["watchdog_sec"] = -0.5
    with pytest.raises(ValueError, match="watchdog_sec"):
        Config.validate(data)


def test_enabled_lidar_3d_needs_a_frame():
    data = _base_mapping()
    lidar = data["habitat"]["sensors"]["lidar_3d"]
    lidar["enabled"] = True
    lidar.pop("frame", None)
    with pytest.raises(ValueError, match="lidar_3d.frame"):
        Config.validate(data)


def test_cmd_vel_type_is_validated():
    data = _base_mapping()
    data["habitat"]["robot"]["cmd_vel_type"] = "twist_array"
    with pytest.raises(ValueError, match="cmd_vel_type"):
        Config.validate(data)


def test_joint_command_rejects_a_wheeled_base():
    data = _base_mapping()
    data["habitat"]["robot"]["joint_command"] = {
        "enabled": True,
        "channel": "/joint_commands",
    }
    with pytest.raises(ValueError, match="articulated"):
        Config.validate(data)


def test_joint_states_reject_a_wheeled_base():
    data = _base_mapping()
    data["habitat"]["sensors"]["joint_states"] = {
        "enabled": True,
        "channel": "/joint_states",
    }
    with pytest.raises(ValueError, match="articulated"):
        Config.validate(data)


def test_missing_mesh_is_reported_by_name(tmp_path):
    urdf = tmp_path / "robot.urdf"
    urdf.write_text(
        """<robot name=\"r\">
  <link name=\"base_link\"><visual><geometry>
    <mesh filename=\"meshes/missing.stl\"/>
  </geometry></visual></link>
  <link name=\"tip\"/>
  <joint name=\"j\" type=\"fixed\"><parent link=\"base_link\"/>
    <child link=\"tip\"/></joint>
</robot>"""
    )
    assert missing_meshes(urdf) == ["meshes/missing.stl"]
    with pytest.raises(ValueError, match="missing mesh files"):
        Config.validate_urdf(str(urdf))


@pytest.mark.parametrize("path", ROBOT_CONFIGS, ids=lambda p: p.stem)
def test_shipped_robot_meshes_all_exist(path: Path):
    data = Config._load_yaml(path)
    urdf = apply_profile(data["habitat"]["robot"]).get("urdf", "")
    resolved = UrdfModel.resolve(urdf)
    assert resolved is not None and resolved.is_file()
    assert missing_meshes(resolved) == []


def test_sensor_selection_reports_why_a_sensor_is_unavailable():
    with pytest.raises(ValueError) as error:
        compose_config(
            ROOT / "config" / "default.yaml",
            robot="husky",
            sensors="joint_states",
        )
    message = str(error.value)
    assert "joint_states" in message
    assert "articulated" in message


def test_sensors_all_skips_what_the_robot_cannot_provide():
    settings = compose_config(
        ROOT / "config" / "default.yaml",
        robot="turtlebot3_burger",
        sensors="all",
    )
    sensors = settings["habitat"]["sensors"]
    assert sensors["joint_states"]["enabled"] is False
    assert sensors["stereo"]["enabled"] is False
    assert sensors["camera"]["enabled"] is True


def test_sensors_all_keeps_google_calibrated_blocks():
    settings = compose_config(
        ROOT / "config" / "default.yaml",
        robot="google_robot",
        sensors="all",
    )
    sensors = settings["habitat"]["sensors"]
    assert sensors["stereo"]["enabled"] is True
    assert sensors["range"]["enabled"] is True
    assert sensors["joint_states"]["enabled"] is True


def test_capability_matrix_flags_stereo_without_a_urdf():
    matrix = support_matrix({"profile": "husky"}, "habitat", None)
    assert matrix["stereo"].supported is False
    assert "URDF" in matrix["stereo"].reason


def test_capability_matrix_flags_joint_state_on_a_wheeled_base():
    matrix = support_matrix({"profile": "turtlebot3_burger"}, "habitat", None)
    assert matrix["joint_states"].supported is False


def test_preflight_reports_resolved_assets():
    settings = compose_config(
        ROOT / "config" / "default.yaml", scene="empty", robot="unitree_go2"
    )
    lines = Config.preflight(settings.data)
    assert any(line.startswith("ok: urdf=") for line in lines)
    assert any("unitree_go2" in line for line in lines)


def test_preflight_does_not_report_an_empty_camera_stub():
    settings = compose_config(
        ROOT / "config" / "default.yaml", scene="empty", robot="unitree_go2"
    )
    lines = Config.preflight(settings.data)
    assert not any("cameras" in line for line in lines)


def test_preflight_rejects_a_missing_scene(tmp_path):
    data = copy.deepcopy(_base_mapping())
    data["habitat"]["path"] = str(tmp_path / "nowhere.glb")
    with pytest.raises(ValueError, match="habitat.path not found"):
        Config.preflight(data)
