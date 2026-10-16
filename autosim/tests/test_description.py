# Copyright 2026 The Openbot Authors (duyongquan)

"""Tests for /robot_description URDF preparation."""

from __future__ import annotations

from pathlib import Path

from autosim.robot.description import (
    absolutize_mesh_filenames,
    host_visible_path,
    load_robot_description,
)


ROOT = Path(__file__).resolve().parents[1]


def test_absolutize_mesh_filenames():
    xml = '<mesh filename="meshes/bases/burger_base.stl" scale="0.001 0.001 0.001"/>'
    out = absolutize_mesh_filenames(xml, ROOT / "robot" / "turtlebot3_burger")
    assert "meshes/bases/burger_base.stl" not in out or out.count("meshes/") == 1
    assert "/robot/turtlebot3_burger/meshes/bases/burger_base.stl" in out.replace("\\", "/")
    assert 'scale="0.001 0.001 0.001"' in out


def test_load_turtlebot_description_is_absolute_urdf_path():
    payload = load_robot_description("robot/turtlebot3_burger/turtlebot3_burger.urdf")
    assert payload is not None
    path = Path(payload)
    assert path.is_absolute()
    assert path.name == "turtlebot3_burger.urdf"
    # Remapped host path may not exist inside the container; the real file does.
    assert (ROOT / "robot" / "turtlebot3_burger" / "meshes" / "bases" / "burger_base.stl").is_file()


def test_host_visible_path_env_remap(monkeypatch, tmp_path):
    host_root = tmp_path / "host_autonomy"
    host_root.mkdir()
    monkeypatch.setenv("AUTOSIM_HOST_ROOT", str(host_root))
    monkeypatch.setenv("AUTOSIM_CONTAINER_ROOT", "/workspace/autonomy")
    remapped = host_visible_path(Path("/workspace/autonomy/src/autonomy/autosim/robot/x.urdf"))
    assert remapped == host_root / "src" / "autonomy" / "autosim" / "robot" / "x.urdf"


def test_host_visible_path_noop_without_env(monkeypatch, tmp_path):
    monkeypatch.delenv("AUTOSIM_HOST_ROOT", raising=False)
    monkeypatch.delenv("AUTOSIM_CONTAINER_ROOT", raising=False)
    # Even under a /workspace bind, do not remap unless AUTOSIM_HOST_ROOT is set
    # (Autoviz typically shares the SpaceHero container filesystem).
    containerish = Path("/workspace/autonomy/src/autonomy/autosim/robot/x.urdf")
    assert host_visible_path(containerish).as_posix() == containerish.resolve().as_posix()
    outside = tmp_path / "local.urdf"
    outside.write_text("<robot name='t'/>", encoding="utf-8")
    assert host_visible_path(outside) == outside.resolve()


def test_load_empty_description():
    assert load_robot_description("") is None
