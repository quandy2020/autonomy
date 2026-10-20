# Copyright 2026 The Openbot Authors (duyongquan)

"""Open Beni 7-DoF ONNX policy unit tests."""

from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest

from autosim.robot.policy.beni import (
    ACTION_JOINTS,
    DEFAULT_Q,
    BeniBalancePolicy,
    axis_angle_rpy,
)
from autosim.robot.policy.driver import PolicyDriver, policy_config_from_habitat
from autosim.robot.urdf import UrdfModel

ROOT = Path(__file__).resolve().parents[1]
BENI_ONNX = ROOT / "robot" / "beni" / "models" / "policy.onnx"


def test_beni_urdf_joint_axes():
    model = UrdfModel.load("robot/beni/beni.urdf")
    assert model is not None
    assert model.joint_name["L1_Link"] == "L1"
    assert abs(model.joint_axis["L1_Link"][1] + 1.0) < 1e-9


def test_axis_angle_rpy_identity():
    rpy = axis_angle_rpy((0.0, 0.0, 0.0), (0.0, -1.0, 0.0), 0.0)
    assert all(abs(v) < 1e-12 for v in rpy)


def test_policy_config_beni_defaults():
    cfg = policy_config_from_habitat(
        {"policy": {"enabled": True, "action_space": "beni"}}
    )
    assert cfg["action_space"] == "beni"
    assert cfg["model_path"].endswith("robot/beni/models/policy.onnx")
    assert cfg["use_laser"] is False
    assert cfg["rate_hz"] == pytest.approx(100.0)


@pytest.mark.skipif(not BENI_ONNX.is_file(), reason="beni policy.onnx missing")
def test_beni_onnx_step_smoke():
    ort = pytest.importorskip("onnxruntime")
    session = ort.InferenceSession(
        str(BENI_ONNX), providers=["CPUExecutionProvider"]
    )
    policy = BeniBalancePolicy(session)
    vx, wz = policy.step(0.01, vx_cmd=0.5, wz_cmd=0.0, base_yaw_rate=0.0)
    assert policy.last_action.shape == (7,)
    assert len(ACTION_JOINTS) == 7
    # Standing defaults still near nominal after one tick.
    assert abs(policy.q["L1"] - DEFAULT_Q["L1"]) < 0.5
    # Default: no live TF overrides (standing pose stays mesh-baked).
    assert policy.display_angles() == {}
    policy.animate_joints = True
    display = policy.display_angles()
    assert "L1_Link" in display
    # Wheels must not appear in TF overrides (baked meshes would orbit).
    assert "LW_Link" not in display and "RW_Link" not in display
    assert isinstance(vx, float) and isinstance(wz, float)


@pytest.mark.skipif(not BENI_ONNX.is_file(), reason="beni policy.onnx missing")
def test_policy_driver_beni_create():
    pytest.importorskip("onnxruntime")
    driver = PolicyDriver.create_when_enabled(
        {
            "policy": {
                "enabled": True,
                "action_space": "beni",
                "model_path": str(BENI_ONNX),
                "rate_hz": 100.0,
            }
        }
    )
    assert driver is not None
    assert driver.action_space == "beni"
    cmd = driver.compute_command(
        0.01,
        pose=(0.0, 0.0, 0.0),
        velocity=(0.0, 0.0),
        scan=None,
        twist_cmd=(0.3, 0.1),
    )
    assert cmd is not None
    # animate_joints defaults off → empty TF overrides (no kicking).
    assert driver.joint_display_angles == {}


@pytest.mark.skipif(not BENI_ONNX.is_file(), reason="beni policy.onnx missing")
def test_beni_cmd_vel_zero_stops_and_latches():
    """Zero twist must stop; subsequent None ticks must not revive motion."""
    pytest.importorskip("onnxruntime")
    driver = PolicyDriver.create_when_enabled(
        {
            "policy": {
                "enabled": True,
                "action_space": "beni",
                "model_path": str(BENI_ONNX),
                "rate_hz": 100.0,
                "drive_from_wheels": False,
            }
        }
    )
    assert driver is not None
    kwargs = dict(pose=(0.0, 0.0, 0.0), velocity=(0.0, 0.0), scan=None)
    moving = driver.compute_command(0.01, twist_cmd=(0.8, 0.0), **kwargs)
    assert moving is not None and abs(moving[0]) > 1e-3
    stopped = driver.compute_command(0.01, twist_cmd=(0.0, 0.0), **kwargs)
    assert stopped == (0.0, 0.0)
    # Edge-triggered reader: next ticks have no new msg (twist_cmd=None).
    held = driver.compute_command(0.01, twist_cmd=None, **kwargs)
    assert held == (0.0, 0.0)


class _FakeArticulation:
    """Minimal stand-in for BeniArticulation (no Isaac Kit)."""

    ready = True

    def __init__(self) -> None:
        self.applied: list = []
        self.q = {name: float(DEFAULT_Q[name]) for name in ACTION_JOINTS}
        self.dq = {name: 0.0 for name in ACTION_JOINTS}

    def sync_joint_state(self, q, dq) -> None:
        for name in ACTION_JOINTS:
            q[name] = self.q[name]
            dq[name] = self.dq[name]

    def read_angular_velocity(self):
        return (0.1, -0.05, 0.2)

    def projected_gravity(self):
        return (0.0, 0.0, -1.0)

    def apply_policy_action(self, action) -> None:
        self.applied.append(np.asarray(action, dtype=np.float64).copy())

    def display_angles(self):
        return {"L1_Link": 0.05, "R1_Link": -0.02}


@pytest.mark.skipif(not BENI_ONNX.is_file(), reason="beni policy.onnx missing")
def test_policy_driver_binds_articulation_and_applies_action():
    """With PhysX Articulation bound, ONNX actions go to joint drives."""
    pytest.importorskip("onnxruntime")
    driver = PolicyDriver.create_when_enabled(
        {
            "policy": {
                "enabled": True,
                "action_space": "beni",
                "model_path": str(BENI_ONNX),
                "rate_hz": 100.0,
                "animate_joints": False,
            }
        }
    )
    assert driver is not None
    art = _FakeArticulation()
    driver.bind_articulation(art)
    assert driver.beni.animate_joints is True
    cmd = driver.compute_command(
        0.01,
        pose=(0.0, 0.0, 0.0),
        velocity=(0.0, 0.0),
        scan=None,
        twist_cmd=(0.2, 0.0),
    )
    assert cmd is not None
    assert len(art.applied) >= 1
    assert art.applied[0].shape == (7,)
    assert "L1_Link" in driver.joint_display_angles


def test_beni_articulation_fallen_heuristic():
    """fallen() uses projected gravity tilt + low z (no Kit required)."""
    from autosim.backends.isaac.robots.beni import BeniArticulation

    art = BeniArticulation("/World/Beni")
    art._ready = True
    art.art = object()  # mark ready without SingleArticulation

    upright = {"pose": (0.0, 0.0, 0.0, 0.11), "grav": (0.0, 0.0, -1.0)}
    tipped = {"pose": (0.0, 0.0, 0.0, 0.11), "grav": (0.9, 0.0, -0.4)}
    crushed = {"pose": (0.0, 0.0, 0.0, 0.01), "grav": (0.0, 0.0, -1.0)}

    art.read_pose = lambda: upright["pose"]  # type: ignore[method-assign]
    art.projected_gravity = lambda: upright["grav"]  # type: ignore[method-assign]
    assert art.fallen() is False

    art.read_pose = lambda: tipped["pose"]  # type: ignore[method-assign]
    art.projected_gravity = lambda: tipped["grav"]  # type: ignore[method-assign]
    assert art.fallen() is True

    art.read_pose = lambda: crushed["pose"]  # type: ignore[method-assign]
    art.projected_gravity = lambda: upright["grav"]  # type: ignore[method-assign]
    assert art.fallen() is True
