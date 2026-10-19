# Copyright 2026 The Openbot Authors (duyongquan)

"""Beni teleop / ONNX action-sequence regression tests."""

from __future__ import annotations

from pathlib import Path
from typing import List, Optional, Sequence, Tuple

import numpy as np
import pytest

from autosim.policy.beni import DEFAULT_Q, BeniBalancePolicy
from autosim.policy.beni_sequences import BENI_SEQUENCES, SequenceStep
from autosim.policy.driver import PolicyDriver

ROOT = Path(__file__).resolve().parents[1]
BENI_ONNX = ROOT / "robot" / "beni" / "models" / "policy.onnx"

Twist = Tuple[float, float]


def _make_driver(*, drive_from_wheels: bool = False) -> PolicyDriver:
    pytest.importorskip("onnxruntime")
    driver = PolicyDriver.create_when_enabled(
        {
            "policy": {
                "enabled": True,
                "action_space": "beni",
                "model_path": str(BENI_ONNX),
                "rate_hz": 100.0,
                "drive_from_wheels": drive_from_wheels,
                "animate_joints": False,
                "base_height": 0.11,
            }
        }
    )
    assert driver is not None and driver.beni is not None
    return driver


def _run_segment(
    driver: PolicyDriver,
    *,
    vx: float,
    wz: float,
    duration_s: float,
    dt: float = 0.01,
) -> List[Twist]:
    """Drive ``duration_s`` with one twist publish then None (edge-triggered)."""
    kwargs = dict(pose=(0.0, 0.0, 0.0), velocity=(0.0, 0.0), scan=None)
    out: List[Twist] = []
    steps = max(1, int(round(duration_s / dt)))
    for i in range(steps):
        twist: Optional[Twist] = (vx, wz) if i == 0 else None
        cmd = driver.compute_command(dt, twist_cmd=twist, **kwargs)
        assert cmd is not None
        out.append((float(cmd[0]), float(cmd[1])))
    return out


def _assert_finite_joints(policy: BeniBalancePolicy) -> None:
    for name, q in policy.q.items():
        assert np.isfinite(q), f"q[{name}] not finite: {q}"
        assert abs(q) < 5.0, f"q[{name}] exploded: {q}"
    for name, dq in policy.dq.items():
        assert np.isfinite(dq), f"dq[{name}] not finite: {dq}"
        assert abs(dq) < 50.0, f"dq[{name}] exploded: {dq}"
    assert np.all(np.isfinite(policy.last_action))
    assert np.all(np.abs(policy.last_action) <= 1.0 + 1e-5)


@pytest.mark.skipif(not BENI_ONNX.is_file(), reason="beni policy.onnx missing")
@pytest.mark.parametrize("name", sorted(BENI_SEQUENCES.keys()))
def test_beni_named_sequence(name: str):
    """Each named maneuver must stay finite and honor stop."""
    driver = _make_driver()
    steps = BENI_SEQUENCES[name]
    last: Optional[Twist] = None
    for label, vx, wz, dur in steps:
        traces = _run_segment(driver, vx=vx, wz=wz, duration_s=dur)
        last = traces[-1]
        _assert_finite_joints(driver.beni)
        if abs(vx) < 1e-9 and abs(wz) < 1e-9:
            assert traces[0] == (0.0, 0.0), f"{name}/{label}: stop not immediate"
            assert all(t == (0.0, 0.0) for t in traces), f"{name}/{label}: drift after stop"
        else:
            # After ramp, plant should track teleop hold (drive_from_wheels=false).
            assert abs(last[0] - vx) < 0.08 and abs(last[1] - wz) < 0.08, (
                f"{name}/{label}: expected ≈({vx},{wz}), got {last}"
            )


@pytest.mark.skipif(not BENI_ONNX.is_file(), reason="beni policy.onnx missing")
def test_beni_sequence_forward_then_turn_then_stop():
    """Classic teleop chain: advance → yaw → hard stop → None latch."""
    driver = _make_driver()
    chain: Sequence[SequenceStep] = (
        ("fwd", 0.9, 0.0, 0.40),
        ("left", 0.0, 1.0, 0.30),
        ("stop", 0.0, 0.0, 0.05),
    )
    for _, vx, wz, dur in chain:
        _run_segment(driver, vx=vx, wz=wz, duration_s=dur)
        _assert_finite_joints(driver.beni)
    # Edge-triggered silence after stop.
    kwargs = dict(pose=(0.0, 0.0, 0.0), velocity=(0.0, 0.0), scan=None)
    for _ in range(20):
        cmd = driver.compute_command(0.01, twist_cmd=None, **kwargs)
        assert cmd == (0.0, 0.0)


@pytest.mark.skipif(not BENI_ONNX.is_file(), reason="beni policy.onnx missing")
@pytest.mark.parametrize("height", [0.10, 0.11, 0.12])
def test_beni_height_band(height: float):
    """Standing height command stays in Open Beni training band."""
    ort = pytest.importorskip("onnxruntime")
    session = ort.InferenceSession(
        str(BENI_ONNX), providers=["CPUExecutionProvider"]
    )
    policy = BeniBalancePolicy(session, base_height=height)
    assert policy.base_height == pytest.approx(height)
    for vx, wz in ((0.0, 0.0), (0.4, 0.0), (0.0, 0.5), (-0.3, -0.4)):
        policy.step(0.01, vx_cmd=vx, wz_cmd=wz)
        _assert_finite_joints(policy)
        obs = policy.build_obs(vx_cmd=vx, wz_cmd=wz, base_pose_cmd=(0.0, 0.0, height))
        assert obs[28:31].tolist() == pytest.approx([0.0, 0.0, height])


@pytest.mark.skipif(not BENI_ONNX.is_file(), reason="beni policy.onnx missing")
def test_beni_long_mixed_sequence_no_overflow():
    """Multi-second mixed cmds must not blow joint / obs state."""
    driver = _make_driver()
    pattern = [
        (0.5, 0.0),
        (0.0, 1.0),
        (-0.4, 0.0),
        (0.0, -1.0),
        (0.3, 0.5),
        (0.0, 0.0),
    ]
    for vx, wz in pattern * 3:
        _run_segment(driver, vx=vx, wz=wz, duration_s=0.25)
        _assert_finite_joints(driver.beni)
        # Standing defaults stay in soft limits when animate_joints is off.
        for joint in ("L1", "R1", "L2", "R2"):
            assert abs(driver.beni.q[joint] - DEFAULT_Q[joint]) < 1.5
    assert driver.joint_display_angles == {}
