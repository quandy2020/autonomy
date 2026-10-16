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

"""Open Beni 7-DoF balance ONNX (obs[40] + GRU → actions[7]).

Contract matches open-beni ``web/game/physics.mjs`` / ``beni_robot.py``:
action order L1,R1,L2,R2,LW,RW,head_yaw; standing defaults baked into Autoviz
mesh origins so TF publishes ``q - q_default`` for position joints.
"""

from __future__ import annotations

import math
from typing import Dict, Mapping, MutableMapping, Optional, Sequence, Tuple

import numpy as np

ACTION_JOINTS: Tuple[str, ...] = (
    "L1",
    "R1",
    "L2",
    "R2",
    "LW",
    "RW",
    "head_yaw_joint",
)
OBS_POS_JOINTS: Tuple[str, ...] = (
    "L1",
    "R1",
    "head_yaw_joint",
    "L2",
    "R2",
)
OBS_VEL_JOINTS: Tuple[str, ...] = OBS_POS_JOINTS + ("LW", "RW")

DEFAULT_Q: Dict[str, float] = {
    "L1": 0.3,
    "R1": 0.3,
    "L2": 0.5,
    "R2": 0.5,
    "LW": 0.0,
    "RW": 0.0,
    "head_yaw_joint": 0.0,
    "head_roll_joint": 0.0,
}

ACTION_SCALE: Dict[str, float] = {
    "L1": 0.25,
    "R1": 0.25,
    "L2": 0.25,
    "R2": 0.25,
    "LW": 20.0,
    "RW": 20.0,
    "head_yaw_joint": 0.5,
}

# URDF child link for each actuated joint (for TF overrides).
JOINT_CHILD_LINK: Dict[str, str] = {
    "L1": "L1_Link",
    "R1": "R1_Link",
    "L2": "L2_Link",
    "R2": "R2_Link",
    "LW": "LW_Link",
    "RW": "RW_Link",
    "head_yaw_joint": "head_intermediate",
    "head_roll_joint": "head",
}

# Soft PD-ish tracking gains for kinematic joint integration (no MuJoCo).
_POS_STIFFNESS = {
    "L1": 0.5,
    "R1": 0.5,
    "L2": 0.5,
    "R2": 0.5,
    "head_yaw_joint": 3.0,
}
_POS_DAMPING = {
    "L1": 0.02,
    "R1": 0.02,
    "L2": 0.02,
    "R2": 0.02,
    "head_yaw_joint": 0.05,
}
_WHEEL_DAMPING = 0.004


class BeniBalancePolicy:
    """ONNX balance policy + kinematic joint state for Autoviz TF."""

    OBS_SIZE = 40
    HIDDEN = (1, 1, 512)

    def __init__(
        self,
        session: object,
        *,
        wheel_radius: float = 0.038,
        base_height: float = 0.11,
        animate_joints: bool = False,
    ) -> None:
        self.session = session
        self.wheel_radius = float(wheel_radius)
        # Clamp to Open Beni training band so the policy stays in-distribution.
        self.base_height = float(np.clip(base_height, 0.10, 0.12))
        # Without MuJoCo/PhysX feedback, ONNX hip/knee targets thrash; Autoviz
        # meshes already bake standing pose, so TF stays at URDF defaults.
        self.animate_joints = bool(animate_joints)
        self.q: Dict[str, float] = dict(DEFAULT_Q)
        self.dq: Dict[str, float] = {name: 0.0 for name in DEFAULT_Q}
        self.last_action = np.zeros(7, dtype=np.float32)
        self.hidden = np.zeros(self.HIDDEN, dtype=np.float32)
        self.command = np.zeros(2, dtype=np.float64)  # vx, wz (ramped)
        self.input_names = [inp.name for inp in session.get_inputs()]  # type: ignore[attr-defined]
        self.output_names = [out.name for out in session.get_outputs()]  # type: ignore[attr-defined]

    def reset(self) -> None:
        """Reset joints / GRU / command to standing defaults."""
        self.q = dict(DEFAULT_Q)
        self.dq = {name: 0.0 for name in DEFAULT_Q}
        self.last_action = np.zeros(7, dtype=np.float32)
        self.hidden = np.zeros(self.HIDDEN, dtype=np.float32)
        self.command[:] = 0.0

    def ramp_command(self, vx: float, wz: float, dt: float) -> Tuple[float, float]:
        """Browser-style command slew (≈ ±0.03 / ±0.06 per 10 ms step)."""
        step_vx = 3.0 * float(dt)  # 0.03 per 0.01 s
        step_wz = 6.0 * float(dt)
        self.command[0] += float(np.clip(vx - self.command[0], -step_vx, step_vx))
        self.command[1] += float(np.clip(wz - self.command[1], -step_wz, step_wz))
        return float(self.command[0]), float(self.command[1])

    def build_obs(
        self,
        *,
        vx_cmd: float,
        wz_cmd: float,
        base_ang_vel: Sequence[float] = (0.0, 0.0, 0.0),
        projected_gravity: Sequence[float] = (0.0, 0.0, -1.0),
        base_pose_cmd: Sequence[float] = (0.0, 0.0, 0.11),
        gimbal_cmd: Sequence[float] = (0.0, 1.0),
        head_ang_vel: Sequence[float] = (0.0, 0.0, 0.0),
        jump_cmd: Sequence[float] = (0.0, 0.0, 0.0, 0.0),
    ) -> np.ndarray:
        """Assemble float32 observation ``[40]`` (Open Beni layout)."""
        obs = np.zeros(self.OBS_SIZE, dtype=np.float32)
        obs[0:3] = np.asarray(base_ang_vel, dtype=np.float32).reshape(3)
        obs[3:6] = np.asarray(projected_gravity, dtype=np.float32).reshape(3)
        obs[6:9] = (float(vx_cmd), 0.0, float(wz_cmd))
        for index, name in enumerate(OBS_POS_JOINTS):
            obs[9 + index] = float(
                np.clip(self.q[name] - DEFAULT_Q[name], -2.0, 2.0)
            )
        for index, name in enumerate(OBS_VEL_JOINTS):
            obs[14 + index] = float(np.clip(self.dq[name], -40.0, 40.0))
        obs[21:28] = np.clip(self.last_action, -1.0, 1.0)
        obs[28:31] = np.asarray(base_pose_cmd, dtype=np.float32).reshape(3)
        obs[31:33] = np.asarray(gimbal_cmd, dtype=np.float32).reshape(2)
        obs[33:36] = np.asarray(head_ang_vel, dtype=np.float32).reshape(3)
        obs[36:40] = np.asarray(jump_cmd, dtype=np.float32).reshape(4)
        return obs

    def infer(self, obs: np.ndarray) -> np.ndarray:
        """Run ONNX; update GRU hidden; return actions ``[7]``."""
        feeds: Dict[str, np.ndarray] = {}
        for name in self.input_names:
            lower = name.lower()
            if "obs" in lower:
                feeds[name] = obs.reshape(1, self.OBS_SIZE).astype(np.float32)
            elif "h" in lower:
                feeds[name] = self.hidden.astype(np.float32)
        outputs = self.session.run(None, feeds)  # type: ignore[attr-defined]
        action = np.asarray(outputs[0], dtype=np.float32).reshape(-1)
        if action.size < 7:
            action = np.pad(action, (0, 7 - action.size))
        action = action[:7]
        if len(outputs) > 1:
            hidden = np.asarray(outputs[1], dtype=np.float32)
            if hidden.size == self.hidden.size:
                self.hidden = hidden.reshape(self.HIDDEN)
        self.last_action = action.astype(np.float32)
        return action

    def apply_action(self, action: np.ndarray, dt: float) -> None:
        """Kinematic track of PD targets (no PhysX / MuJoCo)."""
        dt = float(max(dt, 1e-4))
        for index, name in enumerate(ACTION_JOINTS):
            a = float(np.clip(action[index], -1.0, 1.0))
            scale = ACTION_SCALE[name]
            if name in ("LW", "RW"):
                omega_des = a * scale
                # Track wheel rate for obs / twist only — do not accumulate a
                # display angle (baked wheel meshes orbit if LW/RW TF spins).
                self.dq[name] = float(
                    np.clip(0.7 * self.dq[name] + 0.3 * omega_des, -40.0, 40.0)
                )
                self.q[name] = 0.0
                continue
            q_des = DEFAULT_Q[name] + a * scale
            kp = _POS_STIFFNESS[name]
            kd = _POS_DAMPING[name]
            err = q_des - self.q[name]
            # Critically-damped-ish velocity command, then integrate.
            omega = kp * 20.0 * err - kd * 50.0 * self.dq[name]
            self.dq[name] = float(np.clip(omega, -20.0, 20.0))
            # Soft joint limits around standing defaults (kinematic only).
            self.q[name] = float(
                np.clip(self.q[name] + self.dq[name] * dt, -1.5, 2.0)
            )

    def step(
        self,
        dt: float,
        *,
        vx_cmd: float,
        wz_cmd: float,
        base_yaw_rate: float = 0.0,
    ) -> Tuple[float, float]:
        """Ramp command → obs → ONNX → joints. Returns wheel-derived ``(vx, wz)``."""
        vx_cmd, wz_cmd = self.ramp_command(vx_cmd, wz_cmd, dt)
        obs = self.build_obs(
            vx_cmd=vx_cmd,
            wz_cmd=wz_cmd,
            base_ang_vel=(0.0, 0.0, float(base_yaw_rate)),
            base_pose_cmd=(0.0, 0.0, self.base_height),
        )
        action = self.infer(obs)
        self.apply_action(action, dt)
        return self.wheel_twist()

    def wheel_twist(self) -> Tuple[float, float]:
        """Differential-drive twist from wheel angular rates."""
        r = self.wheel_radius
        # Axis is (0,-1,0); positive dq sign matched to Open Beni training.
        v_l = float(self.dq["LW"]) * r
        v_r = float(self.dq["RW"]) * r
        vx = 0.5 * (v_l + v_r)
        # Track ≈ 0.16 m (profile wheel_separation).
        wz = (v_r - v_l) / 0.16
        return vx, wz

    def display_angles(self) -> Dict[str, float]:
        """Joint angles for TF: hip/knee/head relative to mesh-baked defaults.

        Default (``animate_joints=False``): empty — keep URDF/mesh standing pose.
        Kinematic ONNX without physics makes L1/R1/L2/R2 thrash ("legs kicking").

        When enabled, wheel links are still omitted: OBJ visuals bake standing
        pose into ``visual/origin``, so LW/RW spin orbits the axle.
        """
        if not self.animate_joints:
            return {}
        out: Dict[str, float] = {}
        for name, link in JOINT_CHILD_LINK.items():
            if name not in self.q or name in ("LW", "RW"):
                continue
            out[link] = float(self.q[name] - DEFAULT_Q.get(name, 0.0))
        return out

    def joint_state_map(self) -> Dict[str, float]:
        """``joint_name → q`` for ``/joint_states`` publishers."""
        return {name: float(self.q[name]) for name in ACTION_JOINTS}


def _rpy_to_quat(
    roll: float, pitch: float, yaw: float
) -> Tuple[float, float, float, float]:
    """URDF fixed-axis RPY → quaternion (x, y, z, w)."""
    half_roll = 0.5 * float(roll)
    half_pitch = 0.5 * float(pitch)
    half_yaw = 0.5 * float(yaw)
    cr, sr = math.cos(half_roll), math.sin(half_roll)
    cp, sp = math.cos(half_pitch), math.sin(half_pitch)
    cy, sy = math.cos(half_yaw), math.sin(half_yaw)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def axis_angle_rpy(
    base_rpy: Tuple[float, float, float],
    axis: Tuple[float, float, float],
    angle: float,
) -> Tuple[float, float, float]:
    """Compose URDF joint origin RPY with a rotation about ``axis`` by ``angle``."""
    q0 = _rpy_to_quat(base_rpy[0], base_rpy[1], base_rpy[2])
    ax = np.asarray(axis, dtype=np.float64)
    norm = float(np.linalg.norm(ax))
    if norm < 1e-12 or abs(angle) < 1e-12:
        return base_rpy
    ax = ax / norm
    half = 0.5 * float(angle)
    s = math.sin(half)
    q1 = (ax[0] * s, ax[1] * s, ax[2] * s, math.cos(half))
    # q = q0 ⊗ q1 (apply joint angle after fixed origin).
    x0, y0, z0, w0 = q0
    x1, y1, z1, w1 = q1
    q = (
        w0 * x1 + x0 * w1 + y0 * z1 - z0 * y1,
        w0 * y1 - x0 * z1 + y0 * w1 + z0 * x1,
        w0 * z1 + x0 * y1 - y0 * x1 + z0 * w1,
        w0 * w1 - x0 * x1 - y0 * y1 - z0 * z1,
    )
    return _quat_to_rpy(q)


def _quat_to_rpy(q: Tuple[float, float, float, float]) -> Tuple[float, float, float]:
    """Quaternion (x,y,z,w) → URDF fixed-axis RPY."""
    x, y, z, w = q
    # roll (x)
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    # pitch (y)
    sinp = 2.0 * (w * y - z * x)
    sinp = float(np.clip(sinp, -1.0, 1.0))
    pitch = math.asin(sinp)
    # yaw (z)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return (roll, pitch, yaw)
