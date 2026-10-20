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

"""Optional ONNX planar-twist policy driver."""

from __future__ import annotations

import logging
from pathlib import Path
from typing import Any, Dict, Mapping, Optional, Tuple

import numpy as np

from autosim.runtime.paths import project_root

logger = logging.getLogger(__name__)


def policy_config_from_habitat(habitat: Mapping[str, Any]) -> dict[str, Any]:
    """Build a policy config dict from ``habitat.policy`` (or ``habitat.isaac.policy``)."""
    block: Mapping[str, Any] = {}
    if isinstance(habitat.get("policy"), Mapping):
        block = habitat["policy"]  # type: ignore[assignment]
    else:
        isaac = habitat.get("isaac")
        if isinstance(isaac, Mapping) and isinstance(isaac.get("policy"), Mapping):
            block = isaac["policy"]  # type: ignore[assignment]
    action_space = str(block.get("action_space") or "twist").strip().lower()
    if action_space == "beni":
        raise ValueError(
            "habitat.policy.action_space=beni was removed; use action_space=twist"
        )
    model_path = str(block.get("model_path") or "").strip()
    return {
        "enabled": bool(block.get("enabled", False)),
        "model_path": model_path,
        "action_space": action_space,
        "rate_hz": float(block.get("rate_hz", 10.0)),
        "exclusive": bool(block.get("exclusive", False)),
        "max_vx": float(block.get("max_vx", 0.5)),
        "max_wz": float(block.get("max_wz", 1.0)),
        "laser_bins": int(block.get("laser_bins", 180)),
        "input_kinematic": str(block.get("input_kinematic") or "observation_kinematic"),
        "input_laser": str(block.get("input_laser") or "observation_laser"),
        "output_action": str(block.get("output_action") or "action"),
        "use_laser": bool(block.get("use_laser", True)),
        "wheel_radius": float(block.get("wheel_radius", 0.033)),
        "drive_from_wheels": bool(block.get("drive_from_wheels", False)),
    }


class PolicyDriver:
    """Run an ONNXRuntime twist policy at ``rate_hz``; no-op when disabled.

    ``action_space``:
      - ``twist``: kinematic ``[1,6]`` (+ optional laser) → ``(vx, wz)``
    """

    def __init__(self, config: Mapping[str, Any]) -> None:
        self.config = dict(config)
        self.enabled = bool(config.get("enabled", False)) and bool(
            config.get("model_path")
        )
        self.session: Any = None
        self.elapsed = 0.0
        self.period = 1.0 / max(float(config.get("rate_hz", 10.0)), 1e-3)
        self.last_command = (0.0, 0.0)
        self._hold_cmd = (0.0, 0.0)
        self._joint_display: Dict[str, float] = {}
        if self.enabled:
            self.session = self.load_session(str(config["model_path"]))
            if self.session is None:
                self.enabled = False

    @property
    def action_space(self) -> str:
        """Policy action space (``twist``)."""
        return str(self.config.get("action_space") or "twist").strip().lower()

    @staticmethod
    def load_session(model_path: str) -> Any:
        """Load an ONNX inference session, or return None on failure."""
        path = Path(model_path).expanduser()
        if not path.is_absolute():
            path = (project_root() / path).resolve()
        if not path.is_file():
            logger.warning("policy model not found: %s", path)
            return None
        try:
            import onnxruntime as ort
        except ImportError:
            logger.warning(
                "policy.enabled but onnxruntime is not installed; "
                "pip install onnxruntime"
            )
            return None
        try:
            session = ort.InferenceSession(
                str(path), providers=["CPUExecutionProvider"]
            )
        except Exception as exc:  # noqa: BLE001
            logger.warning("failed to load policy ONNX %s: %s", path, exc)
            return None
        logger.info("RL policy loaded: %s", path)
        return session

    @classmethod
    def create_when_enabled(
        cls, habitat: Mapping[str, Any]
    ) -> Optional["PolicyDriver"]:
        """Return a ready driver when ``habitat.policy.enabled`` and the model load."""
        config = policy_config_from_habitat(habitat)
        if not config["enabled"]:
            return None
        driver = cls(config)
        return driver if driver.enabled else None

    maybe_create = create_when_enabled

    @property
    def exclusive(self) -> bool:
        """True when teleop ``cmd_vel`` must not override the policy."""
        return bool(self.config.get("exclusive", False))

    @property
    def joint_display_angles(self) -> Dict[str, float]:
        """Child-link → revolute angle for TF (empty for twist robots)."""
        return dict(self._joint_display)

    def compute_command(
        self,
        dt: float,
        *,
        pose: Tuple[float, float, float],
        velocity: Tuple[float, float],
        scan: Optional[np.ndarray],
        twist_cmd: Optional[Tuple[float, float]] = None,
    ) -> Optional[Tuple[float, float]]:
        """Return ``(vx, wz)`` for the plant."""
        del twist_cmd
        if not self.enabled or self.session is None:
            return None
        return self._compute_twist(dt, pose=pose, velocity=velocity, scan=scan)

    tick = compute_command

    def _compute_twist(
        self,
        dt: float,
        *,
        pose: Tuple[float, float, float],
        velocity: Tuple[float, float],
        scan: Optional[np.ndarray],
    ) -> Optional[Tuple[float, float]]:
        self.elapsed += float(dt)
        if self.elapsed < self.period and self.last_command != (0.0, 0.0):
            return self.last_command if self.exclusive else None
        if self.elapsed < self.period:
            return None
        self.elapsed = 0.0
        feeds = self.build_feeds(pose, velocity, scan)
        try:
            outputs = self.session.run(None, feeds)
        except Exception as exc:  # noqa: BLE001
            logger.debug("policy infer failed: %s", exc)
            return None
        if not outputs:
            return None
        action = np.asarray(outputs[0], dtype=np.float64).reshape(-1)
        if action.size < 2:
            return None
        vx = float(np.clip(action[0], -self.config["max_vx"], self.config["max_vx"]))
        wz = float(np.clip(action[1], -self.config["max_wz"], self.config["max_wz"]))
        self.last_command = (vx, wz)
        return self.last_command

    def build_feeds(
        self,
        pose: Tuple[float, float, float],
        velocity: Tuple[float, float],
        scan: Optional[np.ndarray],
    ) -> dict[str, np.ndarray]:
        """Assemble ONNX input tensors from pose, velocity, and optional laser."""
        x, y, yaw = pose
        vx, wz = velocity
        kinematic = np.asarray([[x, y, yaw, vx, 0.0, wz]], dtype=np.float32)
        feeds: dict[str, np.ndarray] = {
            str(self.config["input_kinematic"]): kinematic,
        }
        if self.config.get("use_laser", True):
            bins = max(1, int(self.config.get("laser_bins", 180)))
            laser = np.full((1, bins), 30.0, dtype=np.float32)
            if scan is not None:
                ranges = np.asarray(scan, dtype=np.float32).reshape(-1)
                finite = np.isfinite(ranges)
                ranges = np.where(finite, ranges, 30.0)
                if ranges.size == bins:
                    laser[0] = ranges
                elif ranges.size > 0:
                    indices = np.linspace(0, ranges.size - 1, bins).astype(int)
                    laser[0] = ranges[indices]
            feeds[str(self.config["input_laser"])] = laser
        try:
            names = {inp.name for inp in self.session.get_inputs()}
            feeds = {key: value for key, value in feeds.items() if key in names}
        except Exception:
            pass
        return feeds
