# Copyright 2026 The Openbot Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0.

"""ONNX / heuristic policy planner (local Twist output, no ROS)."""

from __future__ import annotations

import logging
import math
from pathlib import Path
from typing import Any

import numpy as np

from benchmark.common.types import Pose2D, Twist2D
from benchmark.tasks.planning.base import Planner
from benchmark.tasks.planning.pure_pursuit import PurePursuitPlanner

_LOG = logging.getLogger(__name__)


class PolicyPlanner(Planner):
    """Observation → Twist planner.

    Prefers an ONNX model (``model_path``). When missing, falls back to a
    lidar-aware reactive policy that mimics a lightweight DRL deployment.
    """

    name = "policy"

    def __init__(self, config: dict[str, Any] | None = None) -> None:
        super().__init__(config)
        self.max_v = float(self.config.get("desired_linear_vel", self.config.get("max_vx", 0.4)))
        self.max_w = float(self.config.get("max_angular_vel", self.config.get("max_wz", 1.2)))
        self.session: Any = None
        self._fallback = PurePursuitPlanner(config)
        self._last_obs: dict[str, Any] | None = None
        model_path = str(self.config.get("model_path") or "").strip()
        if model_path:
            self.session = self._load_onnx(model_path)

    @staticmethod
    def _load_onnx(model_path: str) -> Any:
        path = Path(model_path).expanduser()
        if not path.is_file():
            _LOG.warning("policy model not found: %s", path)
            return None
        try:
            import onnxruntime as ort
        except ImportError:
            _LOG.warning("onnxruntime not installed; policy uses reactive fallback")
            return None
        try:
            session = ort.InferenceSession(str(path), providers=["CPUExecutionProvider"])
            _LOG.info("loaded policy ONNX: %s", path)
            return session
        except Exception as exc:  # noqa: BLE001
            _LOG.warning("failed to load ONNX %s: %s", path, exc)
            return None

    def set_observation(self, obs: dict[str, Any]) -> None:
        """Inject latest lidar/goal observation from the simulator step."""
        self._last_obs = obs

    def compute(self, pose: Pose2D, goal: Pose2D) -> Twist2D:
        if self.session is not None and self._last_obs is not None:
            twist = self._infer_onnx(pose, goal)
            if twist is not None:
                return twist
        return self._reactive(pose, goal)

    def _infer_onnx(self, pose: Pose2D, goal: Pose2D) -> Twist2D | None:
        assert self.session is not None and self._last_obs is not None
        lidar = np.asarray(self._last_obs.get("lidar"), dtype=np.float32).ravel()
        goal_vec = np.array(
            [goal.x - pose.x, goal.y - pose.y, goal.yaw - pose.yaw], dtype=np.float32
        )
        feeds: dict[str, np.ndarray] = {}
        for inp in self.session.get_inputs():
            name = inp.name
            shape = [d if isinstance(d, int) else 1 for d in (inp.shape or [])]
            flat = int(np.prod(shape)) if shape else lidar.size + goal_vec.size
            if "laser" in name.lower() or "lidar" in name.lower():
                vec = lidar
            elif "goal" in name.lower() or "kinematic" in name.lower():
                vec = goal_vec
            else:
                vec = np.concatenate([lidar, goal_vec])
            if vec.size < flat:
                pad = np.zeros(flat, dtype=np.float32)
                pad[: vec.size] = vec
                vec = pad
            feeds[name] = vec[:flat].reshape(shape).astype(np.float32)
        try:
            outs = self.session.run(None, feeds)
        except Exception as exc:  # noqa: BLE001
            _LOG.debug("onnx infer failed: %s", exc)
            return None
        action = np.asarray(outs[0]).ravel()
        if action.size < 2:
            return None
        return Twist2D(
            linear=float(np.clip(action[0], 0.0, self.max_v)),
            angular=float(np.clip(action[1], -self.max_w, self.max_w)),
        )

    def _reactive(self, pose: Pose2D, goal: Pose2D) -> Twist2D:
        """Lidar-gated pure pursuit: slow/turn when frontal beams are short."""
        base = self._fallback.compute(pose, goal)
        obs = self._last_obs or {}
        lidar = np.asarray(obs.get("lidar", []), dtype=np.float32)
        if lidar.size == 0:
            return base
        # Beams near forward (±30°)
        n = lidar.size
        forward = []
        for i, r in enumerate(lidar):
            ang = (2 * math.pi * i / n) - math.pi
            if abs(ang) < math.pi / 6:
                forward.append(float(r))
        if not forward:
            return base
        min_f = min(forward)
        if min_f < 0.8:
            base = Twist2D(
                linear=base.linear * 0.3,
                angular=base.angular + (1.0 if forward[0] > forward[-1] else -1.0),
            )
        elif min_f < 1.5:
            base = Twist2D(linear=base.linear * 0.6, angular=base.angular)
        return Twist2D(
            linear=float(np.clip(base.linear, 0.0, self.max_v)),
            angular=float(np.clip(base.angular, -self.max_w, self.max_w)),
        )

