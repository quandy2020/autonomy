# Copyright 2026 The Openbot Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0.

"""Gymnasium-compatible social navigation env (Arena training backend)."""

from __future__ import annotations

import math
from typing import Any, Dict, Optional, Tuple

import numpy as np

from benchmark.core.geometry import reached
from benchmark.core.types import Pose2D, Twist2D
from benchmark.rl.rewards.reward_functions import navigation_reward
from benchmark.rl.spaces.observation import flatten_obs
from benchmark.simulators.fake.simulator import FakeSimulator
from benchmark.tasks.modes import create_mode


class NavEnv:
    """Minimal gym-like API without hard dependency on gymnasium."""

    metadata = {"render_modes": []}

    def __init__(self, config: Optional[Dict[str, Any]] = None) -> None:
        self.config = dict(config or {})
        self.max_steps = int(self.config.get("max_steps", 300))
        self.goal_tol = float(self.config.get("goal_tolerance_m", 0.35))
        self.mode = create_mode(
            str(self.config.get("mode", "random")),
            dict(self.config.get("mode_config") or {}),
        )
        self.sim = FakeSimulator(
            {
                "rate_hz": self.config.get("rate_hz", 10),
                "width": self.config.get("width", 20),
                "height": self.config.get("height", 20),
            }
        )
        self.sim.start()
        self._step = 0
        self._prev_dist = 0.0
        # spaces described as shapes for SB3 wrappers
        obs_dim = 36 + 3
        self.observation_space_shape = (obs_dim,)
        self.action_space_low = np.array([-0.0, -1.5], dtype=np.float32)
        self.action_space_high = np.array([0.5, 1.5], dtype=np.float32)

    def reset(self, *, seed: Optional[int] = None) -> Tuple[np.ndarray, Dict[str, Any]]:
        setup = self.mode.setup(int(seed or 0), self.sim)
        self.sim.configure_world(
            obstacles=setup.obstacles,
            pedestrians=setup.pedestrians,
            moving_obstacles=setup.moving_obstacles,
            seed=seed,
            start=setup.start,
            goal=setup.goal,
        )
        self._step = 0
        pose = self.sim.get_pose()
        goal = self.sim.get_goal()
        self._prev_dist = math.hypot(goal.x - pose.x, goal.y - pose.y)
        obs = flatten_obs(self.sim.world.observation())
        return obs, {"setup": setup.info}

    def step(self, action: np.ndarray) -> Tuple[np.ndarray, float, bool, bool, Dict[str, Any]]:
        self._step += 1
        action = np.asarray(action, dtype=np.float32).ravel()
        v = float(np.clip(action[0], self.action_space_low[0], self.action_space_high[0]))
        w = float(np.clip(action[1], self.action_space_low[1], self.action_space_high[1]))
        result = self.sim.step(Twist2D(linear=v, angular=w))
        pose = result.pose
        goal = self.sim.get_goal()
        done_collision = bool(result.info.get("collision"))
        done_goal = reached(pose, goal, self.goal_tol)
        reward, parts = navigation_reward(
            pose=pose,
            goal=goal,
            prev_dist=self._prev_dist,
            collision=done_collision,
            reached=done_goal,
        )
        self._prev_dist = math.hypot(goal.x - pose.x, goal.y - pose.y)
        terminated = done_collision or done_goal
        truncated = self._step >= self.max_steps
        obs = flatten_obs(self.sim.world.observation())
        info = {"reward_parts": parts, "collision": done_collision, "goal": done_goal}
        return obs, reward, terminated, truncated, info

    def close(self) -> None:
        self.sim.stop()
