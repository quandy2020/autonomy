# Copyright 2026 The Openbot Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0.

"""Fake Arena-style 2D simulator with obstacles, pedestrians, and lidar."""

from __future__ import annotations

import logging
from typing import Any, Dict, Optional

from benchmark.core.types import Pose2D, StepResult, Twist2D
from benchmark.simulators.base import SimulatorAdapter
from benchmark.simulators.world_state import WorldState

_LOG = logging.getLogger(__name__)


class FakeSimulator(SimulatorAdapter):
    name = "fake"

    def __init__(self, config: Optional[Dict[str, Any]] = None) -> None:
        super().__init__(config)
        rate = float(self.config.get("rate_hz", 10) or 10)
        self.world = WorldState(
            width=float(self.config.get("width", 20)),
            height=float(self.config.get("height", 20)),
            robot_radius=float(self.config.get("robot_radius", 0.2)),
            dt=1.0 / rate,
        )
        self._step = 0
        self._backend: Any = None
        self._install_human_model(self.config.get("humans") or self.config.get("human"))

    def _install_human_model(self, humans_cfg: Any) -> None:
        if not humans_cfg:
            return
        if isinstance(humans_cfg, dict) and humans_cfg.get("enabled") is False:
            return
        try:
            from benchmark.humans.factory import create_human_model

            self.world.human_model = create_human_model(
                humans_cfg if isinstance(humans_cfg, dict) else {"name": str(humans_cfg)}
            )
            _LOG.info("human model: %s", getattr(self.world.human_model, "name", "?"))
        except Exception as exc:  # noqa: BLE001
            _LOG.warning("human model unavailable: %s", exc)

    def start(self) -> None:
        autosim_cfg = self.config.get("autosim_config")
        if autosim_cfg:
            try:
                self._try_autosim(str(autosim_cfg))
            except Exception as exc:
                _LOG.warning("autosim fake unavailable (%s); using Arena kinematic world", exc)
        self._started = True
        _LOG.info(
            "FakeSimulator started (backend=%s size=%.0fx%.0f)",
            "autosim" if self._backend else "kinematic",
            self.world.width,
            self.world.height,
        )

    def _try_autosim(self, path: str) -> None:
        from autosim.backend.registry import create_backend  # type: ignore
        from autosim.config.settings import Config  # type: ignore

        settings = Config.load(path)
        self._backend = create_backend(settings.data)

    def configure_world(
        self,
        *,
        obstacles: int = 0,
        pedestrians: int = 0,
        seed: Optional[int] = None,
        start: Optional[Pose2D] = None,
        goal: Optional[Pose2D] = None,
        moving_obstacles: int = 0,
    ) -> Pose2D:
        self.world.clear_dynamic()
        pose = self.world.reset(seed=seed, start=start, goal=goal)
        self.world.spawn_random_obstacles(obstacles)
        if moving_obstacles > 0:
            for obs in self.world.obstacles[-moving_obstacles:]:
                obs.vx = float(self.world.rng.uniform(-0.4, 0.4))
                obs.vy = float(self.world.rng.uniform(-0.4, 0.4))
        if self.world.human_model is not None and pedestrians > 0:
            self.world.human_model.spawn(self.world, pedestrians)
        else:
            self.world.spawn_random_pedestrians(pedestrians)
        self._step = 0
        return pose

    def reset(self, *, seed: Optional[int] = None) -> Pose2D:
        obstacles = int(self.config.get("obstacles", 15))
        pedestrians = int(self.config.get("pedestrians", 5))
        moving = int(self.config.get("moving_obstacles", 3))
        return self.configure_world(
            obstacles=obstacles,
            pedestrians=pedestrians,
            moving_obstacles=moving,
            seed=seed,
        )

    def set_goal(self, goal: Pose2D) -> None:
        self.world.goal = goal

    def get_goal(self) -> Pose2D:
        return self.world.goal

    def step(self, twist: Twist2D) -> StepResult:
        self._step += 1
        self.world.step_dynamics()
        pose = self.world.step_robot(twist)
        info = {
            "collision": self.world.collision,
            "min_clearance": self.world.min_clearance,
            "observation": self.world.observation(),
        }
        return StepResult(
            step=self._step,
            pose=pose,
            twist=twist,
            done=self.world.collision,
            info=info,
        )

    def get_pose(self) -> Pose2D:
        return self.world.pose

    def stop(self) -> None:
        self._backend = None
        self._started = False
