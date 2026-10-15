# Copyright 2026 The Openbot Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0.

"""Dataset collection via local JSONL (+ optional autosim LeRobot)."""

from __future__ import annotations

import logging
from pathlib import Path
from typing import List

import numpy as np

from benchmark.core.types import EpisodeResult, EpisodeStatus, Twist2D
from benchmark.data.recorder import EpisodeRecorder
from benchmark.planning.factory import create_planner
from benchmark.simulators.factory import create_simulator
from benchmark.tasks.base import Task
from benchmark.tasks.episode_runner import _apply_setup
from benchmark.tasks.modes import create_mode
from benchmark.core.geometry import reached

_LOG = logging.getLogger(__name__)


class CollectTask(Task):
    name = "collect"

    def run(self) -> List[EpisodeResult]:
        episodes = int(self.task_cfg.get("episodes", 1))
        frames_target = int(self.task_cfg.get("episode_frames", 100))
        output = Path(str(self.task_cfg.get("output", "data/datasets/collect")))
        fps = float(self.task_cfg.get("fps", 10))
        use_lerobot = bool(self.task_cfg.get("lerobot", False))
        tol = float(self.task_cfg.get("goal_tolerance_m", 0.4))
        mode = create_mode(
            str(self.task_cfg.get("mode", "scenario")),
            dict(self.task_cfg.get("mode_config") or self.task_cfg),
        )
        planner = create_planner(self.config.section("planner"))
        sim = self.simulator or create_simulator(self.config)
        recorder = EpisodeRecorder(output, fps=fps, use_lerobot=use_lerobot)
        results: List[EpisodeResult] = []

        _LOG.info("CollectTask: %s episodes → %s (lerobot=%s)", episodes, output, use_lerobot)
        with sim:
            for ep in range(episodes):
                setup = mode.setup(ep, sim)
                pose = _apply_setup(sim, setup, seed=ep)
                recorder.start_episode(ep)
                goal = setup.goal
                if hasattr(planner, "set_observation") and hasattr(sim, "world"):
                    planner.set_observation(sim.world.observation())  # type: ignore[attr-defined]
                n = 0
                while n < frames_target:
                    cmd = planner.compute(pose, goal)
                    step = sim.step(cmd)
                    laser = None
                    obs = step.info.get("observation") or {}
                    if "lidar" in obs:
                        laser = np.asarray(obs["lidar"], dtype=np.float32)
                    if obs and hasattr(planner, "set_observation"):
                        planner.set_observation(obs)  # type: ignore[attr-defined]
                    dt = 1.0 / fps
                    recorder.maybe_record(
                        dt,
                        pose=step.pose,
                        twist=step.twist,
                        command=cmd,
                        laser=laser,
                    )
                    pose = step.pose
                    n = len(recorder.frames)
                    if step.info.get("collision") or reached(pose, goal, tol):
                        # Keep rolling a bit after goal for IL; break if collision.
                        if step.info.get("collision"):
                            break
                        if n >= max(10, frames_target // 5):
                            break
                path = recorder.flush()
                results.append(
                    EpisodeResult(
                        episode_id=ep,
                        status=EpisodeStatus.SUCCESS,
                        steps=len(recorder.frames),
                        metrics={"frames": float(len(recorder.frames))},
                        info={"output": str(path), "stub": False},
                    )
                )
        return results
