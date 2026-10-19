# Copyright 2026 The Openbot Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0.

"""Dataset collection via local JSONL (+ optional autosim LeRobot)."""

from __future__ import annotations

import logging
from pathlib import Path

import numpy as np

from benchmark.common.geometry import reached
from benchmark.common.types import EpisodeResult, EpisodeStatus
from benchmark.evaluation.dataset import EpisodeRecorder
from benchmark.simulators.factory import create_simulator
from benchmark.tasks.base import Task
from benchmark.tasks.episode_runner import _apply_setup
from benchmark.tasks.modes import create_mode
from benchmark.tasks.planning.factory import create_planner

_LOG = logging.getLogger(__name__)


class CollectTask(Task):
    name = "collect"

    def run(self) -> list[EpisodeResult]:
        episodes = int(self.task_cfg.get("episodes", 1))
        frames_target = int(self.task_cfg.get("episode_frames", 100))
        output = Path(str(self.task_cfg.get("output", "data/datasets/collect")))
        fps = float(self.task_cfg.get("fps", 10))
        use_lerobot = bool(self.task_cfg.get("lerobot", False))
        channels = {
            "pose": bool(self.task_cfg.get("pose", True)),
            "velocity": bool(self.task_cfg.get("velocity", True)),
            "action": bool(self.task_cfg.get("action", True)),
            "include_laser": bool(self.task_cfg.get("include_laser", True)),
            "include_image": bool(self.task_cfg.get("include_image", False)),
            "lerobot": use_lerobot,
        }
        tol = float(self.task_cfg.get("goal_tolerance_m", 0.4))
        mode = create_mode(
            str(self.task_cfg.get("mode", "scenario")),
            dict(self.task_cfg.get("mode_config") or self.task_cfg),
        )
        planner = create_planner(self.config.section("planner"))
        sim = self.simulator or create_simulator(self.config)
        recorder = EpisodeRecorder(
            output, fps=fps, use_lerobot=use_lerobot, channels=channels
        )
        results: list[EpisodeResult] = []

        enabled = [k for k, v in channels.items() if v]
        _LOG.info(
            "CollectTask: %s episodes → %s (channels=%s)",
            episodes,
            output,
            enabled,
        )
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
                    image = None
                    obs = step.info.get("observation") or {}
                    if channels["include_laser"] and "lidar" in obs:
                        laser = np.asarray(obs["lidar"], dtype=np.float32)
                    if channels["include_image"]:
                        for key in ("rgb", "image", "camera"):
                            if key in obs and obs[key] is not None:
                                image = obs[key]
                                break
                    if obs and hasattr(planner, "set_observation"):
                        planner.set_observation(obs)  # type: ignore[attr-defined]
                    dt = 1.0 / fps
                    recorder.maybe_record(
                        dt,
                        pose=step.pose,
                        twist=step.twist,
                        command=cmd,
                        laser=laser,
                        image=image,
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
