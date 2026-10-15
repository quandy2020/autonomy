# Copyright 2026 The Openbot Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0.

"""SLAM / localization evaluation against simulator ground truth."""

from __future__ import annotations

import logging
import math
from typing import List, Tuple

import numpy as np

from benchmark.bridge.autolink_bridge import AutolinkBridge
from benchmark.core.geometry import reached
from benchmark.core.types import EpisodeResult, EpisodeStatus, Pose2D
from benchmark.evaluation.metrics.slam_metrics import slam_summary
from benchmark.planning.factory import create_planner
from benchmark.simulators.factory import create_simulator
from benchmark.tasks.base import Task
from benchmark.tasks.episode_runner import _apply_setup
from benchmark.tasks.modes import create_mode

_LOG = logging.getLogger(__name__)


def _noisy_odom(gt: Pose2D, rng: np.random.Generator, sigma: float) -> Pose2D:
    return Pose2D(
        x=gt.x + float(rng.normal(0, sigma)),
        y=gt.y + float(rng.normal(0, sigma)),
        yaw=gt.yaw + float(rng.normal(0, sigma * 0.5)),
    )


class SlamTask(Task):
    """Drive a path while comparing estimated pose to GT.

    Estimate source priority:
    1. Autolink ``/odom`` when connected
    2. Synthetic noisy odometry (models localization drift for offline CI)
    """

    name = "slam"

    def run(self) -> List[EpisodeResult]:
        episodes = int(self.task_cfg.get("episodes", 1))
        timeout_steps = int(self.task_cfg.get("timeout_steps", 200))
        noise = float(self.task_cfg.get("odom_noise", 0.05))
        use_autolink = bool(self.task_cfg.get("autolink", False))
        tol = float(self.task_cfg.get("goal_tolerance_m", 0.4))
        mode = create_mode(
            str(self.task_cfg.get("mode", "scenario")),
            dict(self.task_cfg.get("mode_config") or self.task_cfg),
        )
        planner = create_planner(self.config.section("planner"))
        sim = self.simulator or create_simulator(self.config)
        bridge = AutolinkBridge(node_name="benchmark_slam")
        bridge_ok = bridge.connect(subscribe_odom=True) if use_autolink else False
        results: List[EpisodeResult] = []

        _LOG.info("SlamTask episodes=%s autolink=%s noise=%.3f", episodes, bridge_ok, noise)
        with sim:
            for ep in range(episodes):
                rng = np.random.default_rng(ep + 7)
                setup = mode.setup(ep, sim)
                pose = _apply_setup(sim, setup, seed=ep)
                goal = setup.goal
                gt_traj: List[Pose2D] = [pose]
                est_traj: List[Pose2D] = [_noisy_odom(pose, rng, noise)]
                hits: List[Tuple[float, float]] = [(pose.x, pose.y)]
                status = EpisodeStatus.TIMEOUT
                for step_i in range(timeout_steps):
                    if hasattr(planner, "set_observation") and hasattr(sim, "world"):
                        planner.set_observation(sim.world.observation())  # type: ignore[attr-defined]
                    cmd = planner.compute(pose, goal)
                    if bridge_ok:
                        bridge.publish_twist(cmd)
                        bridge.spin_once()
                    result = sim.step(cmd)
                    pose = result.pose
                    gt_traj.append(pose)
                    live = bridge.latest_pose() if bridge_ok else None
                    est_traj.append(live if live is not None else _noisy_odom(pose, rng, noise))
                    # Ray endpoints as crude mapping hits
                    obs = result.info.get("observation") or {}
                    lidar = obs.get("lidar")
                    if lidar is not None and hasattr(sim, "world"):
                        ranges = np.asarray(lidar, dtype=np.float32)
                        n = ranges.size
                        for i, r in enumerate(ranges[:: max(1, n // 12)]):
                            ang = pose.yaw + (2 * math.pi * i / max(1, n // 12)) - math.pi
                            if r < 7.5:
                                hits.append((pose.x + float(r) * math.cos(ang), pose.y + float(r) * math.sin(ang)))
                    hits.append((pose.x, pose.y))
                    if result.info.get("collision"):
                        status = EpisodeStatus.COLLISION
                        break
                    if reached(pose, goal, tol):
                        status = EpisodeStatus.GOAL_REACHED
                        break
                metrics = slam_summary(gt_traj, est_traj, hits)
                metrics["trajectory_length"] = float(
                    sum(
                        math.hypot(gt_traj[i].x - gt_traj[i - 1].x, gt_traj[i].y - gt_traj[i - 1].y)
                        for i in range(1, len(gt_traj))
                    )
                )
                _LOG.info(
                    "slam ep=%s status=%s ate=%.3f rpe=%.3f cov=%.3f",
                    ep,
                    status.value,
                    metrics["ate"],
                    metrics["rpe"],
                    metrics["map_coverage"],
                )
                results.append(
                    EpisodeResult(
                        episode_id=ep,
                        status=status,
                        steps=len(gt_traj) - 1,
                        metrics=metrics,
                        info={"autolink": bridge_ok, "stub": False},
                    )
                )
        bridge.shutdown()
        return results
