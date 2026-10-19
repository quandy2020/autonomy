# Copyright 2026 The Openbot Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0.

"""Perception eval: lidar disk detection vs simulator obstacle GT."""

from __future__ import annotations

import logging
import math
import time
from collections.abc import Sequence

import numpy as np

from benchmark.common.types import EpisodeResult, EpisodeStatus, Pose2D
from benchmark.simulators.factory import create_simulator
from benchmark.simulators.fake.simulator import FakeSimulator
from benchmark.simulators.world_state import ObstacleDisk
from benchmark.tasks.base import Task
from benchmark.tasks.modes import create_mode

_LOG = logging.getLogger(__name__)


def _detect_from_lidar(
    pose: Pose2D, ranges: np.ndarray, cluster_gap: float = 0.6
) -> list[tuple[float, float]]:
    """Cluster lidar hits into obstacle centroids (xy)."""
    pts: list[tuple[float, float]] = []
    n = ranges.size
    for i, r in enumerate(ranges):
        if not np.isfinite(r) or r >= 7.5:
            continue
        ang = pose.yaw + (2 * math.pi * i / n) - math.pi
        pts.append((pose.x + float(r) * math.cos(ang), pose.y + float(r) * math.sin(ang)))
    if not pts:
        return []
    # Greedy clustering
    clusters: list[list[tuple[float, float]]] = []
    for p in pts:
        placed = False
        for c in clusters:
            cx = sum(q[0] for q in c) / len(c)
            cy = sum(q[1] for q in c) / len(c)
            if math.hypot(p[0] - cx, p[1] - cy) < cluster_gap:
                c.append(p)
                placed = True
                break
        if not placed:
            clusters.append([p])
    dets = []
    for c in clusters:
        if len(c) < 2:
            continue
        dets.append((sum(q[0] for q in c) / len(c), sum(q[1] for q in c) / len(c)))
    return dets


def _match_precision_recall(
    dets: Sequence[tuple[float, float]],
    gt: Sequence[ObstacleDisk],
    match_dist: float = 1.0,
) -> tuple[float, float]:
    if not gt and not dets:
        return 1.0, 1.0
    if not gt:
        return 0.0, 1.0
    if not dets:
        return 1.0, 0.0
    matched = set()
    tp = 0
    for dx, dy in dets:
        best_j = -1
        best_d = match_dist
        for j, obs in enumerate(gt):
            if j in matched:
                continue
            d = math.hypot(dx - obs.x, dy - obs.y)
            if d < best_d:
                best_d = d
                best_j = j
        if best_j >= 0:
            matched.add(best_j)
            tp += 1
    precision = tp / len(dets)
    recall = tp / len(gt)
    return precision, recall


class PerceptionTask(Task):
    name = "perception"

    def run(self) -> list[EpisodeResult]:
        episodes = int(self.task_cfg.get("episodes", 5))
        match_dist = float(self.task_cfg.get("match_dist", 1.0))
        mode = create_mode(
            str(self.task_cfg.get("mode", "scenario")),
            dict(self.task_cfg.get("mode_config") or self.task_cfg),
        )
        sim = self.simulator or create_simulator(self.config)
        results: list[EpisodeResult] = []
        _LOG.info("PerceptionTask episodes=%s", episodes)

        with sim:
            for ep in range(episodes):
                setup = mode.setup(ep, sim)
                if hasattr(sim, "configure_world"):
                    pose = sim.configure_world(  # type: ignore[attr-defined]
                        obstacles=setup.obstacles,
                        pedestrians=0,
                        moving_obstacles=0,
                        seed=ep,
                        start=setup.start,
                        goal=setup.goal,
                    )
                else:
                    pose = sim.reset(seed=ep)
                t0 = time.perf_counter()
                # One sensing step
                from benchmark.common.types import Twist2D

                step = sim.step(Twist2D())
                latency_ms = (time.perf_counter() - t0) * 1000.0
                obs = step.info.get("observation") or {}
                lidar = np.asarray(obs.get("lidar", []), dtype=np.float32)
                dets = _detect_from_lidar(step.pose, lidar) if lidar.size else []
                gt: list[ObstacleDisk] = []
                if isinstance(sim, FakeSimulator):
                    gt = list(sim.world.obstacles)
                elif hasattr(sim, "_twin") and isinstance(getattr(sim, "_twin"), FakeSimulator):
                    gt = list(sim._twin.world.obstacles)  # type: ignore[attr-defined]
                precision, recall = _match_precision_recall(dets, gt, match_dist=match_dist)
                _LOG.info(
                    "perception ep=%s P=%.2f R=%.2f dets=%s gt=%s lat=%.1fms",
                    ep,
                    precision,
                    recall,
                    len(dets),
                    len(gt),
                    latency_ms,
                )
                results.append(
                    EpisodeResult(
                        episode_id=ep,
                        status=EpisodeStatus.SUCCESS,
                        metrics={
                            "precision": precision,
                            "recall": recall,
                            "latency_ms": latency_ms,
                            "detections": float(len(dets)),
                            "gt_obstacles": float(len(gt)),
                        },
                        info={"stub": False, "pose": [pose.x, pose.y, pose.yaw]},
                    )
                )
        return results
