# Copyright 2026 The Openbot Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0.

"""Shared episode loop with trajectory recording (Arena Evaluation input)."""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Sequence

from benchmark.core.geometry import distance, reached
from benchmark.core.types import EpisodeResult, EpisodeStatus, Pose2D, Twist2D
from benchmark.planning.base import Planner
from benchmark.planning.factory import create_planner
from benchmark.simulators.base import SimulatorAdapter
from benchmark.simulators.fake.simulator import FakeSimulator
from benchmark.tasks.modes.base import EpisodeSetup, TaskMode
from benchmark.tasks.modes.dynamic import DynamicMode
from benchmark.tasks.modes.explore import ExploreMode
from benchmark.utils.social_metrics import update_social_step


@dataclass
class TrajectoryPoint:
    t: float
    pose: Pose2D
    twist: Twist2D
    clearance: float = float("inf")


@dataclass
class EpisodeTrace:
    episode_id: int
    setup: EpisodeSetup
    points: List[TrajectoryPoint] = field(default_factory=list)
    status: EpisodeStatus = EpisodeStatus.RUNNING
    collisions: int = 0
    coverage: float = 0.0
    time_in_private_zone: float = 0.0
    time_facing_pedestrians: float = 0.0
    time_seen_by_pedestrians: float = 0.0

    @property
    def path_length(self) -> float:
        total = 0.0
        for a, b in zip(self.points, self.points[1:]):
            total += distance(a.pose, b.pose)
        return total

    @property
    def time_s(self) -> float:
        if len(self.points) < 2:
            return 0.0
        return self.points[-1].t - self.points[0].t

    @property
    def min_clearance(self) -> float:
        if not self.points:
            return float("inf")
        return min(p.clearance for p in self.points)

    def twists(self) -> List[Twist2D]:
        return [p.twist for p in self.points]


def _apply_setup(sim: SimulatorAdapter, setup: EpisodeSetup, seed: int) -> Pose2D:
    if hasattr(sim, "configure_world"):
        return sim.configure_world(  # type: ignore[no-any-return, attr-defined]
            obstacles=setup.obstacles,
            pedestrians=setup.pedestrians,
            moving_obstacles=setup.moving_obstacles,
            seed=seed,
            start=setup.start,
            goal=setup.goal,
        )
    pose = sim.reset(seed=seed)
    if hasattr(sim, "set_goal"):
        sim.set_goal(setup.goal)  # type: ignore[attr-defined]
    return pose


def run_episode(
    simulator: SimulatorAdapter,
    mode: TaskMode,
    planner: Planner,
    *,
    episode_id: int = 0,
    timeout_s: float = 60.0,
    goal_tolerance: float = 0.35,
) -> EpisodeTrace:
    setup = mode.setup(episode_id, simulator)
    pose = _apply_setup(simulator, setup, seed=episode_id)
    if isinstance(mode, DynamicMode):
        mode.apply_map(simulator)
    trace = EpisodeTrace(episode_id=episode_id, setup=setup)
    t0 = time.monotonic()
    last_t = 0.0
    trace.points.append(TrajectoryPoint(t=0.0, pose=pose, twist=Twist2D(), clearance=float("inf")))

    waypoints: Sequence[Pose2D] = setup.waypoints or [setup.goal]
    wp_idx = 0
    current_goal = waypoints[0]

    # Seed observation for learning / hybrid planners before first compute.
    if isinstance(simulator, FakeSimulator) and hasattr(planner, "set_observation"):
        planner.set_observation(simulator.world.observation())  # type: ignore[attr-defined]

    while time.monotonic() - t0 < timeout_s:
        twist = planner.compute(pose, current_goal)
        step = simulator.step(twist)
        pose = step.pose
        clear = float(step.info.get("min_clearance", float("inf")))
        now = time.monotonic() - t0
        dt = max(1e-3, now - last_t)
        last_t = now
        if isinstance(simulator, FakeSimulator):
            social = update_social_step(
                robot_x=pose.x,
                robot_y=pose.y,
                robot_yaw=pose.yaw,
                pedestrians=simulator.world.pedestrians,
                dt=dt,
            )
            trace.time_in_private_zone += social["private"]
            trace.time_facing_pedestrians += social["facing"]
            trace.time_seen_by_pedestrians += social["seen"]
        obs = step.info.get("observation")
        if obs is not None and hasattr(planner, "set_observation"):
            planner.set_observation(obs)  # type: ignore[attr-defined]
        if step.info.get("collision"):
            trace.collisions += 1
            trace.status = EpisodeStatus.COLLISION
            trace.points.append(
                TrajectoryPoint(t=now, pose=pose, twist=twist, clearance=clear)
            )
            return trace

        if isinstance(mode, ExploreMode):
            trace.coverage = mode.update_coverage(pose)
            if trace.coverage >= float(mode.config.get("coverage_target", 0.6)):
                trace.status = EpisodeStatus.GOAL_REACHED
                trace.points.append(
                    TrajectoryPoint(t=now, pose=pose, twist=twist, clearance=clear)
                )
                return trace

        if reached(pose, current_goal, goal_tolerance):
            wp_idx += 1
            if wp_idx >= len(waypoints):
                trace.status = EpisodeStatus.GOAL_REACHED
                trace.points.append(
                    TrajectoryPoint(t=now, pose=pose, twist=twist, clearance=clear)
                )
                return trace
            current_goal = waypoints[wp_idx]
            if isinstance(simulator, FakeSimulator):
                simulator.set_goal(current_goal)

        trace.points.append(
            TrajectoryPoint(t=now, pose=pose, twist=twist, clearance=clear)
        )

    trace.status = EpisodeStatus.TIMEOUT
    return trace


def trace_to_result(trace: EpisodeTrace) -> EpisodeResult:
    success = trace.status == EpisodeStatus.GOAL_REACHED
    return EpisodeResult(
        episode_id=trace.episode_id,
        status=trace.status,
        steps=max(0, len(trace.points) - 1),
        path_length=trace.path_length,
        time_s=trace.time_s,
        collisions=trace.collisions,
        metrics={
            "success": 1.0 if success else 0.0,
            "path_length": trace.path_length,
            "time_to_goal": trace.time_s,
            "collisions": float(trace.collisions),
            "min_clearance": trace.min_clearance if trace.min_clearance < 1e8 else 0.0,
            "coverage": trace.coverage,
            "time_in_private_zone": trace.time_in_private_zone,
            "time_facing_pedestrians": trace.time_facing_pedestrians,
            "time_seen_by_pedestrians": trace.time_seen_by_pedestrians,
        },
        info={"mode": trace.setup.info.get("mode"), **trace.setup.info},
    )


def make_planner(config: Dict[str, Any]) -> Planner:
    return create_planner(config)
