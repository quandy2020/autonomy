# Copyright 2026 The Openbot Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0.

"""Hardware HAL-in-the-loop with safety limits and optional twin."""

from __future__ import annotations

import logging
import time

from benchmark.bridge.autolink_bridge import AutolinkBridge
from benchmark.common.config import BenchmarkConfig
from benchmark.common.geometry import reached
from benchmark.common.types import EpisodeResult, EpisodeStatus, Twist2D
from benchmark.simulators.factory import create_simulator
from benchmark.tasks.base import Task
from benchmark.tasks.modes import create_mode
from benchmark.tasks.planning.factory import create_planner

_LOG = logging.getLogger(__name__)


def _clamp_twist(twist: Twist2D, max_v: float, max_w: float) -> Twist2D:
    v = max(-max_v, min(max_v, twist.linear))
    # HIL usually forbids reverse at high speed; keep forward-biased.
    v = max(0.0, v)
    w = max(-max_w, min(max_w, twist.angular))
    return Twist2D(linear=v, angular=w)


class HilTask(Task):
    name = "hil"

    def run(self) -> list[EpisodeResult]:
        safety = dict(self.task_cfg.get("safety") or {})
        max_v = float(safety.get("max_linear_vel", self.task_cfg.get("max_linear_vel", 0.3)))
        max_w = float(safety.get("max_angular_vel", 0.8))
        timeout_s = float(self.task_cfg.get("timeout_s", 30))
        tol = float(self.task_cfg.get("goal_tolerance_m", 0.5))
        use_twin = bool(self.task_cfg.get("use_sim_twin", True))
        estop = bool(self.task_cfg.get("estop", False))

        raw = dict(self.config.raw)
        sim_block = dict(raw.get("simulator") or {})
        sim_block["name"] = "hil"
        sim_block["use_sim_twin"] = use_twin
        raw["simulator"] = sim_block
        cfg = BenchmarkConfig(raw=raw, path=self.config.path)

        mode = create_mode(
            str(self.task_cfg.get("mode", "scenario")),
            dict(self.task_cfg.get("mode_config") or self.task_cfg),
        )
        planner = create_planner(self.config.section("planner"))
        sim = self.simulator or create_simulator(cfg)
        bridge = AutolinkBridge(
            node_name="benchmark_hil",
            cmd_vel=str(self.task_cfg.get("cmd_vel", "/cmd_vel")),
        )
        bridge_ok = bridge.connect(subscribe_odom=True)

        _LOG.info(
            "HilTask twin=%s autolink=%s max_v=%.2f estop=%s",
            use_twin,
            bridge_ok,
            max_v,
            estop,
        )

        results: list[EpisodeResult] = []
        with sim:
            setup = mode.setup(0, sim)
            if hasattr(sim, "configure_world"):
                pose = sim.configure_world(  # type: ignore[attr-defined]
                    obstacles=int(self.task_cfg.get("obstacles", 0)),
                    pedestrians=0,
                    moving_obstacles=0,
                    seed=0,
                    start=setup.start,
                    goal=setup.goal,
                )
            else:
                pose = sim.reset(seed=0)
            goal = setup.goal
            t0 = time.monotonic()
            steps = 0
            status = EpisodeStatus.TIMEOUT
            path = 0.0
            prev = pose

            if estop:
                bridge.publish_twist(Twist2D())
                status = EpisodeStatus.ABORTED
            else:
                while time.monotonic() - t0 < timeout_s:
                    steps += 1
                    if hasattr(planner, "set_observation"):
                        world = getattr(sim, "world", None)
                        if world is None and hasattr(sim, "_twin"):
                            world = getattr(sim._twin, "world", None)  # type: ignore[attr-defined]
                        if world is not None:
                            planner.set_observation(world.observation())  # type: ignore[attr-defined]
                    cmd = _clamp_twist(planner.compute(pose, goal), max_v, max_w)
                    if bridge_ok:
                        bridge.publish_twist(cmd)
                        bridge.spin_once()
                    step = sim.step(cmd)
                    live = bridge.latest_pose() if bridge_ok else None
                    pose = live or step.pose
                    path += ((pose.x - prev.x) ** 2 + (pose.y - prev.y) ** 2) ** 0.5
                    prev = pose
                    if step.info.get("collision"):
                        status = EpisodeStatus.COLLISION
                        bridge.publish_twist(Twist2D())  # soft stop
                        break
                    if reached(pose, goal, tol):
                        status = EpisodeStatus.GOAL_REACHED
                        bridge.publish_twist(Twist2D())
                        break

            results.append(
                EpisodeResult(
                    episode_id=0,
                    status=status,
                    steps=steps,
                    path_length=path,
                    time_s=time.monotonic() - t0,
                    metrics={
                        "success": 1.0 if status == EpisodeStatus.GOAL_REACHED else 0.0,
                        "max_linear_vel": max_v,
                    },
                    info={
                        "stub": False,
                        "autolink": bridge_ok,
                        "use_sim_twin": use_twin,
                        "estop": estop,
                    },
                )
            )
        bridge.shutdown()
        return results
