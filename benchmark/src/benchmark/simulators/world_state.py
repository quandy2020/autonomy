# Copyright 2026 The Openbot Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0.

"""Lightweight 2D world state for Arena-style episodes (no external engine)."""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any

import numpy as np

from benchmark.common.types import Pose2D, Twist2D


@dataclass
class ObstacleDisk:
    x: float
    y: float
    radius: float
    vx: float = 0.0
    vy: float = 0.0


@dataclass
class PedestrianAgent:
    x: float
    y: float
    goal_x: float
    goal_y: float
    radius: float = 0.25
    preferred_speed: float = 1.0
    vx: float = 0.0
    vy: float = 0.0


@dataclass
class WorldState:
    """Kinematic arena used by FakeSimulator and RL env."""

    width: float = 20.0
    height: float = 20.0
    robot_radius: float = 0.2
    obstacles: list[ObstacleDisk] = field(default_factory=list)
    pedestrians: list[PedestrianAgent] = field(default_factory=list)
    pose: Pose2D = field(default_factory=Pose2D)
    goal: Pose2D = field(default_factory=lambda: Pose2D(x=5.0, y=0.0))
    dt: float = 0.1
    collision: bool = False
    min_clearance: float = float("inf")
    rng: np.random.Generator = field(default_factory=lambda: np.random.default_rng(0))
    human_model: Any = None

    def reset(
        self,
        *,
        seed: int | None = None,
        start: Pose2D | None = None,
        goal: Pose2D | None = None,
    ) -> Pose2D:
        if seed is not None:
            self.rng = np.random.default_rng(seed)
        self.collision = False
        self.min_clearance = float("inf")
        self.pose = start or Pose2D()
        self.goal = goal or Pose2D(x=float(self.width * 0.35), y=0.0)
        return self.pose

    def clear_dynamic(self) -> None:
        self.obstacles.clear()
        self.pedestrians.clear()

    def spawn_random_obstacles(
        self, count: int, radius_range: tuple[float, float] = (0.3, 0.8)
    ) -> None:
        for _ in range(max(0, count)):
            r = float(self.rng.uniform(*radius_range))
            x, y = self._sample_free(r + self.robot_radius + 0.5)
            self.obstacles.append(ObstacleDisk(x=x, y=y, radius=r))

    def spawn_random_pedestrians(self, count: int) -> None:
        for _ in range(max(0, count)):
            x, y = self._sample_free(0.5)
            gx, gy = self._sample_free(0.5)
            self.pedestrians.append(
                PedestrianAgent(
                    x=x,
                    y=y,
                    goal_x=gx,
                    goal_y=gy,
                    preferred_speed=float(self.rng.uniform(0.6, 1.2)),
                )
            )

    def _on_start_goal_corridor(self, x: float, y: float, width: float = 1.2) -> bool:
        """Reject samples that block the straight corridor from start → goal."""
        sx, sy = self.pose.x, self.pose.y
        gx, gy = self.goal.x, self.goal.y
        vx, vy = gx - sx, gy - sy
        length = math.hypot(vx, vy)
        if length < 1e-6:
            return False
        ux, uy = vx / length, vy / length
        wx, wy = x - sx, y - sy
        proj = wx * ux + wy * uy
        if proj < 0.0 or proj > length:
            return False
        lat = abs(-uy * wx + ux * wy)
        return lat < width

    def _sample_free(self, margin: float) -> tuple[float, float]:
        for _ in range(200):
            x = float(self.rng.uniform(-self.width / 2 + margin, self.width / 2 - margin))
            y = float(self.rng.uniform(-self.height / 2 + margin, self.height / 2 - margin))
            if math.hypot(x - self.pose.x, y - self.pose.y) < 1.5:
                continue
            if math.hypot(x - self.goal.x, y - self.goal.y) < 1.0:
                continue
            if self._on_start_goal_corridor(x, y):
                continue
            if self._clearance_at(x, y, 0.0) > margin:
                return x, y
        return 0.0, float(self.height / 4)

    def _clearance_at(self, x: float, y: float, radius: float) -> float:
        clear = float("inf")
        for obs in self.obstacles:
            clear = min(clear, math.hypot(x - obs.x, y - obs.y) - obs.radius - radius)
        for ped in self.pedestrians:
            clear = min(clear, math.hypot(x - ped.x, y - ped.y) - ped.radius - radius)
        return clear

    def step_robot(self, twist: Twist2D) -> Pose2D:
        yaw = self.pose.yaw + twist.angular * self.dt
        x = self.pose.x + twist.linear * math.cos(self.pose.yaw) * self.dt
        y = self.pose.y + twist.linear * math.sin(self.pose.yaw) * self.dt
        # Soft wall clamp
        half_w, half_h = self.width / 2, self.height / 2
        x = max(-half_w + self.robot_radius, min(half_w - self.robot_radius, x))
        y = max(-half_h + self.robot_radius, min(half_h - self.robot_radius, y))
        self.pose = Pose2D(x=x, y=y, yaw=yaw)
        clear = self._clearance_at(x, y, self.robot_radius)
        self.min_clearance = min(self.min_clearance, clear)
        if clear < 0.0:
            self.collision = True
        return self.pose

    def step_dynamics(self) -> None:
        """Advance obstacles / pedestrians one tick."""
        for obs in self.obstacles:
            obs.x += obs.vx * self.dt
            obs.y += obs.vy * self.dt
            if abs(obs.x) > self.width / 2 - obs.radius:
                obs.vx *= -1
            if abs(obs.y) > self.height / 2 - obs.radius:
                obs.vy *= -1

        if self.human_model is not None:
            self.human_model.step(self)
            return

        # Built-in social-force-lite fallback
        for ped in self.pedestrians:
            dx, dy = ped.goal_x - ped.x, ped.goal_y - ped.y
            dist = math.hypot(dx, dy)
            if dist < 0.4:
                ped.goal_x, ped.goal_y = self._sample_free(0.5)
                dx, dy = ped.goal_x - ped.x, ped.goal_y - ped.y
                dist = max(math.hypot(dx, dy), 1e-6)
            fx = ped.preferred_speed * dx / dist
            fy = ped.preferred_speed * dy / dist
            rx, ry = ped.x - self.pose.x, ped.y - self.pose.y
            rd = math.hypot(rx, ry) + 1e-6
            if rd < 2.0:
                fx += 1.5 * rx / rd
                fy += 1.5 * ry / rd
            for other in self.pedestrians:
                if other is ped:
                    continue
                ox, oy = ped.x - other.x, ped.y - other.y
                od = math.hypot(ox, oy) + 1e-6
                if od < 1.2:
                    fx += 0.8 * ox / od
                    fy += 0.8 * oy / od
            ped.vx = 0.7 * ped.vx + 0.3 * fx
            ped.vy = 0.7 * ped.vy + 0.3 * fy
            speed = math.hypot(ped.vx, ped.vy)
            if speed > ped.preferred_speed:
                ped.vx *= ped.preferred_speed / speed
                ped.vy *= ped.preferred_speed / speed
            ped.x += ped.vx * self.dt
            ped.y += ped.vy * self.dt

    def lidar(self, n_beams: int = 36, max_range: float = 8.0) -> np.ndarray:
        ranges = np.full((n_beams,), max_range, dtype=np.float32)
        for i in range(n_beams):
            ang = self.pose.yaw + (2 * math.pi * i / n_beams) - math.pi
            hit = max_range
            for s in np.linspace(0.05, max_range, 40):
                x = self.pose.x + s * math.cos(ang)
                y = self.pose.y + s * math.sin(ang)
                if abs(x) > self.width / 2 or abs(y) > self.height / 2:
                    hit = s
                    break
                if self._clearance_at(x, y, 0.0) < 0.05:
                    hit = s
                    break
            ranges[i] = hit
        return ranges

    def observation(self) -> dict:
        goal_dx = self.goal.x - self.pose.x
        goal_dy = self.goal.y - self.pose.y
        return {
            "lidar": self.lidar(),
            "goal": np.array([goal_dx, goal_dy, self.goal.yaw - self.pose.yaw], dtype=np.float32),
            "pose": np.array([self.pose.x, self.pose.y, self.pose.yaw], dtype=np.float32),
        }
