# Copyright 2026 The Openbot Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0.

"""Operational missions: navigate / dock / explore-mapping / patrol / teleop."""

from __future__ import annotations

import logging
import math
from typing import Any

import numpy as np

from benchmark.common.geometry import reached
from benchmark.common.types import EpisodeResult, EpisodeStatus, Pose2D, Twist2D
from benchmark.simulators.factory import create_simulator
from benchmark.tasks.base import Task
from benchmark.tasks.episode_runner import (
    EpisodeTrace,
    TrajectoryPoint,
    _apply_setup,
    make_planner,
    run_episode,
    trace_to_result,
)
from benchmark.tasks.modes import create_mode

_LOG = logging.getLogger(__name__)

# Dashboard / catalog labels (zh) — 编排任务（可组合、可设测试次数）.
MISSION_CATALOG: list[dict[str, Any]] = [
    {
        "id": "navigate",
        "label": "导航",
        "description": "目标点导航（Arena modes）",
        "kind": "orchestration",
        "default_episodes": 3,
        "rpc": "NavigationService/Navigate",
    },
    {
        "id": "dock",
        "label": "回充",
        "description": "驶向充电桩位姿并精停",
        "kind": "orchestration",
        "default_episodes": 1,
        "rpc": "ChargeService/Return",
    },
    {
        "id": "follow",
        "label": "跟随",
        "description": "跟随目标（gRPC FollowService）",
        "kind": "orchestration",
        "default_episodes": 1,
        "rpc": "FollowService/Follow",
    },
    {
        "id": "teleop",
        "label": "遥操",
        "description": "开环/脚本速度指令遥操回放",
        "kind": "orchestration",
        "default_episodes": 1,
        "rpc": "TeleopService/Velocity",
    },
    {
        "id": "explore_mapping",
        "label": "探索建图",
        "description": "覆盖式探索，采集建图轨迹",
        "kind": "orchestration",
        "default_episodes": 1,
        "rpc": "ExplorationService/Explore",
    },
    {
        "id": "patrol",
        "label": "巡检",
        "description": "多航点循环巡检",
        "kind": "orchestration",
        "default_episodes": 2,
        "rpc": "NavigationService/Navigate",
    },
]

# 命令任务：直接对应 automsgs/proto/rpcs 的 gRPC 入口.
# fields.path 用点号路径，支持数组下标（如 waypoints.0.pose.position.x）。
NAMED_RPC_CATALOG: list[dict[str, Any]] = [
    {
        "id": "rpc.navigate",
        "label": "去导航点",
        "description": "发一个目标位姿，机器人开始导航",
        "group": "导航",
        "service": "NavigationService",
        "method": "Navigate",
        "rpc": "automsgs.rpcs.navigation.NavigationService/Navigate",
        "default_body": {
            "goal_id": "cmd-nav",
            "waypoints": [
                {
                    "header": {"frame_id": "map"},
                    "pose": {
                        "position": {"x": 1.0, "y": 0.0, "z": 0.0},
                        "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
                    },
                }
            ],
            "options": {"timeout_seconds": 120.0},
        },
        "fields": [
            {"path": "goal_id", "label": "goal_id", "type": "text"},
            {"path": "waypoints.0.pose.position.x", "label": "x (m)", "type": "number", "step": 0.1},
            {"path": "waypoints.0.pose.position.y", "label": "y (m)", "type": "number", "step": 0.1},
            {"path": "waypoints.0.header.frame_id", "label": "frame", "type": "text"},
            {"path": "options.timeout_seconds", "label": "超时 (s)", "type": "number", "step": 1},
        ],
    },
    {
        "id": "rpc.dock",
        "label": "回充",
        "description": "返回充电桩并对接",
        "group": "充电",
        "service": "ChargeService",
        "method": "Return",
        "rpc": "automsgs.rpcs.charge.ChargeService/Return",
        "default_body": {"station_id": "", "goal_id": "cmd-dock"},
        "fields": [
            {"path": "goal_id", "label": "goal_id", "type": "text"},
            {"path": "station_id", "label": "充电站 ID", "type": "text", "placeholder": "空=默认"},
        ],
    },
    {
        "id": "rpc.undock",
        "label": "离桩",
        "description": "离开充电桩",
        "group": "充电",
        "service": "ChargeService",
        "method": "Leave",
        "rpc": "automsgs.rpcs.charge.ChargeService/Leave",
        "default_body": {"goal_id": "cmd-undock"},
        "fields": [{"path": "goal_id", "label": "goal_id", "type": "text"}],
    },
    {
        "id": "rpc.follow",
        "label": "跟随",
        "description": "跟随人员或目标",
        "group": "跟随",
        "service": "FollowService",
        "method": "Follow",
        "rpc": "automsgs.rpcs.follow.FollowService/Follow",
        "default_body": {
            "goal_id": "cmd-follow",
            "target_type": "FOLLOW_TARGET_TYPE_PERSON",
            "target_id": "",
            "options": {"desired_distance_m": 1.2},
        },
        "fields": [
            {"path": "goal_id", "label": "goal_id", "type": "text"},
            {
                "path": "target_type",
                "label": "目标类型",
                "type": "select",
                "options": [
                    "FOLLOW_TARGET_TYPE_PERSON",
                    "FOLLOW_TARGET_TYPE_OBJECT",
                    "FOLLOW_TARGET_TYPE_VEHICLE",
                ],
            },
            {"path": "target_id", "label": "target_id", "type": "text", "placeholder": "空=自选"},
            {"path": "options.desired_distance_m", "label": "跟随距离 (m)", "type": "number", "step": 0.1},
        ],
    },
    {
        "id": "rpc.teleop_velocity",
        "label": "遥操速度",
        "description": "下发线速度 / 角速度",
        "group": "遥操",
        "service": "TeleopService",
        "method": "Velocity",
        "rpc": "automsgs.rpcs.teleop.TeleopService/Velocity",
        "default_body": {
            "command": "VELOCITY_COMMAND_TWIST",
            "goal_id": "cmd-teleop",
            "twist": {
                "header": {"frame_id": "base_link"},
                "twist": {
                    "linear": {"x": 0.2, "y": 0.0, "z": 0.0},
                    "angular": {"x": 0.0, "y": 0.0, "z": 0.0},
                },
            },
        },
        "fields": [
            {"path": "goal_id", "label": "goal_id", "type": "text"},
            {"path": "twist.twist.linear.x", "label": "线速度 vx", "type": "number", "step": 0.05},
            {"path": "twist.twist.angular.z", "label": "角速度 wz", "type": "number", "step": 0.05},
        ],
    },
    {
        "id": "rpc.explore",
        "label": "自主探索",
        "description": "探索未知区域并建图",
        "group": "建图",
        "service": "ExplorationService",
        "method": "Explore",
        "rpc": "automsgs.rpcs.exploration.ExplorationService/Explore",
        "default_body": {"goal_id": "cmd-explore", "map_name": "explore", "options": {}},
        "fields": [
            {"path": "goal_id", "label": "goal_id", "type": "text"},
            {"path": "map_name", "label": "地图名", "type": "text"},
        ],
    },
    {
        "id": "rpc.start_mapping",
        "label": "开始建图",
        "description": "启动建图会话",
        "group": "建图",
        "service": "MapService",
        "method": "StartMapping",
        "rpc": "automsgs.rpcs.mapping.MapService/StartMapping",
        "default_body": {"map_name": "floor1", "goal_id": "cmd-map"},
        "fields": [
            {"path": "goal_id", "label": "goal_id", "type": "text"},
            {"path": "map_name", "label": "地图名", "type": "text"},
        ],
    },
    {
        "id": "rpc.relocalize",
        "label": "设置初值",
        "description": "重定位：给定地图系初始位姿",
        "group": "定位",
        "service": "LocalizationService",
        "method": "SetInitialPose",
        "rpc": "automsgs.rpcs.localization.LocalizationService/SetInitialPose",
        "default_body": {
            "pose": {
                "pose": {
                    "header": {"frame_id": "map"},
                    "pose": {
                        "position": {"x": 0.0, "y": 0.0, "z": 0.0},
                        "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
                    },
                }
            },
            "map_id": "",
        },
        "fields": [
            {"path": "pose.pose.pose.position.x", "label": "x (m)", "type": "number", "step": 0.1},
            {"path": "pose.pose.pose.position.y", "label": "y (m)", "type": "number", "step": 0.1},
            {"path": "pose.pose.header.frame_id", "label": "frame", "type": "text"},
            {"path": "map_id", "label": "map_id", "type": "text", "placeholder": "空=当前图"},
        ],
    },
    {
        "id": "rpc.cancel_all",
        "label": "取消全部",
        "description": "取消当前活动目标",
        "group": "系统",
        "service": "SystemService",
        "method": "CancelAllGoals",
        "rpc": "automsgs.rpcs.system.SystemService/CancelAllGoals",
        "default_body": {"goal_kinds": [], "reason": "dashboard"},
        "fields": [{"path": "reason", "label": "原因", "type": "text"}],
    },
    {
        "id": "rpc.estop",
        "label": "急停",
        "description": "紧急停止（慎用）",
        "group": "系统",
        "danger": True,
        "service": "SystemService",
        "method": "EmergencyStop",
        "rpc": "automsgs.rpcs.system.SystemService/EmergencyStop",
        "default_body": {"reason": "dashboard"},
        "fields": [{"path": "reason", "label": "原因", "type": "text"}],
    },
]


def _mode_cfg_from_task(task_cfg: dict[str, Any]) -> dict[str, Any]:
    mode_cfg = dict(task_cfg.get("mode_config") or {})
    for key in (
        "obstacles",
        "pedestrians",
        "moving_obstacles",
        "difficulty",
        "waypoints",
        "seed",
        "start_x",
        "start_y",
        "goal_x",
        "goal_y",
        "coverage_target",
        "scenario",
        "scenario_file",
        "stages",
        "map",
        "start_zones",
        "goal_zones",
        "explore_radius",
        "cell_size",
        "dock_x",
        "dock_y",
        "dock_yaw",
    ):
        if key in task_cfg and key not in mode_cfg:
            mode_cfg[key] = task_cfg[key]
    return mode_cfg


class _MissionBase(Task):
    """Shared episode runner for operational missions."""

    default_mode: str = "scenario"
    default_tolerance: float = 0.35

    def _run_standard(self, *, mode_name: str | None = None) -> list[EpisodeResult]:
        sim = self.simulator or create_simulator(self.config)
        episodes = int(self.task_cfg.get("episodes", 1))
        timeout_s = float(self.task_cfg.get("timeout_s", 60))
        tol = float(self.task_cfg.get("goal_tolerance_m", self.default_tolerance))
        mode_name = mode_name or str(self.task_cfg.get("mode", self.default_mode))
        mode = create_mode(mode_name, _mode_cfg_from_task(self.task_cfg))
        planner = make_planner(self.config.section("planner"))
        results: list[EpisodeResult] = []
        with sim:
            for ep in range(episodes):
                trace = run_episode(
                    sim,
                    mode,
                    planner,
                    episode_id=ep,
                    timeout_s=timeout_s,
                    goal_tolerance=tol,
                )
                result = trace_to_result(trace)
                result.info["mission"] = self.name
                _LOG.info(
                    "%s ep=%s → %s steps=%s v_max=%.2f jerk_max=%.2f",
                    self.name,
                    ep,
                    result.status.value,
                    result.steps,
                    result.metrics.get("v_max", 0.0),
                    result.metrics.get("jerk_max", 0.0),
                )
                results.append(result)
        return results


class NavigateMissionTask(_MissionBase):
    name = "navigate"
    default_mode = "scenario"

    def run(self) -> list[EpisodeResult]:
        return self._run_standard()


class DockTask(_MissionBase):
    """Drive to a charging dock pose with tight lateral / yaw tolerance."""

    name = "dock"
    default_mode = "scenario"
    default_tolerance = 0.12

    def run(self) -> list[EpisodeResult]:
        # Inject dock pose as goal for scenario mode.
        self.task_cfg.setdefault("goal_x", float(self.task_cfg.get("dock_x", 3.5)))
        self.task_cfg.setdefault("goal_y", float(self.task_cfg.get("dock_y", 0.0)))
        self.task_cfg.setdefault("obstacles", int(self.task_cfg.get("obstacles", 4)))
        self.task_cfg.setdefault("pedestrians", 0)
        return self._run_standard(mode_name="scenario")


class ExploreMappingTask(_MissionBase):
    name = "explore_mapping"
    default_mode = "explore"

    def run(self) -> list[EpisodeResult]:
        self.task_cfg.setdefault("coverage_target", 0.55)
        self.task_cfg.setdefault("timeout_s", 45)
        return self._run_standard(mode_name="explore")


class PatrolTask(_MissionBase):
    """Visit a closed waypoint loop (inspection)."""

    name = "patrol"
    default_mode = "guided"
    default_tolerance = 0.4

    def run(self) -> list[EpisodeResult]:
        if "waypoints" not in self.task_cfg:
            self.task_cfg["waypoints"] = [
                {"x": 2.0, "y": 0.0},
                {"x": 2.0, "y": 2.0},
                {"x": 0.0, "y": 2.0},
                {"x": 0.0, "y": 0.0},
            ]
        mode_cfg = _mode_cfg_from_task(self.task_cfg)
        mode_cfg["waypoints"] = list(self.task_cfg["waypoints"])
        self.task_cfg["mode_config"] = mode_cfg
        return self._run_standard(mode_name=str(self.task_cfg.get("mode", "guided")))


class TeleopTask(Task):
    """Scripted open-loop velocity profile (teleop replay style)."""

    name = "teleop"

    def run(self) -> list[EpisodeResult]:
        sim = self.simulator or create_simulator(self.config)
        episodes = int(self.task_cfg.get("episodes", 1))
        timeout_s = float(self.task_cfg.get("timeout_s", 25))
        tol = float(self.task_cfg.get("goal_tolerance_m", 0.45))
        profile = str(self.task_cfg.get("teleop_profile", "smooth"))
        results: list[EpisodeResult] = []
        mode = create_mode("scenario", _mode_cfg_from_task(self.task_cfg))

        with sim:
            for ep in range(episodes):
                setup = mode.setup(ep, sim)
                pose = _apply_setup(sim, setup, seed=ep)
                trace = EpisodeTrace(episode_id=ep, setup=setup)
                import time as _time

                wall0 = _time.monotonic()
                last_t = 0.0
                trace.points.append(
                    TrajectoryPoint(t=0.0, pose=pose, twist=Twist2D(), clearance=float("inf"))
                )
                while _time.monotonic() - wall0 < timeout_s:
                    now = _time.monotonic() - wall0
                    dt = max(1e-3, now - last_t)
                    last_t = now
                    cmd = _teleop_command(now, setup.goal, pose, profile=profile, dt=dt)
                    step = sim.step(cmd)
                    pose = step.pose
                    clear = float(step.info.get("min_clearance", float("inf")))
                    if step.info.get("collision"):
                        trace.collisions += 1
                        trace.status = EpisodeStatus.COLLISION
                        trace.points.append(
                            TrajectoryPoint(t=now, pose=pose, twist=cmd, clearance=clear)
                        )
                        break
                    trace.points.append(
                        TrajectoryPoint(t=now, pose=pose, twist=cmd, clearance=clear)
                    )
                    if reached(pose, setup.goal, tol):
                        trace.status = EpisodeStatus.GOAL_REACHED
                        break
                else:
                    trace.status = EpisodeStatus.TIMEOUT
                result = trace_to_result(trace)
                result.info["mission"] = self.name
                result.info["teleop_profile"] = profile
                results.append(result)
                _LOG.info("teleop ep=%s → %s", ep, result.status.value)
        return results


def _teleop_command(
    t: float,
    goal: Pose2D,
    pose: Pose2D,
    *,
    profile: str,
    dt: float,
) -> Twist2D:
    """Human-like teleop: proportional heading + profiled linear speed."""
    dx = goal.x - pose.x
    dy = goal.y - pose.y
    dist = math.hypot(dx, dy)
    heading = math.atan2(dy, dx)
    yaw_err = (heading - pose.yaw + math.pi) % (2 * math.pi) - math.pi
    if profile == "aggressive":
        v_cap, ramp = 0.9, 0.6
    elif profile == "cautious":
        v_cap, ramp = 0.35, 1.5
    else:
        v_cap, ramp = 0.55, 1.0
    # Trapezoidal-ish ramp with mild noise.
    ramp_v = min(v_cap, (t / max(ramp, 1e-3)) * v_cap)
    if dist < 0.8:
        ramp_v *= dist / 0.8
    noise = 0.02 * math.sin(8.0 * t)
    v = max(0.0, ramp_v + noise)
    w = float(np.clip(1.8 * yaw_err, -1.2, 1.2))
    if abs(yaw_err) > 0.7:
        v *= 0.35
    del dt
    return Twist2D(linear=v, angular=w)


__all__ = [
    "MISSION_CATALOG",
    "DockTask",
    "ExploreMappingTask",
    "NavigateMissionTask",
    "PatrolTask",
    "TeleopTask",
]
