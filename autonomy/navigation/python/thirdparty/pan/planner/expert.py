"""The expert the planner imitates: a greedy shortest-path follower on the full navmesh.

It heads for the first path vertex beyond `expert_lookahead_m`, turns when the heading error exceeds
half a turn step, and STOPs inside `expert_stop_radius`.

Episodes: start and goal are navigable points on the tour's floor, island and observed coverage, at
least `min_geodesic_m` apart, under Habitat PointNav's soft detour-ratio filter; episodes the expert
cannot finish within `max_steps` moves + STOP are dropped.
"""
from __future__ import annotations

import math
from typing import Optional

import numpy as np

from thirdparty.pan.util.constants import Action
from thirdparty.pan.util.geometry import heading_of, wrap_angle

from .env import Episode, EpisodeConfig, TourScene, detour_ratio, find_path, navmesh_step


def expert_action(pf, point: np.ndarray, yaw: float, goal_point: np.ndarray,
                  cfg: EpisodeConfig) -> Optional[int]:
    """The expert's action at (point, yaw); None when there is no route to the goal."""
    path, geodesic = find_path(pf, point, goal_point)
    if path is None:
        return None
    if geodesic <= cfg.expert_stop_radius:
        return Action.STOP
    distance = np.linalg.norm(path[:, [0, 2]] - point[[0, 2]], axis=1)
    beyond = np.nonzero(distance >= cfg.expert_lookahead_m)[0]
    if len(beyond) == 0:                         # standing on the last vertex
        return Action.TURN_LEFT
    dx, dz = path[beyond[0], [0, 2]] - point[[0, 2]]
    error = wrap_angle(heading_of(dx, dz) - yaw)
    if abs(error) > math.radians(cfg.turn_deg) / 2.0:
        return Action.TURN_LEFT if error > 0 else Action.TURN_RIGHT
    return Action.FORWARD


def expert_steps(pf, point: np.ndarray, yaw: float, goal_point: np.ndarray, cfg: EpisodeConfig,
                 n: int):
    """Up to `n` expert steps from (point, yaw), ending at the first STOP -> (poses
    [[x, z, yaw], ...], actions); None when the expert loses its route."""
    point, yaw = np.asarray(point, np.float64), float(yaw)
    poses, actions = [], []
    for _ in range(n):
        action = expert_action(pf, point, yaw, goal_point, cfg)
        if action is None:
            return None
        poses.append([float(point[0]), float(point[2]), yaw])
        actions.append(action)
        if action == Action.STOP:
            break
        point, yaw = navmesh_step(pf, point, yaw, action, cfg)
    return poses, actions


def expert_rollout(pf, start_point: np.ndarray, start_yaw: float, goal_point: np.ndarray,
                   cfg: EpisodeConfig) -> Optional[Episode]:
    """The expert's episode from start to goal; None unless it STOPs after at most `max_steps` moves."""
    start_point, goal_point = np.asarray(start_point, np.float64), np.asarray(goal_point, np.float64)
    _, geodesic = find_path(pf, start_point, goal_point)
    steps = (expert_steps(pf, start_point, start_yaw, goal_point, cfg, cfg.max_steps + 1)
             if np.isfinite(geodesic) else None)
    if steps is None or steps[1][-1] != Action.STOP:
        return None
    return Episode(start_point, float(start_yaw), goal_point, float(geodesic),
                   np.asarray(steps[0], np.float32), np.asarray(steps[1], np.uint8))


# ---------------------------------------------------------------- episode sampling
def _random_point(scene: TourScene) -> np.ndarray:
    return np.asarray(scene.pathfinder.get_random_navigable_point(island_index=scene.island),
                      np.float64)


def _detour_accepted(start_point, goal_point, geodesic: float, rng: np.random.Generator,
                     cfg: EpisodeConfig) -> bool:
    """Minimum geodesic, then Habitat PointNav's soft filter: below `min_detour_ratio`
    (geodesic / euclidean) an episode is kept with probability 20 (ratio - 0.98)^2."""
    if not (np.isfinite(geodesic) and geodesic >= cfg.min_geodesic_m):
        return False
    ratio = detour_ratio(start_point, goal_point, geodesic)
    return ratio >= cfg.min_detour_ratio or float(rng.random()) <= 20.0 * (ratio - 0.98) ** 2


def sample_episode(scene: TourScene, rng: np.random.Generator, cfg: EpisodeConfig,
                   goal_point: Optional[np.ndarray] = None, max_tries: int = 60) -> Optional[Episode]:
    """One expert episode from a random admissible start to `goal_point`, or to a random
    admissible goal."""
    for _ in range(max_tries):
        start = _random_point(scene)
        goal = _random_point(scene) if goal_point is None else goal_point
        if not (scene.admits(start, cfg) and scene.admits(goal, cfg)):
            continue
        _, geodesic = find_path(scene.pathfinder, start, goal)
        if not _detour_accepted(start, goal, geodesic, rng, cfg):
            continue
        episode = expert_rollout(scene.pathfinder, start, float(rng.uniform(-math.pi, math.pi)),
                                 goal, cfg)
        if episode is not None:
            return episode
    return None


def sample_goal(scene: TourScene, cfg: EpisodeConfig, max_tries: int = 40) -> Optional[np.ndarray]:
    """A random admissible navmesh point, or None."""
    for _ in range(max_tries):
        point = _random_point(scene)
        if scene.admits(point, cfg):
            return point
    return None


# ---------------------------------------------------------------- chunk targets
def stop_padded(actions, chunk: int) -> np.ndarray:
    """Up to `chunk` actions -> (chunk,) int64, STOP past the end."""
    out = np.full(chunk, Action.STOP, np.int64)
    out[:len(actions)] = actions
    return out


def chunk_targets_from_trajectory(actions: np.ndarray, t: int, chunk: int) -> np.ndarray:
    """Expert actions t .. t+chunk-1, STOP past the end."""
    return stop_padded(actions[t:t + chunk], chunk)


def chunk_targets_from_state(pf, point: np.ndarray, yaw: float, goal_point: np.ndarray,
                             cfg: EpisodeConfig, chunk: int) -> Optional[np.ndarray]:
    """`chunk` expert actions rolled from an arbitrary state, STOP past a STOP; None when
    the expert has no route."""
    steps = expert_steps(pf, point, yaw, goal_point, cfg, chunk)
    return None if steps is None else stop_padded(steps[1], chunk)
