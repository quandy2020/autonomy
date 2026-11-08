"""The navigation environment: navmesh access, the simulator, kinematics, and closed-loop scoring.

Kinematics and scoring use a navmesh `PathFinder`: standalone (`Stepper.use_pathfinder`, no simulator,
no GPU) or the simulator's (`Stepper.use_scene`), whose habitat agent can also `observe`. Habitat's
sliding `move_forward` is `pathfinder.try_step`: both move the agent identically.

The navmesh serves motion, scoring and the expert; a policy receives only its pose, the goal position,
and the stepper it observes through.
"""
from __future__ import annotations

import math
import os
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Optional, Sequence, Tuple

import numpy as np
import quaternion as npq

from thirdparty.pan.util.constants import (AGENT_HEIGHT, AGENT_RADIUS, FORWARD_STEP_M, HABITAT_DATA_ROOT,
                                   HM3D_SCENE_DATASET_CONFIG, NAVMESH_SUFFIX, SENSOR_HEIGHT,
                                   SENSOR_HFOV_DEG, SENSOR_RES, TURN_STEP_DEG, Action)
from thirdparty.pan.util.geometry import raster_cell, wrap_angle


# ---------------------------------------------------------------- navmesh access
def resolve_scene_path(path) -> str:
    """A corpus's scene asset path, remapped under this repo's scene_datasets when it does not
    exist here."""
    path = str(path)
    if not os.path.exists(path) and "scene_datasets/" in path:
        local = str(HABITAT_DATA_ROOT / "raw/scene_datasets" / path.split("scene_datasets/", 1)[1])
        if os.path.exists(local):
            return local
    return path


def navmesh_path(scene_glb) -> str:
    """The navmesh baked for the embodiment, next to a scene."""
    return str(Path(str(scene_glb)).with_suffix(NAVMESH_SUFFIX))


def load_pathfinder(scene_glb):
    """Standalone navmesh `PathFinder` for a scene (`bake_navmesh` writes the embodiment's)."""
    import habitat_sim

    nav = resolve_scene_path(navmesh_path(scene_glb))
    if not os.path.exists(nav):
        raise FileNotFoundError(f"no navmesh at {nav}; bake it with env.bake_navmesh")
    pf = habitat_sim.nav.PathFinder()
    pf.load_nav_mesh(nav)
    if not pf.is_loaded:
        raise RuntimeError(f"failed to load navmesh {nav}")
    return pf


def find_path(pf, start_point, goal_point) -> Tuple[Optional[np.ndarray], float]:
    """Geodesic shortest path -> (path points (M, 3) | None, geodesic distance)."""
    import habitat_sim

    sp = habitat_sim.ShortestPath()
    sp.requested_start = np.asarray(start_point, dtype=np.float32)
    sp.requested_end = np.asarray(goal_point, dtype=np.float32)
    if not pf.find_path(sp):
        return None, float("inf")
    return np.asarray(sp.points, dtype=np.float64).reshape(-1, 3), float(sp.geodesic_distance)


def snap_point(pf, point) -> np.ndarray:
    """Nearest navmesh point, (3,) float64; NaN when there is no navmesh nearby."""
    return np.asarray(pf.snap_point(np.asarray(point, dtype=np.float32)), np.float64).reshape(3)


# ---------------------------------------------------------------- simulator
SIM_ACTIONS = {Action.FORWARD: "move_forward", Action.TURN_LEFT: "turn_left",
               Action.TURN_RIGHT: "turn_right"}


def make_sim(scene_glb: str, scene_dataset_config: str, gpu_device_id: int = 0,
             sensors: Sequence[str] = ("rgb", "depth"), bake_missing_navmesh: bool = True):
    """Habitat-sim with the requested cameras at SENSOR_HEIGHT and the SIM_ACTIONS action space, on the
    embodiment's navmesh. `sensors=()` builds no renderer (kinematics and navmesh only, no GPU)."""
    import habitat_sim

    backend = habitat_sim.SimulatorConfiguration()
    backend.scene_id = scene_glb
    backend.scene_dataset_config_file = scene_dataset_config
    backend.enable_physics = False
    backend.gpu_device_id = gpu_device_id
    backend.create_renderer = bool(sensors)
    types = {"rgb": habitat_sim.SensorType.COLOR, "depth": habitat_sim.SensorType.DEPTH}
    assert set(sensors) <= set(types), f"unknown sensor(s) {sorted(set(sensors) - set(types))}"
    cameras = []
    for uuid in sensors:
        spec = habitat_sim.CameraSensorSpec()
        spec.uuid, spec.sensor_type = uuid, types[uuid]
        spec.resolution, spec.position = list(SENSOR_RES), [0.0, SENSOR_HEIGHT, 0.0]
        spec.hfov = SENSOR_HFOV_DEG
        cameras.append(spec)
    agent_cfg = habitat_sim.AgentConfiguration()
    agent_cfg.height, agent_cfg.radius = AGENT_HEIGHT, AGENT_RADIUS
    agent_cfg.sensor_specifications = cameras
    agent_cfg.action_space = {
        name: habitat_sim.ActionSpec(name, habitat_sim.ActuationSpec(
            amount=FORWARD_STEP_M if action == Action.FORWARD else TURN_STEP_DEG))
        for action, name in SIM_ACTIONS.items()}
    sim = habitat_sim.Simulator(habitat_sim.Configuration(backend, [agent_cfg]))
    bake_navmesh(sim, scene_glb, bake_missing_navmesh)
    return sim


def bake_navmesh(sim, scene_glb: str, bake_missing: bool = True) -> None:
    """Load the embodiment's navmesh (AGENT_RADIUS, AGENT_HEIGHT; habitat defaults otherwise) into
    `sim.pathfinder`. A missing one is baked and saved next to the .glb, where `load_pathfinder`
    reads it, or raises unless `bake_missing`."""
    import habitat_sim

    path, pf = navmesh_path(scene_glb), sim.pathfinder
    if os.path.exists(path):
        if not pf.load_nav_mesh(path):
            raise RuntimeError(f"failed to load cached navmesh {path}")
        return
    if not bake_missing:
        raise FileNotFoundError(f"no embodiment navmesh at {path}")
    settings = habitat_sim.NavMeshSettings()
    settings.set_defaults()
    settings.agent_radius, settings.agent_height = AGENT_RADIUS, AGENT_HEIGHT
    if not sim.recompute_navmesh(pf, settings):
        raise RuntimeError(f"navmesh recompute failed for {scene_glb}")
    if not pf.save_nav_mesh(path):
        raise RuntimeError(f"failed to write navmesh {path}")


def agent_matrix(state, dtype=np.float64) -> np.ndarray:
    """A habitat agent state -> its (4, 4) agent-to-world matrix."""
    m = np.eye(4, dtype=dtype)
    m[:3, :3] = npq.as_rotation_matrix(state.rotation)
    m[:3, 3] = np.asarray(state.position, dtype)
    return m


def camera_matrix(agent_to_world: np.ndarray) -> np.ndarray:
    """(..., 4, 4) agent-to-world -> camera-to-world of the cameras make_sim mounts (+y SENSOR_HEIGHT),
    in the input's dtype."""
    offset = np.eye(4, dtype=agent_to_world.dtype)
    offset[1, 3] = SENSOR_HEIGHT
    return agent_to_world @ offset


# ---------------------------------------------------------------- tours and episodes
@dataclass(frozen=True)
class EpisodeConfig:
    success_radius: float = 0.2         # PointNav success: STOP within this geodesic distance
    max_steps: int = 500                # closed loop: actions incl. STOP; expert.py: moves + STOP
    turn_deg: float = TURN_STEP_DEG
    expert_stop_radius: float = 0.15    # inside success_radius
    expert_lookahead_m: float = 0.1     # steer toward the first path vertex beyond this
    min_geodesic_m: float = 1.0         # episode start -> goal
    min_detour_ratio: float = 1.1       # soft filter on geodesic / euclidean (expert.py)
    floor_tolerance_m: float = 0.5      # |y - floor_y| of points on the tour's floor


@dataclass(eq=False)
class TourScene:
    """One tour's scene: the full navmesh, the island episode endpoints are drawn from, and the
    floor and observed cells `admits` checks."""

    pathfinder: object
    island: int
    floor_y: float
    coverage: np.ndarray                # (rows, cols) uint8 top-down; 2 = observed
    origin_position: Tuple[float, float]    # world (x, z) of the corner of cell (0, 0)
    mpp: float

    def on_floor(self, point: np.ndarray, cfg: EpisodeConfig) -> bool:
        """Finite and on the tour's floor."""
        return bool(np.isfinite(point).all() and abs(point[1] - self.floor_y) <= cfg.floor_tolerance_m)

    def admits(self, point: np.ndarray, cfg: EpisodeConfig) -> bool:
        """A valid episode endpoint: on the tour's floor, inside its observed coverage."""
        if not self.on_floor(point, cfg):
            return False
        r, c = raster_cell(point[[0, 2]], self.origin_position, self.mpp)
        return bool(0 <= r < self.coverage.shape[0] and 0 <= c < self.coverage.shape[1]
                    and self.coverage[r, c] == 2)


def tour_island(pf, start_point: np.ndarray) -> int:
    """The navmesh island a tour ran on: that of its snapped start."""
    return int(pf.get_island(snap_point(pf, start_point).astype(np.float32)))


@dataclass(frozen=True, eq=False)
class Episode:
    start_point: np.ndarray             # (3,) navmesh point
    start_yaw: float
    goal_point: np.ndarray              # (3,) navmesh point
    geodesic: float                     # metres, start -> goal
    poses: Optional[np.ndarray] = None  # (S, 3) float32 expert states; state s -> action s
    actions: Optional[np.ndarray] = None    # (S,) uint8 expert actions; the last is STOP

    @property
    def goal_position(self) -> np.ndarray:
        return self.goal_point[[0, 2]]


def detour_ratio(start_point: np.ndarray, goal_point: np.ndarray, geodesic: float) -> float:
    """Geodesic over euclidean start -> goal distance."""
    return float(geodesic) / max(float(np.linalg.norm(start_point - goal_point)), 1e-6)


# ---------------------------------------------------------------- kinematics
def navmesh_step(pf, point: np.ndarray, yaw: float, action: int, cfg: EpisodeConfig):
    """(point, yaw) after `action`: FORWARD slides along the navmesh (`try_step`), turns are exact."""
    if action == Action.FORWARD:
        forward = np.array([-math.sin(yaw), 0.0, -math.cos(yaw)])
        point = np.asarray(pf.try_step(point.astype(np.float32),
                                       (point + forward * FORWARD_STEP_M).astype(np.float32)),
                           np.float64)
    elif action == Action.TURN_LEFT:
        yaw = wrap_angle(yaw + math.radians(cfg.turn_deg))
    elif action == Action.TURN_RIGHT:
        yaw = wrap_angle(yaw - math.radians(cfg.turn_deg))
    return point, yaw


class Stepper:
    """The agent's discrete motion. On a bare navmesh (`use_pathfinder`) FORWARD slides with
    `try_step`; on a scene (`use_scene`, its simulator kept until the scene changes) the
    habitat agent moves identically and can also `observe`."""

    def __init__(self, cfg: EpisodeConfig, gpu_device_id: int = 0, sensors: Sequence[str] = ()):
        self.cfg, self.gpu_device_id, self.sensors = cfg, gpu_device_id, tuple(sensors)
        self.pathfinder = self.sim = self.scene_glb = None
        self.point, self.yaw = np.zeros(3), 0.0
        self.n_loads, self.load_seconds = 0, 0.0

    def use_pathfinder(self, pathfinder) -> "Stepper":
        self.close()
        self.pathfinder = pathfinder
        return self

    def use_scene(self, scene_glb: str) -> "Stepper":
        if scene_glb != self.scene_glb:
            assert self.cfg.turn_deg == TURN_STEP_DEG, "the simulator turns by TURN_STEP_DEG"
            self.close()
            t0 = time.perf_counter()
            self.sim = make_sim(resolve_scene_path(scene_glb), str(HM3D_SCENE_DATASET_CONFIG),
                                self.gpu_device_id, self.sensors, bake_missing_navmesh=False)
            self.load_seconds += time.perf_counter() - t0
            self.n_loads += 1
            self.scene_glb, self.pathfinder = scene_glb, self.sim.pathfinder
        return self

    def reset(self, point: np.ndarray, yaw: float) -> None:
        if self.sim is None:
            self.point, self.yaw = np.asarray(point, np.float64).copy(), float(yaw)
            return
        import habitat_sim

        state = habitat_sim.AgentState()
        state.position = np.asarray(point, np.float32)
        state.rotation = npq.quaternion(math.cos(yaw / 2.0), 0.0, math.sin(yaw / 2.0), 0.0)
        self.sim.get_agent(0).set_state(state)
        self._read()

    def step(self, action: int) -> None:
        if self.sim is None:
            self.point, self.yaw = navmesh_step(self.pathfinder, self.point, self.yaw, action,
                                                self.cfg)
        else:
            self.sim.get_agent(0).act(SIM_ACTIONS[action])
            self._read()

    def _read(self) -> None:
        state = self.sim.get_agent(0).get_state()
        self.point = np.asarray(state.position, np.float64)
        self.yaw = wrap_angle(2.0 * math.atan2(float(state.rotation.y), float(state.rotation.w)))

    def observe(self):
        """-> (depth (H, W) float32, camera-to-world (4, 4) float64), the camera as collection
        stores `camera_pose_world`."""
        depth = np.asarray(self.sim.get_sensor_observations()["depth"], np.float32)
        return depth, camera_matrix(agent_matrix(self.sim.get_agent(0).get_state()))

    def close(self) -> None:
        if self.sim is not None:
            self.sim.close()
        self.sim = self.scene_glb = self.pathfinder = None


# ---------------------------------------------------------------- rollout
Policy = Callable[[np.ndarray, np.ndarray, Stepper], int]


def policy_rollout(stepper: Stepper, policy: Policy, episode: Episode, cfg: EpisodeConfig,
                   record_trajectory: bool = False) -> dict:
    """Closed-loop execution of `policy(pose, goal_position, stepper) -> action` from the
    episode's start -> success (STOP within `success_radius` geodesic of the goal), spl,
    steps, stopped, geo0 / final_geo (geodesic to the goal at the start and the end; geo0 when
    the end has no path), traj
    (visited [x, z], when recorded)."""
    pf, goal_position = stepper.pathfinder, episode.goal_position
    stepper.reset(episode.start_point, float(episode.start_yaw))
    traveled, steps, success, stopped = 0.0, 0, False, False
    trajectory = [[float(stepper.point[0]), float(stepper.point[2])]] if record_trajectory else None
    geodesic = episode.geodesic
    while steps < cfg.max_steps:
        point, yaw = stepper.point, stepper.yaw
        action = int(policy(np.array([point[0], point[2], yaw], np.float64), goal_position, stepper))
        if action == Action.STOP:
            stopped, success = True, geodesic <= cfg.success_radius
            break
        stepper.step(action)
        traveled += float(np.linalg.norm(stepper.point - point))
        if trajectory is not None:
            trajectory.append([float(stepper.point[0]), float(stepper.point[2])])
        _, geodesic = find_path(pf, stepper.point, episode.goal_point)
        if not np.isfinite(geodesic):
            geodesic = episode.geodesic
        steps += 1
    spl = episode.geodesic / max(traveled, episode.geodesic, 1e-6) if success else 0.0
    return dict(success=int(success), spl=float(spl), steps=steps, stopped=bool(stopped),
                geo0=float(episode.geodesic), final_geo=float(geodesic), traj=trajectory)
