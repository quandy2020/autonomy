from __future__ import annotations

import warnings
from typing import Protocol, Tuple

import numpy as np

from autosim.agent import AgentState


class World(Protocol):
    def reset(self, x: float, y: float, yaw: float) -> None: ...
    def set_pose(self, x: float, y: float, yaw: float) -> None: ...
    def step(self) -> None: ...
    def pose(self) -> Tuple[float, float, float]: ...
    def depth_ring(self, num_beams: int, range_max: float) -> np.ndarray: ...
    def rgb_depth(self) -> Tuple[np.ndarray, np.ndarray]: ...
    def close(self) -> None: ...


class MockWorld:
    """Deterministic stand-in for CI without habitat-sim."""

    def __init__(self, width: int = 64, height: int = 48) -> None:
        self.width = int(width)
        self.height = int(height)
        self._agent = AgentState()

    def reset(self, x: float, y: float, yaw: float) -> None:
        self._agent = AgentState(x=x, y=y, yaw=yaw)

    def set_pose(self, x: float, y: float, yaw: float) -> None:
        self._agent = AgentState(x=x, y=y, yaw=yaw)

    def step(self) -> None:
        return None

    def pose(self) -> Tuple[float, float, float]:
        return self._agent.as_tuple()

    def depth_ring(self, num_beams: int, range_max: float) -> np.ndarray:
        # Constant mid-range scan for pipeline tests.
        return np.full((num_beams,), 0.5 * float(range_max), dtype=np.float32)

    def rgb_depth(self) -> Tuple[np.ndarray, np.ndarray]:
        rgb = np.zeros((self.height, self.width, 3), dtype=np.uint8)
        rgb[..., 1] = 128
        depth = np.full((self.height, self.width), 2.0, dtype=np.float32)
        return rgb, depth

    def close(self) -> None:
        return None


class HabitatWorld:
    """Habitat-Sim backed world. Requires optional dependency habitat-sim."""

    def __init__(self, cfg: dict) -> None:
        try:
            import habitat_sim
        except ImportError as exc:
            raise ImportError(
                "habitat-sim is not installed. Install with: pip install habitat-sim"
            ) from exc

        self._habitat_sim = habitat_sim
        self._cfg = cfg
        self._agent = AgentState()
        backend = habitat_sim.SimulatorConfiguration()
        backend.gpu_device_id = int(cfg["habitat"]["gpu"])
        scene_path = cfg["scene"].get("path") or ""
        if cfg["scene"]["backend"] == "minimal" and not scene_path:
            # Empty stage: pipeline bring-up without external datasets.
            backend.scene_id = habitat_sim.utils.settings.default_sim_settings.get(
                "scene", "NONE"
            )
            if hasattr(habitat_sim, "STAGE_EMPTY_SCENE"):
                backend.scene_id = habitat_sim.STAGE_EMPTY_SCENE
            else:
                backend.scene_id = "NONE"
        else:
            if not scene_path:
                raise FileNotFoundError(
                    "scene.path is required for backend=habitat; "
                    "set path to an HM3D/Replica/MP3D scene file"
                )
            path = scene_path
            from pathlib import Path

            if not Path(path).exists():
                raise FileNotFoundError(f"scene path does not exist: {path}")
            backend.scene_id = path

        agent_cfg = habitat_sim.agent.AgentConfiguration()
        rgb_sensor = habitat_sim.CameraSensorSpec()
        rgb_sensor.uuid = "rgb"
        rgb_sensor.sensor_type = habitat_sim.SensorType.COLOR
        rgb_sensor.resolution = [int(cfg["habitat"]["height"]), int(cfg["habitat"]["width"])]
        depth_sensor = habitat_sim.CameraSensorSpec()
        depth_sensor.uuid = "depth"
        depth_sensor.sensor_type = habitat_sim.SensorType.DEPTH
        depth_sensor.resolution = [int(cfg["habitat"]["height"]), int(cfg["habitat"]["width"])]
        agent_cfg.sensor_specifications = [rgb_sensor, depth_sensor]

        self._sim = habitat_sim.Simulator(habitat_sim.Configuration(backend, [agent_cfg]))
        spawn = cfg["scene"].get("spawn", [0.0, 0.0, 0.0])
        self.reset(float(spawn[0]), float(spawn[1]), float(spawn[2]))

    def reset(self, x: float, y: float, yaw: float) -> None:
        self.set_pose(x, y, yaw)

    def set_pose(self, x: float, y: float, yaw: float) -> None:
        import magnum as mn

        self._agent = AgentState(x=x, y=y, yaw=yaw)
        agent = self._sim.get_agent(0)
        state = agent.get_state()
        state.position = mn.Vector3(float(x), 0.0, float(y))  # habitat Y-up: map y→z
        state.rotation = mn.Quaternion.rotation(mn.Rad(float(yaw)), mn.Vector3(0, 1, 0))
        agent.set_state(state, infer_sensor_states=True)

    def step(self) -> None:
        # Pose is set explicitly from Drive each cycle; no discrete action needed.
        return None

    def pose(self) -> Tuple[float, float, float]:
        return self._agent.as_tuple()

    def depth_ring(self, num_beams: int, range_max: float) -> np.ndarray:
        obs = self._sim.get_sensor_observations()
        depth = np.asarray(obs["depth"], dtype=np.float32)
        # Approximate 2D lidar from center row of depth image.
        row = depth[depth.shape[0] // 2, :]
        idx = np.linspace(0, row.shape[0] - 1, num=num_beams).astype(np.int32)
        ranges = row[idx]
        ranges[~np.isfinite(ranges)] = float(range_max)
        return ranges.astype(np.float32)

    def rgb_depth(self) -> Tuple[np.ndarray, np.ndarray]:
        obs = self._sim.get_sensor_observations()
        rgb = np.asarray(obs["rgb"])[..., :3].astype(np.uint8)
        depth = np.asarray(obs["depth"], dtype=np.float32)
        return rgb, depth

    def close(self) -> None:
        self._sim.close()


def create_world(cfg: dict) -> World:
    backend = cfg["scene"]["backend"]
    w = int(cfg["habitat"]["width"])
    h = int(cfg["habitat"]["height"])
    if backend == "minimal":
        # Prefer Habitat empty/minimal stage when available; else MockWorld.
        try:
            return HabitatWorld(cfg)
        except Exception as exc:
            warnings.warn(
                f"backend=minimal: HabitatWorld unavailable ({exc!r}), falling back to MockWorld",
                stacklevel=2,
            )
            return MockWorld(width=w, height=h)
    if backend == "habitat":
        return HabitatWorld(cfg)
    raise ValueError(f"unknown backend: {backend}")
