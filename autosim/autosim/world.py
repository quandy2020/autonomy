from __future__ import annotations

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
    def __init__(self, cfg: dict) -> None:
        raise ImportError(
            "habitat-sim is required for HabitatWorld; pip install habitat-sim "
            f"or set scene path. requested path={cfg.get('scene', {}).get('path')!r}"
        )


def create_world(cfg: dict) -> World:
    backend = cfg["scene"]["backend"]
    w = int(cfg["habitat"]["width"])
    h = int(cfg["habitat"]["height"])
    if backend == "minimal":
        # Prefer Habitat empty/minimal stage when available; else MockWorld.
        try:
            return HabitatWorld(cfg)
        except Exception:
            return MockWorld(width=w, height=h)
    if backend == "habitat":
        return HabitatWorld(cfg)
    raise ValueError(f"unknown backend: {backend}")
