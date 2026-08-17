"""Habitat-Sim backend or deterministic mock simulation."""

from __future__ import annotations

import math
import warnings
from pathlib import Path
from typing import Any, Mapping, Tuple

import numpy as np


class Simulator:
    """Owns scene and agent pose; supports a real Habitat session or mock observations."""

    def __init__(
        self,
        backend: str,
        width: int,
        height: int,
        settings: Mapping[str, Any] | None = None,
        use_mock: bool = False,
    ) -> None:
        """Construct the simulator.

        Args:
            backend: ``minimal`` (empty path) or ``habitat`` (external scene path).
            width: Camera horizontal resolution in pixels.
            height: Camera vertical resolution in pixels.
            settings: Full config mapping (must include ``habitat``); may be thin for mocks.
            use_mock: If ``True``, skip Habitat and use deterministic fake data.

        Raises:
            ImportError: Non-mock mode without ``habitat-sim`` installed.
            FileNotFoundError: Invalid Habitat scene path.
        """
        self.backend = backend
        self.width = int(width)
        self.height = int(height)
        self.settings = dict(settings or {})
        self.use_mock = bool(use_mock)
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.session = None

        if not self.use_mock:
            self.open_habitat()

    @classmethod
    def create(cls, settings: Mapping[str, Any]) -> "Simulator":
        """Create a simulator from config; fall back to mock when ``path`` is empty and Habitat fails.

        Args:
            settings: Validated config dict (e.g. ``Config.data``).

        Returns:
            A :class:`Simulator` instance.

        Raises:
            ImportError / FileNotFoundError: Non-empty ``path`` and initialization fails (no fallback).
        """
        habitat = settings["habitat"]
        scene_path = str(habitat.get("path") or "").strip()
        backend = "minimal" if not scene_path else "habitat"
        camera = habitat["sensors"]["camera"]
        width = int(camera["width"])
        height = int(camera["height"])
        if backend == "minimal":
            try:
                return cls(backend=backend, width=width, height=height, settings=settings)
            except Exception as exc:
                warnings.warn(
                    f"habitat.path empty: Habitat unavailable ({exc!r}), using mock mode",
                    stacklevel=2,
                )
                return cls(
                    backend=backend,
                    width=width,
                    height=height,
                    settings=settings,
                    use_mock=True,
                )
        return cls(backend=backend, width=width, height=height, settings=settings)

    def open_habitat(self) -> None:
        """Create a Habitat ``Simulator`` session and apply the initial spawn.

        Raises:
            ImportError: ``habitat-sim`` is not installed.
            FileNotFoundError: An external scene path is required but missing or absent on disk.
        """
        try:
            import habitat_sim
        except ImportError as exc:
            raise ImportError(
                "habitat-sim is not installed. Install with: pip install habitat-sim"
            ) from exc

        configuration = habitat_sim.SimulatorConfiguration()
        configuration.gpu_device_id = int(self.settings["habitat"]["gpu"])
        scene_path = str(self.settings["habitat"].get("path") or "").strip()
        if not scene_path:
            if hasattr(habitat_sim, "STAGE_EMPTY_SCENE"):
                configuration.scene_id = habitat_sim.STAGE_EMPTY_SCENE
            else:
                configuration.scene_id = habitat_sim.utils.settings.default_sim_settings.get(
                    "scene", "NONE"
                )
        else:
            if not Path(scene_path).exists():
                raise FileNotFoundError(f"scene path does not exist: {scene_path}")
            configuration.scene_id = scene_path

        agent_configuration = habitat_sim.agent.AgentConfiguration()
        color_sensor = habitat_sim.CameraSensorSpec()
        color_sensor.uuid = "rgb"
        color_sensor.sensor_type = habitat_sim.SensorType.COLOR
        color_sensor.resolution = [self.height, self.width]
        depth_sensor = habitat_sim.CameraSensorSpec()
        depth_sensor.uuid = "depth"
        depth_sensor.sensor_type = habitat_sim.SensorType.DEPTH
        depth_sensor.resolution = [self.height, self.width]
        agent_configuration.sensor_specifications = [color_sensor, depth_sensor]

        self.session = habitat_sim.Simulator(
            habitat_sim.Configuration(configuration, [agent_configuration])
        )
        spawn = self.settings["habitat"].get("spawn", [0.0, 0.0, 0.0])
        self.reset(float(spawn[0]), float(spawn[1]), float(spawn[2]))

    def reset(self, x: float, y: float, yaw: float) -> None:
        """Reset the agent pose.

        Args:
            x: Planar x (m), mapped to Habitat world x.
            y: Planar y (m), mapped to Habitat world z (Y-up).
            yaw: Heading about the vertical axis (rad).
        """
        self.set_pose(x, y, yaw)

    def set_pose(self, x: float, y: float, yaw: float) -> None:
        """Write planar pose; sync to the Habitat agent when not mocking.

        Args:
            x: Planar x (m).
            y: Planar y (m).
            yaw: Heading (rad).
        """
        self.x = float(x)
        self.y = float(y)
        self.yaw = float(yaw)
        if self.use_mock or self.session is None:
            return
        import magnum as mn

        agent = self.session.get_agent(0)
        state = agent.get_state()
        state.position = mn.Vector3(self.x, 0.0, self.y)
        state.rotation = mn.Quaternion.rotation(mn.Rad(self.yaw), mn.Vector3(0, 1, 0))
        agent.set_state(state, infer_sensor_states=True)

    def step(self) -> None:
        """Advance one simulation step.

        Motion is driven by explicit ``set_pose`` from outside; this method is a
        placeholder for future discrete actions.
        """
        return None

    def pose(self) -> Tuple[float, float, float]:
        """Return the current planar pose.

        Returns:
            ``(x, y, yaw)``.
        """
        return self.x, self.y, self.yaw

    def _angles(self, angle_min: float, angle_max: float, count: int) -> np.ndarray:
        if count <= 1:
            return np.array([float(angle_min)], dtype=np.float64)
        return np.linspace(float(angle_min), float(angle_max), num=int(count), dtype=np.float64)

    def _cast_range(self, yaw_offset: float, pitch: float, range_max: float) -> float | None:
        """Cast one ray; return distance or ``None`` on miss (Habitat only)."""
        import habitat_sim
        import magnum as mn

        origin = mn.Vector3(self.x, 0.0, self.y)
        heading = self.yaw + yaw_offset
        # Bridge: x forward, y left; Habitat Y-up with planar y -> world z.
        direction = mn.Vector3(
            math.cos(heading) * math.cos(pitch),
            math.sin(pitch),
            math.sin(heading) * math.cos(pitch),
        )
        ray = habitat_sim.geo.Ray(origin, direction)
        hit = self.session.cast_ray(ray)
        if hit is None or not getattr(hit, "has_hit", True):
            return None
        point = getattr(hit, "point", None)
        if point is None:
            return None
        delta = point - origin
        return float(math.sqrt(delta.dot(delta)))

    def laser_ranges(
        self,
        angle_min: float,
        angle_max: float,
        num_beams: int,
        range_max: float,
    ) -> np.ndarray:
        """Sample a planar laser ring via ray casting.

        Args:
            angle_min: Start azimuth (rad), relative to agent yaw.
            angle_max: End azimuth (rad).
            num_beams: Number of beams.
            range_max: Fill value on miss (m).

        Returns:
            ``float32`` ranges of shape ``(num_beams,)``.
        """
        if self.use_mock or self.session is None:
            return np.full((int(num_beams),), 0.5 * float(range_max), dtype=np.float32)

        angles = self._angles(angle_min, angle_max, num_beams)
        ranges = np.empty((angles.shape[0],), dtype=np.float32)
        for index, yaw_offset in enumerate(angles):
            distance = self._cast_range(float(yaw_offset), 0.0, float(range_max))
            ranges[index] = float(range_max) if distance is None else distance
        return ranges

    def lidar_points(
        self,
        h_min: float,
        h_max: float,
        h_beams: int,
        v_min: float,
        v_max: float,
        v_rings: int,
        range_max: float,
    ) -> np.ndarray:
        """Sample a multi-ring lidar point cloud via ray casting.

        Args:
            h_min / h_max: Horizontal FOV (rad).
            h_beams: Horizontal beam count.
            v_min / v_max: Vertical FOV (rad).
            v_rings: Vertical ring count.
            range_max: Maximum cast distance (m); misses are skipped.

        Returns:
            ``Nx3 float32`` points in the sensor frame (x forward, y left, z up).
        """
        h_angles = self._angles(h_min, h_max, h_beams)
        v_angles = self._angles(v_min, v_max, v_rings)

        if self.use_mock or self.session is None:
            points = []
            radius = 0.5 * float(range_max)
            for pitch in v_angles:
                for yaw_offset in h_angles:
                    # Unit-radius rings scaled by range_max/2, with vertical offset from pitch.
                    points.append(
                        (
                            radius * math.cos(yaw_offset) * math.cos(pitch),
                            radius * math.sin(yaw_offset) * math.cos(pitch),
                            radius * math.sin(pitch),
                        )
                    )
            return np.asarray(points, dtype=np.float32)

        points = []
        for pitch in v_angles:
            for yaw_offset in h_angles:
                distance = self._cast_range(float(yaw_offset), float(pitch), float(range_max))
                if distance is None:
                    continue
                # Sensor frame: x forward, y left, z up (relative to agent yaw).
                points.append(
                    (
                        distance * math.cos(yaw_offset) * math.cos(pitch),
                        distance * math.sin(yaw_offset) * math.cos(pitch),
                        distance * math.sin(pitch),
                    )
                )
        if not points:
            return np.zeros((0, 3), dtype=np.float32)
        return np.asarray(points, dtype=np.float32)

    def color_depth(self) -> Tuple[np.ndarray, np.ndarray]:
        """Fetch color and depth observations.

        Returns:
            ``(color, depth)``; mock mode returns a fixed green image and constant depth.
        """
        if self.use_mock or self.session is None:
            color = np.zeros((self.height, self.width, 3), dtype=np.uint8)
            color[..., 1] = 128
            depth = np.full((self.height, self.width), 2.0, dtype=np.float32)
            return color, depth
        observations = self.session.get_sensor_observations()
        color = np.asarray(observations["rgb"])[..., :3].astype(np.uint8)
        depth = np.asarray(observations["depth"], dtype=np.float32)
        return color, depth

    def close(self) -> None:
        """Close the Habitat session and release resources."""
        if self.session is not None:
            self.session.close()
            self.session = None
