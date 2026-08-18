# Copyright 2026 The Openbot Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Habitat-Sim session wrapper for agent pose and ``cast_ray`` sensing.

Planar bridge coordinates map to Habitat Y-up: planar ``y`` → world ``z``.
Requires ``habitat-sim``; there is no mock backend.
"""

from __future__ import annotations

import math
import warnings
from pathlib import Path
from typing import Any, Mapping, Tuple

import numpy as np

from autosim.urdf import UrdfModel


class Simulator:
    """Scene and agent pose backend (Habitat only)."""

    def __init__(
        self,
        backend: str,
        width: int,
        height: int,
        settings: Mapping[str, Any] | None = None,
        open_session: bool = True,
    ) -> None:
        """Construct the simulator.

        Args:
            backend: ``minimal`` (empty path) or ``habitat`` (scene file).
            width: Camera width in pixels.
            height: Camera height in pixels.
            settings: Full config map (needs ``habitat``).
            open_session: When ``False``, skip Habitat open (unit tests / URDF only).

        Raises:
            ImportError: Habitat required but not installed.
            FileNotFoundError: Scene path invalid.
        """
        self.backend = backend
        self.width = int(width)
        self.height = int(height)
        self.settings = dict(settings or {})
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.session = None
        self.articulated = None
        self.urdf = self.load_urdf_model()

        if open_session:
            self.open_habitat()

    @classmethod
    def create(cls, settings: Mapping[str, Any]) -> "Simulator":
        """Factory from config; always opens Habitat.

        Args:
            settings: Validated config (e.g. ``Config.data``).

        Returns:
            Ready :class:`Simulator`.

        Raises:
            ImportError: ``habitat-sim`` missing.
            FileNotFoundError: Scene path set but missing on disk.
        """
        habitat = settings["habitat"]
        scene_path = str(habitat.get("path") or "").strip()
        backend = "minimal" if not scene_path else "habitat"
        camera = habitat["sensors"]["camera"]
        return cls(
            backend=backend,
            width=int(camera["width"]),
            height=int(camera["height"]),
            settings=settings,
            open_session=True,
        )

    def load_urdf_model(self) -> UrdfModel | None:
        """Parse ``habitat.robot.urdf`` mounts when configured.

        Returns:
            :class:`UrdfModel` or ``None``.
        """
        robot = self.settings.get("habitat", {}).get("robot", {})
        urdf = robot.get("urdf", "") if isinstance(robot, Mapping) else ""
        try:
            return UrdfModel.load(str(urdf or ""))
        except (FileNotFoundError, ValueError) as exc:
            warnings.warn(f"URDF mounts unavailable ({exc!r})", stacklevel=2)
            return None

    def open_habitat(self) -> None:
        """Open Habitat, load optional URDF, and apply spawn pose.

        Raises:
            ImportError: ``habitat-sim`` missing.
            FileNotFoundError: Required scene path missing on disk.
        """
        try:
            import habitat_sim
        except ImportError as exc:
            raise ImportError(
                "habitat-sim is not installed. Install with: pip install habitat-sim"
            ) from exc

        configuration = self.habitat_configuration(habitat_sim)
        agent_configuration = self.agent_configuration(habitat_sim)
        self.session = habitat_sim.Simulator(
            habitat_sim.Configuration(configuration, [agent_configuration])
        )
        self.attach_urdf(habitat_sim)
        spawn = self.settings["habitat"].get("spawn", [0.0, 0.0, 0.0])
        self.reset(float(spawn[0]), float(spawn[1]), float(spawn[2]))

    def habitat_configuration(self, habitat_sim: Any) -> Any:
        """Build ``SimulatorConfiguration`` from settings."""
        configuration = habitat_sim.SimulatorConfiguration()
        configuration.gpu_device_id = int(self.settings["habitat"]["gpu"])
        configuration.enable_physics = self.urdf is not None
        scene_path = str(self.settings["habitat"].get("path") or "").strip()
        if not scene_path:
            if hasattr(habitat_sim, "STAGE_EMPTY_SCENE"):
                configuration.scene_id = habitat_sim.STAGE_EMPTY_SCENE
            else:
                configuration.scene_id = habitat_sim.utils.settings.default_sim_settings.get(
                    "scene", "NONE"
                )
            return configuration
        if not Path(scene_path).exists():
            raise FileNotFoundError(f"scene path does not exist: {scene_path}")
        configuration.scene_id = scene_path
        return configuration

    def agent_configuration(self, habitat_sim: Any) -> Any:
        """RGB-D agent specs; camera height from URDF when present."""
        agent_configuration = habitat_sim.agent.AgentConfiguration()
        color_sensor = habitat_sim.CameraSensorSpec()
        color_sensor.uuid = "rgb"
        color_sensor.sensor_type = habitat_sim.SensorType.COLOR
        color_sensor.resolution = [self.height, self.width]
        depth_sensor = habitat_sim.CameraSensorSpec()
        depth_sensor.uuid = "depth"
        depth_sensor.sensor_type = habitat_sim.SensorType.DEPTH
        depth_sensor.resolution = [self.height, self.width]
        height = self.camera_height()
        if height is not None:
            color_sensor.position = [0.0, float(height), 0.0]
            depth_sensor.position = [0.0, float(height), 0.0]
        agent_configuration.sensor_specifications = [color_sensor, depth_sensor]
        return agent_configuration

    def camera_height(self) -> float | None:
        """Habitat Y height for cameras from URDF, else ``None`` (default)."""
        if self.urdf is None:
            return None
        if "camera_link" in self.urdf.mounts:
            return float(self.urdf.camera_xyz()[2])
        return float(self.urdf.laser_xyz()[2])

    def attach_urdf(self, habitat_sim: Any) -> None:
        """Instance kinematic articulated object from configured URDF."""
        if self.urdf is None or self.session is None:
            return
        try:
            manager = self.session.get_articulated_object_manager()
            robot = manager.add_articulated_object_from_urdf(
                filepath=str(self.urdf.path),
                fixed_base=False,
            )
            robot.motion_type = habitat_sim.physics.MotionType.KINEMATIC
            try:
                robot.override_collidable_state(False)
            except Exception:
                try:
                    robot.collision_group = habitat_sim.physics.CollisionGroups.Noncollidable
                except Exception:
                    pass
            self.articulated = robot
            self.urdf_z_up = True
        except Exception as exc:
            warnings.warn(
                f"Habitat URDF load failed ({exc!r}); mounts still applied to rays",
                stacklevel=2,
            )
            self.articulated = None

    def sync_articulated(self, mn: Any) -> None:
        """Keep Habitat URDF root aligned; apply ROS Z-up → Habitat Y-up."""
        if self.articulated is None:
            return
        yaw = mn.Quaternion.rotation(mn.Rad(self.yaw), mn.Vector3(0, 1, 0))
        # ROS URDF is Z-up; Habitat is Y-up → rotate −90° about X.
        z_up_to_y_up = mn.Quaternion.rotation(mn.Rad(-math.pi / 2.0), mn.Vector3(1, 0, 0))
        self.articulated.translation = mn.Vector3(self.x, 0.0, self.y)
        self.articulated.rotation = yaw * z_up_to_y_up

    def reset(self, x: float, y: float, yaw: float) -> None:
        """Reset agent pose.

        Args:
            x: Planar x (m) → Habitat world x.
            y: Planar y (m) → Habitat world z.
            yaw: Heading about vertical (rad).
        """
        self.set_pose(x, y, yaw)

    def set_pose(self, x: float, y: float, yaw: float) -> None:
        """Write planar pose and sync Habitat agent when live.

        Args:
            x: Planar x (m).
            y: Planar y (m).
            yaw: Heading (rad).
        """
        self.x = float(x)
        self.y = float(y)
        self.yaw = float(yaw)
        if self.session is None:
            return
        import magnum as mn

        agent = self.session.get_agent(0)
        state = agent.get_state()
        state.position = mn.Vector3(self.x, 0.0, self.y)
        state.rotation = mn.Quaternion.rotation(mn.Rad(self.yaw), mn.Vector3(0, 1, 0))
        agent.set_state(state, infer_sensor_states=True)
        self.sync_articulated(mn)

    def laser_origin(self) -> Tuple[float, float, float]:
        """Habitat world origin for lidar rays (Y-up).

        Returns:
            ``(x, y, z)`` with planar offsets rotated by yaw.
        """
        mx = my = mz = 0.0
        if self.urdf is not None:
            mx, my, mz = self.urdf.laser_xyz()
        cos_y = math.cos(self.yaw)
        sin_y = math.sin(self.yaw)
        return (
            self.x + cos_y * mx - sin_y * my,
            mz,
            self.y + sin_y * mx + cos_y * my,
        )

    def step(self) -> None:
        """Advance one sim step (pose is driven by :meth:`set_pose`)."""
        return None

    def pose(self) -> Tuple[float, float, float]:
        """Current planar pose.

        Returns:
            ``(x, y, yaw)``.
        """
        return self.x, self.y, self.yaw

    def linspace_angles(self, angle_min: float, angle_max: float, count: int) -> np.ndarray:
        """Evenly spaced angles over a FOV.

        Args:
            angle_min: Start angle (rad).
            angle_max: End angle (rad).
            count: Sample count (≥ 1).

        Returns:
            ``float64`` angle array.
        """
        if count <= 1:
            return np.array([float(angle_min)], dtype=np.float64)
        return np.linspace(float(angle_min), float(angle_max), num=int(count), dtype=np.float64)

    def eye_height(self) -> float:
        """Vertical Habitat Y for panoramic / lidar casts (meters)."""
        if self.urdf is not None:
            return float(self.urdf.laser_xyz()[2])
        return 0.5

    def cast_from_origin(
        self,
        origin_x: float,
        origin_y: float,
        origin_z: float,
        yaw: float,
        pitch: float,
        range_max: float,
    ) -> Tuple[float, float, float] | None:
        """Cast one ray; return map-frame hit ``(x, y, z)`` with z up.

        Args:
            origin_x: Habitat world x (= map x).
            origin_y: Habitat world y (up).
            origin_z: Habitat world z (= map y).
            yaw: Azimuth in the horizontal plane (rad).
            pitch: Elevation (rad).
            range_max: Maximum range (m).

        Returns:
            Map-frame point, or ``None`` on miss.
        """
        if self.session is None:
            raise RuntimeError("Habitat session is not open")
        import habitat_sim
        import magnum as mn

        origin = mn.Vector3(float(origin_x), float(origin_y), float(origin_z))
        direction = mn.Vector3(
            math.cos(yaw) * math.cos(pitch),
            math.sin(pitch),
            math.sin(yaw) * math.cos(pitch),
        )
        ray = habitat_sim.geo.Ray(origin, direction)
        try:
            hit = self.session.cast_ray(ray, max_distance=float(range_max))
        except TypeError:
            hit = self.session.cast_ray(ray)
        if hit is None or not getattr(hit, "has_hit", False):
            return None
        if hasattr(hit, "ray_distance"):
            distance = float(hit.ray_distance)
        else:
            point = getattr(hit, "point", None)
            if point is None:
                return None
            delta = point - origin
            distance = float(math.sqrt(delta.dot(delta)))
        if not math.isfinite(distance) or distance > float(range_max):
            return None
        hx = float(origin_x) + distance * math.cos(yaw) * math.cos(pitch)
        hy = float(origin_y) + distance * math.sin(pitch)
        hz = float(origin_z) + distance * math.sin(yaw) * math.cos(pitch)
        return (hx, hz, hy)

    def world_cloud(
        self,
        planar_x: float,
        planar_y: float,
        eye_y: float,
        h_angles: np.ndarray,
        v_angles: np.ndarray,
        range_max: float,
    ) -> np.ndarray:
        """Panoramic cloud in map frame (z-up) from a fixed planar origin.

        Args:
            planar_x: Map / Habitat x.
            planar_y: Map y (= Habitat z).
            eye_y: Habitat up height for the cast origin.
            h_angles: Absolute yaw samples (rad).
            v_angles: Pitch samples (rad).
            range_max: Cast budget (m).

        Returns:
            ``Nx3 float32`` map-frame points.
        """
        points = []
        for pitch in v_angles:
            for yaw in h_angles:
                hit = self.cast_from_origin(
                    planar_x, eye_y, planar_y, float(yaw), float(pitch), float(range_max)
                )
                if hit is None:
                    continue
                points.append(hit)
        if not points:
            return np.zeros((0, 3), dtype=np.float32)
        return np.asarray(points, dtype=np.float32)

    def cast_range(self, yaw_offset: float, pitch: float, range_max: float) -> float | None:
        """Cast one Habitat ray and return hit distance within ``range_max``.

        Args:
            yaw_offset: Azimuth relative to agent yaw (rad).
            pitch: Elevation (rad); 0 is horizontal.
            range_max: Maximum valid range (m); farther hits count as miss.

        Returns:
            Distance in meters, or ``None`` on miss / beyond ``range_max``.
        """
        ox, oy, oz = self.laser_origin()
        hit = self.cast_from_origin(
            ox, oy, oz, self.yaw + yaw_offset, pitch, range_max
        )
        if hit is None:
            return None
        dx = hit[0] - ox
        dy = hit[2] - oy
        dz = hit[1] - oz
        return float(math.sqrt(dx * dx + dy * dy + dz * dz))

    def laser_ranges(
        self,
        angle_min: float,
        angle_max: float,
        num_beams: int,
        range_max: float,
    ) -> np.ndarray:
        """Planar lidar ranges via ray casting.

        Args:
            angle_min: Start azimuth (rad).
            angle_max: End azimuth (rad).
            num_beams: Beam count.
            range_max: Miss fill value (m).

        Returns:
            ``float32`` ranges ``(num_beams,)``.
        """
        angles = self.linspace_angles(angle_min, angle_max, num_beams)
        ranges = np.empty((angles.shape[0],), dtype=np.float32)
        for index, yaw_offset in enumerate(angles):
            distance = self.cast_range(float(yaw_offset), 0.0, float(range_max))
            ranges[index] = float(range_max) if distance is None else distance
        return ranges

    @staticmethod
    def spherical_point(
        radius: float, yaw_offset: float, pitch: float
    ) -> Tuple[float, float, float]:
        """Sensor-frame point from spherical coordinates.

        Args:
            radius: Range (m).
            yaw_offset: Azimuth in sensor frame (rad).
            pitch: Elevation (rad).

        Returns:
            ``(x, y, z)`` with x forward, y left, z up.
        """
        return (
            radius * math.cos(yaw_offset) * math.cos(pitch),
            radius * math.sin(yaw_offset) * math.cos(pitch),
            radius * math.sin(pitch),
        )

    def raycast_cloud(
        self, h_angles: np.ndarray, v_angles: np.ndarray, range_max: float
    ) -> np.ndarray:
        """Multi-ring cloud from Habitat ray casts; misses omitted.

        Args:
            h_angles: Horizontal samples (rad).
            v_angles: Vertical samples (rad).
            range_max: Cast budget (m).

        Returns:
            ``Nx3 float32`` points (possibly empty).
        """
        points = []
        for pitch in v_angles:
            for yaw_offset in h_angles:
                distance = self.cast_range(float(yaw_offset), float(pitch), float(range_max))
                if distance is None:
                    continue
                points.append(self.spherical_point(distance, float(yaw_offset), float(pitch)))
        if not points:
            return np.zeros((0, 3), dtype=np.float32)
        return np.asarray(points, dtype=np.float32)

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
        """Sample a multi-ring lidar cloud.

        Returns:
            ``Nx3 float32`` in the sensor frame.
        """
        h_angles = self.linspace_angles(h_min, h_max, h_beams)
        v_angles = self.linspace_angles(v_min, v_max, v_rings)
        return self.raycast_cloud(h_angles, v_angles, range_max)

    def color_depth(self) -> Tuple[np.ndarray, np.ndarray]:
        """RGB and depth observations from Habitat.

        Returns:
            ``(color, depth)``.

        Raises:
            RuntimeError: Session not open.
        """
        if self.session is None:
            raise RuntimeError("Habitat session is not open")
        observations = self.session.get_sensor_observations()
        color = np.asarray(observations["rgb"])[..., :3].astype(np.uint8)
        depth = np.asarray(observations["depth"], dtype=np.float32)
        return color, depth

    def close(self) -> None:
        """Release the Habitat session."""
        if self.session is not None:
            self.session.close()
            self.session = None
