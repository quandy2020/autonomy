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

"""YAML configuration load and startup validation for autosim."""

from __future__ import annotations

from pathlib import Path
from typing import Any, Dict, Mapping, MutableMapping, Union

import yaml

from autosim.urdf import UrdfModel


class Config:
    """Validated habitat/robot/sensor configuration."""

    REQUIRED_SENSOR_KEYS = ("lidar_2d", "lidar_3d", "camera", "imu", "odom")

    def __init__(self, data: MutableMapping[str, Any]) -> None:
        """Wrap an already-validated mapping.

        Args:
            data: Config dict from :meth:`load`.
        """
        self.data = data

    def __getitem__(self, key: str) -> Any:
        """Top-level lookup.

        Args:
            key: e.g. ``habitat``.

        Returns:
            Mapped value.

        Raises:
            KeyError: Missing key.
        """
        return self.data[key]

    def get(self, key: str, default: Any = None) -> Any:
        """Top-level lookup with default.

        Args:
            key: Top-level key.
            default: Fallback when absent.

        Returns:
            Value or ``default``.
        """
        return self.data.get(key, default)

    def channel_map(self) -> Dict[str, str]:
        """Logical bridge keys → channel names."""
        robot = self.data["habitat"]["robot"]
        sensors = self.data["habitat"]["sensors"]
        mapping = self.data["habitat"].get("map") or {}
        ply = mapping.get("ply") or {}
        grid = mapping.get("grid") or {}
        tf_cfg = robot.get("tf") or {}
        clock_cfg = robot.get("clock") or {}
        footprint_cfg = robot.get("footprint") or {}
        channels = {
            "cmd_vel": robot["cmd_vel"],
            "scan": sensors["lidar_2d"]["channel"],
            "points": sensors["lidar_3d"]["channel"],
            "rgb": sensors["camera"]["rgb_channel"],
            "depth": sensors["camera"]["depth_channel"],
            "camera_info": sensors["camera"]["info_channel"],
            "imu": sensors["imu"]["channel"],
            "odom": sensors["odom"]["channel"],
            "gt_pose": robot["truth"]["channel"],
            "map_grid": grid.get("channel", "/map"),
            "tf": tf_cfg.get("channel", "/tf"),
            "tf_static": tf_cfg.get("static_channel", "/tf_static"),
            "clock": clock_cfg.get("channel", "/clock"),
        }
        if isinstance(footprint_cfg, Mapping) and footprint_cfg.get("enabled", False):
            channels["footprint"] = footprint_cfg.get("channel", "/footprint")
        camera_cfg = sensors.get("camera") or {}
        if camera_cfg.get("semantic_enabled", False):
            semantic_channel = str(camera_cfg.get("semantic_channel") or "").strip()
            if semantic_channel:
                channels["semantic"] = semantic_channel
            semantic_ids_channel = str(camera_cfg.get("semantic_ids_channel") or "").strip()
            if semantic_ids_channel:
                channels["semantic_ids"] = semantic_ids_channel
        ply_channel = str(ply.get("channel") or "").strip()
        if ply_channel:
            channels["map_cloud"] = ply_channel
        return channels

    @classmethod
    def load(cls, source: Union[str, Path, Mapping[str, Any]]) -> "Config":
        """Load YAML path or in-memory mapping, then validate.

        Args:
            source: File path or mapping.

        Returns:
            Validated :class:`Config`.

        Raises:
            ValueError: Bad source or validation failure.
            OSError: File unreadable.
        """
        if isinstance(source, (str, Path)):
            path = Path(source)
            with path.open("r", encoding="utf-8") as handle:
                data = yaml.safe_load(handle)
        elif isinstance(source, Mapping):
            data = dict(source)
        else:
            raise ValueError(f"unsupported config source: {type(source)!r}")

        if not isinstance(data, dict):
            raise ValueError("config root must be a mapping")
        cls.validate(data)
        return cls(data)

    @classmethod
    def validate_urdf(cls, urdf: Any) -> None:
        """Require ``habitat.robot.urdf`` empty or an existing file.

        Args:
            urdf: Path string or empty.

        Raises:
            ValueError: Non-string or missing file.
        """
        if urdf is None:
            return
        if not isinstance(urdf, str):
            raise ValueError("habitat.robot.urdf must be a string")
        path = UrdfModel.resolve(urdf)
        if path is None:
            return
        if not path.is_file():
            raise ValueError(f"habitat.robot.urdf not found: {path}")

    @classmethod
    def require_bool(cls, block: Mapping[str, Any], path: str) -> None:
        """Require ``enabled`` to be a bool.

        Args:
            block: Mapping that must contain ``enabled``.
            path: Dotted path for error messages.

        Raises:
            ValueError: Missing or non-bool ``enabled``.
        """
        if "enabled" not in block or not isinstance(block["enabled"], bool):
            raise ValueError(f"{path}.enabled must be a bool")

    @classmethod
    def validate_lidar_2d(cls, block: Mapping[str, Any]) -> None:
        """Validate planar lidar geometry and enable flag.

        Args:
            block: ``habitat.sensors.lidar_2d``.

        Raises:
            ValueError: Invalid fields.
        """
        cls.require_bool(block, "habitat.sensors.lidar_2d")
        if int(block.get("num_beams", 0)) < 1:
            raise ValueError("habitat.sensors.lidar_2d.num_beams must be >= 1")
        if float(block["angle_max"]) < float(block["angle_min"]):
            raise ValueError("habitat.sensors.lidar_2d angle_max must be >= angle_min")
        if float(block["range_max"]) <= float(block["range_min"]):
            raise ValueError("habitat.sensors.lidar_2d range_max must be > range_min")

    @classmethod
    def validate_lidar_3d(cls, block: Mapping[str, Any]) -> None:
        """Validate multi-ring lidar geometry and enable flag.

        Args:
            block: ``habitat.sensors.lidar_3d``.

        Raises:
            ValueError: Invalid fields.
        """
        cls.require_bool(block, "habitat.sensors.lidar_3d")
        horizontal = block.get("horizontal")
        vertical = block.get("vertical")
        if not isinstance(horizontal, Mapping) or not isinstance(vertical, Mapping):
            raise ValueError("habitat.sensors.lidar_3d requires horizontal and vertical mappings")
        if int(horizontal.get("num_beams", 0)) < 1:
            raise ValueError("habitat.sensors.lidar_3d.horizontal.num_beams must be >= 1")
        if int(vertical.get("num_rings", 0)) < 1:
            raise ValueError("habitat.sensors.lidar_3d.vertical.num_rings must be >= 1")
        if float(horizontal["angle_max"]) < float(horizontal["angle_min"]):
            raise ValueError("habitat.sensors.lidar_3d.horizontal angle_max must be >= angle_min")
        if float(vertical["angle_max"]) < float(vertical["angle_min"]):
            raise ValueError("habitat.sensors.lidar_3d.vertical angle_max must be >= angle_min")
        if float(block["range_max"]) <= float(block["range_min"]):
            raise ValueError("habitat.sensors.lidar_3d range_max must be > range_min")

    @classmethod
    def require_nonneg(cls, value: Any, path: str) -> float:
        """Require a finite noise stddev ≥ 0.

        Args:
            value: Candidate σ.
            path: Dotted path for errors.

        Returns:
            Parsed float σ.

        Raises:
            ValueError: Missing, non-numeric, or negative.
        """
        try:
            sigma = float(value)
        except (TypeError, ValueError) as exc:
            raise ValueError(f"{path} must be a number >= 0") from exc
        if sigma < 0.0:
            raise ValueError(f"{path} must be >= 0")
        return sigma

    @classmethod
    def validate_noise_fields(cls, sensors: Mapping[str, Any]) -> None:
        """Validate Gaussian σ fields on lidar / odom / imu / camera.

        Args:
            sensors: ``habitat.sensors`` mapping.

        Raises:
            ValueError: Negative or malformed noise.
        """
        cls.require_nonneg(sensors["lidar_2d"].get("noise", 0.0), "habitat.sensors.lidar_2d.noise")
        cls.require_nonneg(sensors["lidar_3d"].get("noise", 0.0), "habitat.sensors.lidar_3d.noise")
        cls.require_nonneg(sensors["odom"].get("noise", 0.0), "habitat.sensors.odom.noise")

        imu_noise = sensors["imu"].get("noise", {})
        if imu_noise is None:
            imu_noise = {}
        if not isinstance(imu_noise, Mapping):
            raise ValueError("habitat.sensors.imu.noise must be a mapping")
        cls.require_nonneg(imu_noise.get("gyro", 0.0), "habitat.sensors.imu.noise.gyro")
        cls.require_nonneg(imu_noise.get("accel", 0.0), "habitat.sensors.imu.noise.accel")

        cam_noise = sensors["camera"].get("noise", {})
        if cam_noise is None:
            cam_noise = {}
        if not isinstance(cam_noise, Mapping):
            raise ValueError("habitat.sensors.camera.noise must be a mapping")
        cls.require_nonneg(cam_noise.get("depth", 0.0), "habitat.sensors.camera.noise.depth")

    @classmethod
    def validate(cls, data: Mapping[str, Any]) -> None:
        """Validate habitat robot and sensor channel settings.

        Args:
            data: Root config mapping.

        Raises:
            ValueError: Structure, channel, or geometry errors.
        """
        habitat = data.get("habitat")
        if not isinstance(habitat, Mapping):
            raise ValueError("habitat must be a mapping")

        mode = habitat.get("mode", "nav")
        if mode is not None:
            if not isinstance(mode, str) or str(mode).strip().lower() not in ("slam", "nav"):
                raise ValueError("habitat.mode must be 'slam' or 'nav'")

        robot = habitat.get("robot")
        if not isinstance(robot, Mapping):
            raise ValueError("habitat.robot must be a mapping")
        truth = robot.get("truth")
        if not isinstance(truth, Mapping) or "enabled" not in truth:
            raise ValueError("habitat.robot.truth.enabled required")
        if not isinstance(robot.get("cmd_vel"), str) or not robot["cmd_vel"].strip():
            raise ValueError("empty channel name: habitat.robot.cmd_vel")
        if not isinstance(truth.get("channel"), str) or not truth["channel"].strip():
            raise ValueError("empty channel name: habitat.robot.truth.channel")
        cls.validate_urdf(robot.get("urdf", ""))

        sensors = habitat.get("sensors")
        if not isinstance(sensors, Mapping):
            raise ValueError("habitat.sensors must be a mapping")
        for key in cls.REQUIRED_SENSOR_KEYS:
            if key not in sensors or not isinstance(sensors[key], Mapping):
                raise ValueError(f"missing habitat.sensors.{key}")

        cls.validate_lidar_2d(sensors["lidar_2d"])
        cls.validate_lidar_3d(sensors["lidar_3d"])
        cls.validate_noise_fields(sensors)
        cls.validate_footprint(robot.get("footprint"), robot.get("urdf", ""))
        cls.validate_map(habitat.get("map"))

        names = [
            robot["cmd_vel"],
            sensors["lidar_2d"]["channel"],
            sensors["lidar_3d"]["channel"],
            sensors["camera"]["rgb_channel"],
            sensors["camera"]["depth_channel"],
            sensors["camera"]["info_channel"],
            sensors["imu"]["channel"],
            sensors["odom"]["channel"],
            truth["channel"],
        ]
        mapping = habitat.get("map")
        slam = str(habitat.get("mode", "nav")).strip().lower() == "slam"
        map_publish = (
            False
            if slam
            else bool(mapping.get("publish", True))
            if isinstance(mapping, Mapping)
            else True
        )
        if isinstance(mapping, Mapping) and mapping.get("enabled", False) and map_publish:
            ply = mapping.get("ply") or {}
            grid = mapping.get("grid") or {}
            ply_channel = str(ply.get("channel") or "").strip()
            if ply_channel:
                names.append(ply_channel)
            names.append(grid["channel"])
        tf_cfg = robot.get("tf") or {}
        if isinstance(tf_cfg, Mapping) and tf_cfg.get("enabled", False):
            names.extend([tf_cfg.get("channel", "/tf"), tf_cfg.get("static_channel", "/tf_static")])
        clock_cfg = robot.get("clock") or {}
        if isinstance(clock_cfg, Mapping) and clock_cfg.get("enabled", False):
            names.append(clock_cfg.get("channel", "/clock"))
        footprint = robot.get("footprint")
        if isinstance(footprint, Mapping) and footprint.get("enabled", False):
            names.append(footprint.get("channel", "/footprint"))
        for name in names:
            if not isinstance(name, str) or not name.strip():
                raise ValueError("empty channel name in habitat.sensors or habitat.robot")
        if len(names) != len(set(names)):
            raise ValueError("duplicate channel names")

    @classmethod
    def validate_footprint(cls, block: Any, urdf: Any = "") -> None:
        """Validate optional ``habitat.robot.footprint`` polygon output."""
        if block is None:
            return
        if not isinstance(block, Mapping):
            raise ValueError("habitat.robot.footprint must be a mapping")
        cls.require_bool(block, "habitat.robot.footprint")
        if not block.get("enabled", False):
            return
        if not isinstance(block.get("channel"), str) or not str(block["channel"]).strip():
            raise ValueError("habitat.robot.footprint.channel must be a non-empty string")
        if not isinstance(block.get("frame"), str) or not str(block["frame"]).strip():
            raise ValueError("habitat.robot.footprint.frame must be a non-empty string")
        points = block.get("points")
        if points is None:
            if str(urdf or "").strip():
                return
            raise ValueError(
                "habitat.robot.footprint.points required when URDF auto-derivation is unavailable"
            )
        if not isinstance(points, list) or len(points) < 3:
            raise ValueError("habitat.robot.footprint.points must contain at least 3 points")
        for index, point in enumerate(points):
            if not isinstance(point, (list, tuple)) or len(point) not in (2, 3):
                raise ValueError(
                    f"habitat.robot.footprint.points[{index}] must be [x, y] or [x, y, z]"
                )
            try:
                float(point[0])
                float(point[1])
                if len(point) == 3:
                    float(point[2])
            except (TypeError, ValueError) as exc:
                raise ValueError(
                    f"habitat.robot.footprint.points[{index}] must be numeric"
                ) from exc

    @classmethod
    def validate_map(cls, block: Any) -> None:
        """Validate optional ``habitat.map`` panoramic GT block."""
        if block is None:
            return
        if not isinstance(block, Mapping):
            raise ValueError("habitat.map must be a mapping")
        if "enabled" in block and not isinstance(block["enabled"], bool):
            raise ValueError("habitat.map.enabled must be a bool")
        if "publish" in block and not isinstance(block["publish"], bool):
            raise ValueError("habitat.map.publish must be a bool")
        if not block.get("enabled", False):
            return
        if not isinstance(block.get("frame"), str) or not str(block["frame"]).strip():
            raise ValueError("habitat.map.frame must be a non-empty string")
        ply = block.get("ply")
        grid = block.get("grid")
        if not isinstance(ply, Mapping):
            raise ValueError("habitat.map.ply must be a mapping")
        if not isinstance(grid, Mapping):
            raise ValueError("habitat.map.grid must be a mapping")
        file_path = str(ply.get("file") or "").strip()
        source_path = str(ply.get("source") or "").strip()
        channel = str(ply.get("channel") or "").strip()
        publish = bool(block.get("publish", True))
        if publish and not file_path and not source_path and not channel:
            raise ValueError("habitat.map.ply requires file, source, and/or channel")
        if publish and (
            not isinstance(grid.get("channel"), str) or not str(grid["channel"]).strip()
        ):
            raise ValueError("habitat.map.grid.channel must be a non-empty string")
        # horizontal/vertical/range_max are only required for Habitat ray-cast mode.
        # When ply.source is set the cloud is loaded from an external file and these
        # fields are not needed.
        if not source_path:
            horizontal = ply.get("horizontal")
            vertical = ply.get("vertical")
            if not isinstance(horizontal, Mapping) or not isinstance(vertical, Mapping):
                raise ValueError("habitat.map.ply requires horizontal and vertical mappings")
            if int(horizontal.get("num_beams", 0)) < 1:
                raise ValueError("habitat.map.ply.horizontal.num_beams must be >= 1")
            if int(vertical.get("num_rings", 0)) < 1:
                raise ValueError("habitat.map.ply.vertical.num_rings must be >= 1")
            if float(ply.get("range_max", 0.0)) <= 0.0:
                raise ValueError("habitat.map.ply.range_max must be > 0")
        if float(grid.get("resolution", 0.0)) <= 0.0:
            raise ValueError("habitat.map.grid.resolution must be > 0")
        if float(grid["z_max"]) < float(grid["z_min"]):
            raise ValueError("habitat.map.grid z_max must be >= z_min")
        rate = float(block.get("rate_hz", 0.0))
        if rate < 0.0:
            raise ValueError("habitat.map.rate_hz must be >= 0")
        stride = int(ply.get("stride", 1))
        if stride < 1:
            raise ValueError("habitat.map.ply.stride must be >= 1")
