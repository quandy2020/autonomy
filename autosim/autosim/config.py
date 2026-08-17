"""Simulation config loading and validation."""

from __future__ import annotations

from pathlib import Path
from typing import Any, Dict, Mapping, MutableMapping, Union

import yaml


class Config:
    """YAML config container with startup validation of robot and sensor channels."""

    REQUIRED_SENSOR_KEYS = ("lidar_2d", "lidar_3d", "camera", "imu", "odom")

    def __init__(self, data: MutableMapping[str, Any]) -> None:
        """Build a config from an already-validated mapping.

        Args:
            data: Config dict, typically produced by :meth:`load`.
        """
        self.data = data

    def __getitem__(self, key: str) -> Any:
        """Read a top-level config entry.

        Args:
            key: Top-level key such as ``habitat``.

        Returns:
            The mapped value.

        Raises:
            KeyError: If the key is missing.
        """
        return self.data[key]

    def get(self, key: str, default: Any = None) -> Any:
        """Safely read a top-level config entry.

        Args:
            key: Top-level key.
            default: Value when the key is absent.

        Returns:
            The config value or ``default``.
        """
        return self.data.get(key, default)

    def channel_map(self) -> Dict[str, str]:
        """Expand nested config into the logical channel map used by Bridge.

        Returns:
            Keys such as ``cmd_vel`` / ``scan`` / ``points`` / ``rgb`` mapped to channel names.
        """
        robot = self.data["habitat"]["robot"]
        sensors = self.data["habitat"]["sensors"]
        return {
            "cmd_vel": robot["cmd_vel"],
            "scan": sensors["lidar_2d"]["channel"],
            "points": sensors["lidar_3d"]["channel"],
            "rgb": sensors["camera"]["rgb_channel"],
            "depth": sensors["camera"]["depth_channel"],
            "camera_info": sensors["camera"]["info_channel"],
            "imu": sensors["imu"]["channel"],
            "odom": sensors["odom"]["channel"],
            "gt_pose": robot["truth"]["channel"],
        }

    @classmethod
    def load(cls, source: Union[str, Path, Mapping[str, Any]]) -> "Config":
        """Load and validate config from a file path or in-memory mapping.

        Args:
            source: YAML path or an already-parsed ``dict`` / ``Mapping``.

        Returns:
            A validated :class:`Config` instance.

        Raises:
            ValueError: Unsupported source type, non-mapping root, or validation failure.
            OSError: Config file cannot be opened.
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
    def _require_bool(cls, block: Mapping[str, Any], path: str) -> None:
        if "enabled" not in block or not isinstance(block["enabled"], bool):
            raise ValueError(f"{path}.enabled must be a bool")

    @classmethod
    def _validate_lidar_2d(cls, block: Mapping[str, Any]) -> None:
        cls._require_bool(block, "habitat.sensors.lidar_2d")
        if int(block.get("num_beams", 0)) < 1:
            raise ValueError("habitat.sensors.lidar_2d.num_beams must be >= 1")
        if float(block["angle_max"]) < float(block["angle_min"]):
            raise ValueError("habitat.sensors.lidar_2d angle_max must be >= angle_min")
        if float(block["range_max"]) <= float(block["range_min"]):
            raise ValueError("habitat.sensors.lidar_2d range_max must be > range_min")

    @classmethod
    def _validate_lidar_3d(cls, block: Mapping[str, Any]) -> None:
        cls._require_bool(block, "habitat.sensors.lidar_3d")
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
    def validate(cls, data: Mapping[str, Any]) -> None:
        """Validate ``habitat`` robot and sensor channel settings.

        Args:
            data: Config mapping to validate.

        Raises:
            ValueError: Missing structure, empty channel names, or duplicates.
        """
        habitat = data.get("habitat")
        if not isinstance(habitat, Mapping):
            raise ValueError("habitat must be a mapping")

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

        sensors = habitat.get("sensors")
        if not isinstance(sensors, Mapping):
            raise ValueError("habitat.sensors must be a mapping")
        for key in cls.REQUIRED_SENSOR_KEYS:
            if key not in sensors or not isinstance(sensors[key], Mapping):
                raise ValueError(f"missing habitat.sensors.{key}")

        cls._validate_lidar_2d(sensors["lidar_2d"])
        cls._validate_lidar_3d(sensors["lidar_3d"])

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
        for name in names:
            if not isinstance(name, str) or not name.strip():
                raise ValueError("empty channel name in habitat.sensors or habitat.robot")
        if len(names) != len(set(names)):
            raise ValueError("duplicate channel names")
