from __future__ import annotations

from pathlib import Path
from typing import Any, Mapping, MutableMapping, Union

import yaml

ConfigDict = MutableMapping[str, Any]
ConfigSource = Union[str, Path, Mapping[str, Any]]

_REQUIRED_CHANNELS = (
    "cmd_vel",
    "scan",
    "rgb",
    "depth",
    "camera_info",
    "imu",
    "odom",
    "gt_pose",
)


class ConfigError(ValueError):
    """Invalid autosim configuration."""


def load_config(source: ConfigSource) -> ConfigDict:
    if isinstance(source, (str, Path)):
        path = Path(source)
        with path.open("r", encoding="utf-8") as f:
            data = yaml.safe_load(f)
    elif isinstance(source, Mapping):
        data = dict(source)
    else:
        raise ConfigError(f"unsupported config source: {type(source)!r}")

    if not isinstance(data, dict):
        raise ConfigError("config root must be a mapping")
    _validate(data)
    return data


def _validate(cfg: Mapping[str, Any]) -> None:
    channels = cfg.get("channels")
    if not isinstance(channels, Mapping):
        raise ConfigError("channels must be a mapping")
    names: list[str] = []
    for key in _REQUIRED_CHANNELS:
        if key not in channels:
            raise ConfigError(f"missing channel: {key}")
        name = channels[key]
        if not isinstance(name, str) or not name.strip():
            raise ConfigError(f"empty channel name: {key}")
        names.append(name)
    if len(names) != len(set(names)):
        raise ConfigError("duplicate channel names")
    backend = cfg.get("scene", {}).get("backend")
    if backend not in ("minimal", "habitat"):
        raise ConfigError("scene.backend must be 'minimal' or 'habitat'")
    if "truth" not in cfg or "enabled" not in cfg["truth"]:
        raise ConfigError("truth.enabled required")
