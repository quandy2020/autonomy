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

"""Compose a run from a base config, scene, robot, and sensor selection."""

from __future__ import annotations

import re
from pathlib import Path
from typing import Any, Dict, Mapping, Sequence, Set, Tuple

from autosim.runtime.paths import project_root
from autosim.runtime.settings import Config


EMPTY_SCENES = frozenset({"empty", "none", "studio"})

SENSOR_KEYS = (
    "camera",
    "lidar_2d",
    "lidar_3d",
    "stereo",
    "range",
    "imu",
    "odom",
    "chase",
    "topdown",
    "joint_states",
    "cameras",
    "panorama",
)

SENSOR_ALIASES = {
    "scan": "lidar_2d",
    "laser": "lidar_2d",
    "lidar3d": "lidar_3d",
    "points": "lidar_3d",
    "rgb": "camera",
    "depth": "camera",
    "tof": "range",
    "joints": "joint_states",
    "surround": "cameras",
    "pano": "panorama",
}

# ``all`` and ``none`` are handled explicitly because they change strictness.
SENSOR_PRESETS = {
    "basic": ("camera", "lidar_2d", "imu", "odom"),
    "rgbd": ("camera", "imu", "odom"),
    "lidar": ("lidar_2d", "imu", "odom"),
    "all": None,
    "none": (),
}

# These fields describe one robot model.  When --robot switches models, the
# base config's values must not leak into the selected profile (for example,
# default.yaml's TurtleBot URDF and wheel separation).  Robot YAML values are
# merged back afterwards; apply_profile() fills anything the YAML omits.
PROFILE_SPECIFIC_ROBOT_KEYS = (
    "urdf",
    "usd_robot",
    "usd_prim",
    "wheel_separation",
    "wheel_radius",
    "base_link_height",
    "sensor_frames",
    "instance",
    "fixed_base",
    "root_height",
    "joint_positions",
    "max_linear",
    "max_angular",
)


def _config_root() -> Path:
    """Return the autosim ``config`` directory."""
    return project_root() / "config"


def _yaml_names(folder: Path) -> list[str]:
    """Return public YAML stems, excluding partials such as ``_base``."""
    if not folder.is_dir():
        return []
    return sorted(
        path.stem for path in folder.glob("*.yaml") if not path.name.startswith("_")
    )


def scene_names() -> list[str]:
    """Names accepted by ``--scene``."""
    return _yaml_names(_config_root() / "scenes")


def robot_names() -> list[str]:
    """Names accepted by ``--robot``."""
    return _yaml_names(_config_root() / "robots")


def sensor_names() -> list[str]:
    """Canonical sensor blocks accepted by ``--sensors``."""
    return list(SENSOR_KEYS)


def catalog() -> Dict[str, list[str]]:
    """Return the user-facing scene, robot, and sensor catalog."""
    return {
        "scenes": [*scene_names(), "empty"],
        "robots": robot_names(),
        "sensors": sensor_names(),
    }


def format_catalog() -> str:
    """Format the catalog for ``autosim --list``."""
    entries = catalog()
    lines = [
        "scenes: " + ", ".join(entries["scenes"]),
        "robots: " + ", ".join(entries["robots"]),
        "sensors: " + ", ".join(entries["sensors"]),
        "presets: basic, rgbd, lidar, all, none",
    ]
    return "\n".join(lines)


def _existing_path(value: str) -> Path | None:
    """Find a user path relative to cwd or the autosim project root."""
    path = Path(value).expanduser()
    candidates = [path] if path.is_absolute() else [Path.cwd() / path, project_root() / path]
    for candidate in candidates:
        if candidate.is_file():
            return candidate.resolve()
    return None


def _resolve_scene(value: str | Path) -> Tuple[Dict[str, Any], bool]:
    """Return a scene patch and whether it selects the empty studio."""
    text = str(value).strip()
    if not text:
        raise ValueError("--scene must not be empty")
    if text.lower() in EMPTY_SCENES:
        return {"habitat": {"path": ""}}, True

    suffix = Path(text).suffix.lower()
    if suffix in (".yaml", ".yml"):
        path = _existing_path(text)
        if path is None:
            named = _config_root() / "scenes" / Path(text).name
            path = named if named.is_file() else None
        if path is None:
            raise ValueError(f"scene file not found: {text}")
        return Config.read(path), False

    named = _config_root() / "scenes" / f"{text}.yaml"
    if named.is_file():
        return Config.read(named), False

    path = _existing_path(text)
    if path is not None:
        return {"habitat": {"path": str(path)}}, False
    if suffix in (".glb", ".usd", ".usda", ".usdc"):
        return {"habitat": {"path": text}}, False
    raise ValueError(
        f"unknown scene {text!r}; use a name from --list or a .glb/.usd/.yaml path"
    )


def _resolve_robot(value: str | Path) -> Dict[str, Any]:
    """Load a robot profile by name or YAML path."""
    text = str(value).strip()
    if not text:
        raise ValueError("--robot must not be empty")
    path = _existing_path(text)
    if path is None:
        named = _config_root() / "robots" / f"{text}.yaml"
        path = named if named.is_file() else None
    if path is None:
        raise ValueError(
            f"unknown robot {text!r}; use a name from --list or a robot YAML path"
        )
    return Config.read(path)


def _normalize_sensors(
    values: str | Sequence[str],
) -> Tuple[Set[str], bool]:
    """Expand comma/space-separated names and presets.

    Returns:
        ``(selected, strict)``.  ``strict`` is false for ``all`` so sensors
        absent from the chosen base config are simply skipped.
    """
    if isinstance(values, str):
        values = [values]
    tokens: list[str] = []
    for value in values:
        tokens.extend(
            part for part in re.split(r"[\s,]+", str(value).strip()) if part
        )
    if not tokens:
        raise ValueError("--sensors requires at least one sensor name")

    lowered = [token.lower() for token in tokens]
    if "none" in lowered:
        if len(lowered) != 1:
            raise ValueError("--sensors none cannot be combined with other sensors")
        return set(), True
    if "all" in lowered:
        return set(SENSOR_KEYS), False

    selected: Set[str] = set()
    for token in lowered:
        if token in SENSOR_PRESETS:
            selected.update(SENSOR_PRESETS[token] or ())
        elif token in SENSOR_ALIASES:
            selected.add(SENSOR_ALIASES[token])
        elif token in SENSOR_KEYS:
            selected.add(token)
        else:
            known = ", ".join(SENSOR_KEYS)
            raise ValueError(f"unknown sensor {token!r}; known: {known}")
    return selected, True


def _set_enabled(sensors: Dict[str, Any], key: str, enabled: bool) -> bool:
    """Set ``enabled`` on a sensor block and report whether it exists."""
    block = sensors.get(key)
    if not isinstance(block, Mapping):
        return False
    updated = dict(block)
    updated["enabled"] = enabled
    sensors[key] = updated
    return True


def _sensor_support(habitat: Mapping[str, Any]) -> dict[str, Any]:
    """Build the capability matrix for the robot currently in ``habitat``.

    Args:
        habitat: ``habitat`` mapping after the robot overlay was merged.

    Returns:
        ``sensor key -> SensorSupport``; empty when the robot is unusable, so
        the later validation pass reports the real problem.
    """
    from autosim.robot.profiles import apply_profile
    from autosim.robot.urdf import UrdfModel
    from autosim.runtime.capabilities import support_matrix

    robot = habitat.get("robot")
    if not isinstance(robot, Mapping):
        return {}
    resolved = apply_profile(robot)
    model = None
    urdf = str(resolved.get("urdf") or "").strip()
    if urdf:
        try:
            model = UrdfModel.load(urdf)
        except (OSError, ValueError):
            model = None
    backend = str(habitat.get("backend", "habitat")).strip().lower()
    return support_matrix(resolved, backend, model)


def _apply_sensor_selection(
    sensors: Dict[str, Any],
    selected: Set[str],
    strict: bool,
    support: Mapping[str, Any] | None = None,
    defined: Set[str] | None = None,
) -> None:
    """Enable exactly the selected sensor blocks.

    Args:
        sensors: ``habitat.sensors`` mapping, mutated in place.
        selected: Canonical sensor keys to turn on.
        strict: When true, an unavailable sensor is an error; ``--sensors all``
            passes false so a robot simply gets the sensors it can provide.
        support: Optional capability matrix from
            :func:`autosim.runtime.capabilities.support_matrix`.
        defined: Keys the config/robot actually declared, before stubs were
            filled.  ``--sensors all`` only turns on stub blocks that work
            without robot-specific calibration.

    Raises:
        ValueError: A requested sensor is unsupported or undefined.
    """
    missing: list[str] = []
    unsupported: list[str] = []
    declared = set(defined or ())
    # A stub carries generic defaults; these sensors still work from them.
    stub_safe = {"camera", "lidar_2d", "lidar_3d", "imu", "odom", "chase", "topdown"}
    for key in SENSOR_KEYS:
        if key == "cameras":
            continue
        wanted = key in selected
        if wanted and not strict and key not in declared and key not in stub_safe:
            # ``all`` on a robot that never declared this block: skip it rather
            # than inventing frames that are not in the URDF.
            wanted = False
        if key == "panorama" and wanted and not ({"cameras", "surround"} & declared):
            wanted = False
        verdict = (support or {}).get(key)
        if wanted and verdict is not None and not verdict.supported:
            if strict:
                unsupported.append(f"{key} ({verdict.reason})")
            # ``--sensors all`` keeps the block off instead of failing.
            wanted = False
        if _set_enabled(sensors, key, wanted):
            continue
        if wanted and strict:
            missing.append(key)

    if "cameras" in selected:
        cameras = sensors.get("cameras")
        surround = sensors.get("surround")
        verdict = (support or {}).get("cameras")
        if verdict is not None and not verdict.supported:
            if strict:
                unsupported.append(f"cameras ({verdict.reason})")
            sensors.pop("cameras", None)
            sensors.pop("surround", None)
        elif isinstance(cameras, Mapping) and cameras:
            # A bare camera table is enabled by presence.  A legacy wrapper
            # may carry an explicit enabled flag.
            if "enabled" in cameras:
                updated = dict(cameras)
                updated["enabled"] = True
                sensors["cameras"] = updated
        elif isinstance(surround, Mapping):
            updated = dict(surround)
            updated["enabled"] = True
            sensors["surround"] = updated
        elif strict:
            missing.append("cameras")
    else:
        sensors.pop("cameras", None)
        sensors.pop("surround", None)

    if unsupported:
        names = "; ".join(unsupported)
        raise ValueError(f"sensor(s) not available for this robot: {names}")
    if missing:
        names = ", ".join(missing)
        raise ValueError(
            f"sensor(s) not defined in this config: {names}; "
            "choose a robot/config that provides them or use --sensors all"
        )


def compose_config(
    base: str | Path | Mapping[str, Any],
    *,
    scene: str | Path | None = None,
    robot: str | Path | None = None,
    sensors: str | Sequence[str] | None = None,
) -> Config:
    """Compose and validate a run configuration.

    The base YAML remains the source of backend, map, dataset, and policy
    settings.  ``scene`` overlays a scene fragment or asset path, ``robot``
    overlays the robot and sensor blocks from a profile, and ``sensors``
    enables exactly the requested sensor blocks.

    Args:
        base: Base YAML path or mapping.
        scene: Scene name, scene YAML, GLB/USD path, or ``empty``.
        robot: Robot profile name or robot YAML path.
        sensors: Comma/space-separated sensor names or a preset.

    Returns:
        Validated configuration.

    Raises:
        ValueError: Unknown selector, missing sensor block, or validation error.
    """
    data = Config.read(base)
    empty_scene = False
    if scene is not None:
        scene_patch, empty_scene = _resolve_scene(scene)
        data = Config._deep_merge(data, scene_patch)

    robot_data: Dict[str, Any] | None = None
    if robot is not None:
        robot_data = _resolve_robot(robot)
        base_habitat = data.get("habitat")
        base_robot = (
            base_habitat.get("robot")
            if isinstance(base_habitat, Mapping)
            else None
        )
        if isinstance(base_robot, dict):
            for key in PROFILE_SPECIFIC_ROBOT_KEYS:
                base_robot.pop(key, None)
        habitat = robot_data.get("habitat")
        habitat = habitat if isinstance(habitat, Mapping) else {}
        patch: Dict[str, Any] = {}
        for key in ("robot", "sensors"):
            block = habitat.get(key)
            if isinstance(block, Mapping):
                patch[key] = block
        if empty_scene:
            for key in ("scenario", "render", "spawn"):
                if key in habitat:
                    patch[key] = habitat[key]
        data = Config._deep_merge(data, {"habitat": patch})

    if empty_scene and robot_data is None:
        base_robot = Config.read(_config_root() / "robots" / "_base.yaml")
        habitat = base_robot.get("habitat")
        habitat = habitat if isinstance(habitat, Mapping) else {}
        patch = {
            key: habitat[key]
            for key in ("scenario", "render", "spawn")
            if key in habitat
        }
        data = Config._deep_merge(data, {"habitat": patch})

    declared: Set[str] = set()
    habitat_block = data.get("habitat")
    if isinstance(habitat_block, Mapping):
        sensor_table = habitat_block.get("sensors")
        if isinstance(sensor_table, Mapping):
            declared = {str(key) for key in sensor_table}

    # Stub every sensor block before the selection runs so ``--sensors all``
    # can turn on a block the base config never mentioned.
    Config.fill_omitted_sensors(data)

    if sensors is not None:
        selected, strict = _normalize_sensors(sensors)
        habitat = data.get("habitat")
        if not isinstance(habitat, dict):
            raise ValueError("habitat must be a mapping")
        sensor_block = habitat.get("sensors")
        if not isinstance(sensor_block, dict):
            raise ValueError("habitat.sensors must be a mapping")
        _apply_sensor_selection(
            sensor_block,
            selected,
            strict,
            support=_sensor_support(habitat),
            defined=declared,
        )

    Config.validate(data)
    return Config(data)
