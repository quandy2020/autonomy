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

"""Allowlist schema for the ``habitat`` config tree.

Every key a config may set is listed here.  A typo such as ``hfov`` instead of
``hfov_deg`` is reported with its dotted path instead of being silently
ignored, and the message names the closest accepted key.

Schema values:

* ``LEAF`` — any scalar/list value is accepted, no nested keys are checked.
* ``dict`` — a mapping whose own keys are validated against that dict.
* ``{ANY: <schema>}`` — a mapping with arbitrary user-chosen keys (for example
  ``robot.joint_positions`` or ``sensors.cameras``), each value validated
  against ``<schema>``.
"""

from __future__ import annotations

import difflib
from typing import Any, Iterable, Mapping


LEAF: None = None
ANY = "*"


class AnyOf:
    """Accept a scalar/list, or a mapping validated against ``schema``.

    Used where a block allows shorthand: ``range.sensors`` entries may be a
    bare channel string or a mapping with limits.
    """

    def __init__(self, schema: Mapping[str, Any]) -> None:
        self.schema = schema


def _block(*keys: str) -> dict:
    """Return a mapping schema whose keys are all leaves."""
    return {key: LEAF for key in keys}


def _merge(*schemas: Mapping[str, Any]) -> dict:
    """Merge mapping schemas left to right."""
    out: dict = {}
    for schema in schemas:
        out.update(schema)
    return out


_TIMING = _block("latency_sec", "jitter_sec")

_CAMERA = _merge(
    _block(
        "enabled",
        "width",
        "height",
        "hfov_deg",
        "vfov_deg",
        "rgb_channel",
        "depth_channel",
        "info_channel",
        "frame",
        "axes",
        "follow_urdf",
        "rate_hz",
        "range_min",
        "range_max",
        "sensor_height",
        "semantic_enabled",
        "semantic_channel",
        "semantic_ids_channel",
        "depth_points_channel",
        "depth_points_stride",
        "fov_marker_channel",
        "fov_marker_range",
    ),
    {"noise": _block("depth")},
    _TIMING,
)

_LIDAR_2D = _merge(
    _block(
        "enabled",
        "channel",
        "frame",
        "num_beams",
        "angle_min",
        "angle_max",
        "range_min",
        "range_max",
        "noise",
        "rate_hz",
        "backend",
        "rtx_config",
    ),
    _TIMING,
)

_LIDAR_3D = _merge(
    _block(
        "enabled",
        "channel",
        "frame",
        "range_min",
        "range_max",
        "noise",
        "rate_hz",
        "motion_compensate",
    ),
    {
        "horizontal": _block("angle_min", "angle_max", "num_beams"),
        "vertical": _block("angle_min", "angle_max", "num_rings"),
    },
    _TIMING,
)

_IMU = _merge(
    _block("enabled", "channel", "frame", "rate_hz", "gravity"),
    {"noise": _block("gyro", "accel", "gyro_bias", "accel_bias")},
    _TIMING,
)

_ODOM = _block(
    "enabled",
    "channel",
    "frame",
    "child_frame",
    "noise",
    "path_channel",
    "path_max_poses",
    "path_min_distance",
)

_STEREO = _merge(
    _block(
        "enabled",
        "width",
        "height",
        "hfov_deg",
        "rate_hz",
        "baseline_m",
        "left_frame",
        "right_frame",
        "left_channel",
        "right_channel",
        "left_info_channel",
        "right_info_channel",
    ),
    _TIMING,
)

_RANGE_SPEC = _block(
    "channel",
    "min_range",
    "max_range",
    "fov",
    "rate_hz",
    "radiation_type",
    "cone_deg",
    "samples",
)

_RANGE = _merge(
    _block(
        "enabled",
        "min_range",
        "max_range",
        "fov",
        "rate_hz",
        "radiation_type",
        "cone_deg",
        "samples",
    ),
    {"sensors": {ANY: AnyOf(_RANGE_SPEC)}},
    _TIMING,
)

_CHASE = _merge(
    _block(
        "enabled",
        "uuid",
        "width",
        "height",
        "hfov_deg",
        "distance_m",
        "height_m",
        "look_at_height_m",
        "pitch_deg",
        "rate_hz",
        "frame",
        "channel",
        "info_channel",
        "clear_color",
    ),
    _TIMING,
)

_TOPDOWN = _merge(
    _block(
        "enabled",
        "width",
        "height",
        "mode",
        "hfov_deg",
        "height_m",
        "rate_hz",
        "fit_scene",
        "align_navmesh",
        "ortho_scale",
        "ortho_margin",
        "ortho_fill",
        "frame",
        "channel",
        "info_channel",
    ),
    _TIMING,
)

_PANORAMA = _block("enabled", "channel", "frame", "width", "height", "xyz")

_JOINT_STATES = _block("enabled", "channel", "frame")

# One entry of a 4/6-camera surround rig: same knobs as the main camera plus
# the layout overrides used when no URDF frame is given.
_SURROUND_CAMERA = _merge(
    _CAMERA,
    _block("type", "xyz", "yaw", "yaw_deg"),
)

_SENSORS = {
    "camera": _CAMERA,
    "lidar_2d": _LIDAR_2D,
    "lidar_3d": _LIDAR_3D,
    "stereo": _STEREO,
    "range": _RANGE,
    "imu": _IMU,
    "odom": _ODOM,
    "chase": _CHASE,
    "topdown": _TOPDOWN,
    "panorama": _PANORAMA,
    "joint_states": _JOINT_STATES,
    "cameras": {ANY: _SURROUND_CAMERA},
    # Legacy alias kept for configs written before ``cameras`` was a table.
    "surround": _merge(
        _block(
            "enabled",
            "count",
            "type",
            "hfov_deg",
            "width",
            "height",
            "rate_hz",
            "radius_m",
            "sensor_height",
            "frame",
            "topic_prefix",
        ),
        {"mount": _block("radius", "height"), "cameras": {ANY: _SURROUND_CAMERA}},
    ),
    "timing_seed": LEAF,
}

_ROBOT = _merge(
    _block(
        "profile",
        "urdf",
        "usd_robot",
        "usd_prim",
        "instance",
        "fixed_base",
        "block_motion",
        "root_height",
        "base_link_height",
        "wheel_radius",
        "wheel_separation",
        "map_frame",
        "control_hz",
        "watchdog_sec",
        "max_linear",
        "max_angular",
        "max_linear_accel",
        "max_linear_decel",
        "max_angular_accel",
        "cmd_vel",
        "cmd_vel_type",
        "description_channel",
    ),
    {
        "joint_positions": {ANY: LEAF},
        "sensor_frames": {ANY: LEAF},
        "wheel_radii": {ANY: LEAF},
        "wheel_separations": {ANY: LEAF},
        "joint_command": _block(
            "enabled", "channel", "timeout_sec", "max_step", "mode"
        ),
        "truth": _block("enabled", "channel", "odometry_channel"),
        "tf": _block(
            "enabled", "channel", "static_channel", "publish_map_odom"
        ),
        "clock": _block("enabled", "channel"),
        "footprint": _block("enabled", "channel", "frame", "points"),
        "grounding": _block("enabled", "mode", "clearance_m", "update_hz"),
        "status": _block("enabled", "channel", "rate_hz"),
    },
)

_SCENARIO = {
    "enabled": LEAF,
    "type": LEAF,
    "seed": LEAF,
    "x_length": LEAF,
    "y_length": LEAF,
    "z_length": LEAF,
    "resolution": LEAF,
    "spawn_radius": LEAF,
    "spawn_height": LEAF,
    "add_wall_x": LEAF,
    "add_wall_y": LEAF,
    "room_number": LEAF,
    "width_min": LEAF,
    "width_max": LEAF,
    "height_min": LEAF,
    "height_max": LEAF,
    "max_windows": LEAF,
    "maze_type": LEAF,
    "num_nodes": LEAF,
    "connectivity": LEAF,
    "road_width": LEAF,
    "road_radius": LEAF,
    "tree_dist": LEAF,
    "attenuation": LEAF,
    "complexity": LEAF,
    "fill": LEAF,
    "fractal": LEAF,
    "obstacle_number": LEAF,
    "obstacle": _block(
        "enabled",
        "count",
        "height",
        "radius",
        "seed",
        "speed_min",
        "speed_max",
        "z_obstacle_min",
        "z_obstacle_max",
        "island_radius",
    ),
    "pedestrian": _merge(
        _block(
            "enabled",
            "count",
            "agents",
            "behavior",
            "seed",
            "height",
            "radius",
            "laser_radius",
            "speed",
            "goal_tolerance",
            "stuck_steps",
            "wander",
            "wander_min_dist",
            "spread_spawn",
            "spread_min_dist",
            "social",
            "social_force_factor",
            "social_margin",
            "A",
            "B",
            "max_vel",
            "goal_radius",
            "init_pose",
            "goals",
            "cyclic_goals",
        ),
        {"behavior_configuration": LEAF},
    ),
}

_MAP = {
    "enabled": LEAF,
    "publish": LEAF,
    "frame": LEAF,
    "rate_hz": LEAF,
    "free_doors": LEAF,
    "wall_thickness_m": LEAF,
    "door_opening_inflate_m": LEAF,
    "mesh_inflate_m": LEAF,
    "height_margin_m": LEAF,
    "padding_m": LEAF,
    "robot_clearance_m": LEAF,
    "robot_height_m": LEAF,
    "grid": _block("channel", "resolution", "z_min", "z_max"),
    "ply": _merge(
        _block(
            "channel",
            "source",
            "file",
            "stride",
            "mesh_stride",
            "max_points",
            "range_max",
            "surface_spacing_m",
        ),
        {
            "horizontal": _block("angle_min", "angle_max", "num_beams"),
            "vertical": _block("angle_min", "angle_max", "num_rings"),
        },
    ),
}

_DATASET = _block(
    "enabled",
    "root",
    "repo_id",
    "fps",
    "task",
    "episode_frames",
    "include_image",
    "include_laser",
    "include_people",
    "include_joints",
    "include_imu",
    "include_depth",
    "include_actions",
    "cameras",
    "video_codec",
    "chunks_size",
    "window_size_min",
    "window_size_max",
    "record_path",
    "max_episodes",
    "strict",
)

_HUMANOIDS = _block(
    "enabled",
    "root",
    "names",
    "visual",
    "sync_hz",
    "snap_navmesh",
    "skin_map",
)

_HUNAV = _block(
    "publish_agents",
    "service",
    "agents_channel",
    "config",
    "behavior_configuration",
    "compute_agents",
    "remote_compute",
    "motion_fps",
)

_ISAAC_PHYSICS = _block(
    "enabled",
    "mode",
    "dt",
    "add_ground",
    "block_motion",
    "ensure_scene_colliders",
    "freeze_scene",
    "robot_height",
    "robot_radius",
)

_ISAAC = {
    "headless": LEAF,
    "physics": _ISAAC_PHYSICS,
    "domain_random": _block(
        "enabled",
        "light_color_jitter",
        "light_intensity",
        "shuffle_characters",
    ),
    "traffic": _block(
        "anim_walk_mps",
        "animation_root",
        "character_root",
        "character_yaw_offset_deg",
        "character_z",
        "collision_proxy",
        "obstacle_visual",
        "pedestrian_anim",
        "pedestrian_visual",
        "sync_hz",
        "usd_sync",
    ),
    "wheeled": _block("track_width", "wheel_radius"),
}

_POLICY = _block(
    "enabled",
    "action_space",
    "drive_from_wheels",
    "exclusive",
    "laser_bins",
    "max_vx",
    "max_wz",
    "rate_hz",
    "use_laser",
)

HABITAT_SCHEMA: dict = {
    "backend": LEAF,
    "mode": LEAF,
    "path": LEAF,
    "dataset_config": LEAF,
    "scene_id": LEAF,
    "gpu": LEAF,
    "spawn": LEAF,
    "scenario": _SCENARIO,
    "render": _block("enable_hbao", "clear_color"),
    "robot": _ROBOT,
    "sensors": _SENSORS,
    "map": _MAP,
    "dataset": _DATASET,
    "record": _DATASET,
    "hunav": _HUNAV,
    "humanoids": _HUMANOIDS,
    "isaac": _ISAAC,
    "physics": _ISAAC_PHYSICS,
    "usd": _block("world", "robot", "robot_prim", "usd_root"),
    "policy": _POLICY,
}

ROOT_SCHEMA: dict = {
    "habitat": HABITAT_SCHEMA,
    # ``extends`` is consumed while merging and never reaches validation; it is
    # listed so a config that keeps it after a manual merge still validates.
    "extends": LEAF,
}


def _format_path(path: str, key: Any) -> str:
    """Join a dotted path with a key, quoting keys that need it."""
    text = str(key)
    if not text or any(char in text for char in ". "):
        text = repr(text)
    return f"{path}.{text}" if path else text


def _suggest(key: Any, known: Iterable[str]) -> str:
    """Return a ``; did you mean ...?`` hint when a close key exists."""
    options = [str(item) for item in known if str(item) != ANY]
    matches = difflib.get_close_matches(str(key), options, n=1, cutoff=0.6)
    if not matches:
        return ""
    return f" (did you mean {matches[0]!r}?)"


def validate_keys(
    node: Any,
    schema: Mapping[str, Any],
    path: str = "",
    errors: list[str] | None = None,
) -> list[str]:
    """Collect unknown-key errors for ``node`` against ``schema``.

    Args:
        node: Config value to inspect.
        schema: Mapping schema (see module docstring).
        path: Dotted path of ``node`` for messages.
        errors: Accumulator; a fresh list is created when omitted.

    Returns:
        The accumulated error strings (empty when the tree matches).
    """
    if errors is None:
        errors = []
    if not isinstance(node, Mapping):
        return errors

    wildcard = schema.get(ANY)
    for key, value in node.items():
        where = _format_path(path, key)
        if key in schema:
            child = schema[key]
        elif wildcard is not None:
            child = wildcard
        elif ANY in schema:
            # ``{ANY: LEAF}`` accepts any key with any value.
            continue
        else:
            errors.append(f"unknown config key: {where}{_suggest(key, schema)}")
            continue
        if isinstance(child, AnyOf):
            if isinstance(value, Mapping):
                validate_keys(value, child.schema, where, errors)
            continue
        if isinstance(child, Mapping):
            if not isinstance(value, Mapping):
                errors.append(f"{where} must be a mapping")
                continue
            validate_keys(value, child, where, errors)
    return errors


def check(data: Mapping[str, Any]) -> None:
    """Validate the whole config tree, raising on the first batch of errors.

    Args:
        data: Root config mapping.

    Raises:
        ValueError: Unknown keys or wrong node types.
    """
    errors = validate_keys(data, ROOT_SCHEMA)
    if not errors:
        return
    shown = errors[:8]
    suffix = "" if len(errors) <= len(shown) else f" (+{len(errors) - len(shown)} more)"
    raise ValueError("invalid config: " + "; ".join(shown) + suffix)
