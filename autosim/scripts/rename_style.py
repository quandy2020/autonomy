#!/usr/bin/env python3
# Copyright 2026 The Openbot Authors (duyongquan)
"""One-shot identifier rename for autosim style pass (token-boundary)."""

from __future__ import annotations

import re
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1] / "autosim"

# Longer keys first. Do not touch dunders (__init__, __all__, …).
RENAMES: list[tuple[str, str]] = [
    # loggers / module flags
    ("_LOG", "logger"),
    ("_ATEXIT_REGISTERED", "atexit_registered"),
    ("_BUILTINS_LOADED", "builtins_loaded"),
    ("_APP", "kit_app"),
    ("_DISPLAY_PRIMVARS", "DISPLAY_PRIMVARS"),
    ("_MAX_DEPTH_M", "MAX_DEPTH_M"),
    ("_DEFAULTS", "DEFAULTS"),
    # runtime worker
    ("_queue", "task_queue"),
    ("_thread", "worker_thread"),
    ("_stop", "stop_event"),
    ("_run", "drain_queue"),
    # runtime loop
    ("_last_scan", "last_scan"),
    ("_last_rgb", "last_rgb"),
    ("_last_action", "last_action"),
    ("_sensor_worker", "sensor_worker"),
    ("_sensors_block", "sensors_block"),
    ("_warmup_map", "warmup_map"),
    ("_resolve_camera_rig", "resolve_camera_rig"),
    ("_footprint_in_base_link", "footprint_in_base_link"),
    ("_footprint_urdf", "footprint_urdf"),
    ("_map_sample_origin", "map_sample_origin"),
    ("_urdf_for_footprint", "urdf_for_footprint"),
    # policy
    ("_load_session", "load_session"),
    ("_last_cmd", "last_command"),
    ("_session", "session"),
    ("_elapsed", "elapsed"),
    ("_period", "period"),
    ("_encode", "build_feeds"),
    # dataset collector / lerobot
    ("_episodes_written", "episodes_written"),
    ("_flush_episode", "flush_episode"),
    ("_flush_meta", "flush_meta"),
    ("_ensure_dirs", "ensure_directories"),
    ("_load_existing", "load_existing_meta"),
    ("_task_index", "resolve_task_index"),
    ("_chunk_index", "resolve_chunk_index"),
    ("_write_parquet", "write_parquet"),
    ("_write_video", "write_video"),
    ("_encode_mp4", "encode_mp4"),
    ("_save_png", "save_png"),
    ("_image_fallback", "image_fallback"),
    ("_global_index", "global_index"),
    ("_episodes", "episodes"),
    ("_tasks", "tasks"),
    ("_episode", "episode"),
    ("_writer", "writer"),
    ("_closed", "closed"),
    # registry
    ("_ensure_builtins", "ensure_builtins"),
    # fake world
    ("_volume", "occupancy_volume"),
    ("_sensor", "ray_sensor"),
    # isaac common
    ("_ensure_robot_prim", "ensure_robot_prim"),
    ("_attach_floorplan_map", "attach_floorplan_map"),
    ("_seed_scenario_geometry", "seed_scenario_geometry"),
    ("_static_laser_ranges", "static_laser_ranges"),
    ("_merge_traffic_ranges", "merge_traffic_ranges"),
    ("_merge_nearer", "merge_nearer_ranges"),
    ("_traffic_usd", "traffic_usd"),
    ("_eye_height", "sensor_height"),
    ("_robot_xform", "robot_xform"),
    ("_physics", "physics"),
    ("_stage", "stage"),
    ("_sensors", "engine_sensors"),
    ("_world", "physics_world"),
    ("_rgb_annotator", "rgb_annotator"),
    ("_depth_annotator", "depth_annotator"),
    ("_lidar_annotator", "lidar_annotator"),
    ("_sanitize_depth", "sanitize_depth"),
    ("_read_depth", "read_depth"),
    ("_read_rgb", "read_rgb"),
    ("_to_hw_rgb", "reshape_rgb"),
    ("_to_hw", "reshape_hw"),
    ("_lidar_xyz", "lidar_points_xyz"),
    ("_lidar_xy", "lidar_points_xy"),
    ("_sync_group", "sync_agent_group"),
    ("_ensure_colliders_usdphysics", "ensure_colliders_usdphysics"),
    ("_dense_floorplan_cloud", "dense_floorplan_cloud"),
    ("_wall_cloud", "wall_point_cloud"),
    ("_points_in_polygon", "points_in_polygon"),
    ("_erode_bool", "erode_boolean_mask"),
    ("_apply_hfov", "apply_horizontal_fov"),
    ("_UsdGeom", "UsdGeom"),
    ("_omni", "omni_module"),
    ("_has", "has_module"),
    ("_ready", "ready"),
    ("_pending", "pending"),
    ("_camera", "camera_prim"),
    # scenario cast / volume / cameras helpers
    ("_fold_batch", "fold_hit_batch"),
    ("_cylinders_batch", "cast_cylinders_batch"),
    ("_capsules_batch", "cast_capsules_batch"),
    ("_tubes_batch", "cast_tubes_batch"),
    ("_spheres_batch", "cast_spheres_batch"),
    ("_fold_rows", "fold_hit_rows"),
    ("_dda_hit", "cast_dda_hit"),
    ("_aabb_enter", "aabb_entry_distance"),
    ("_normalize_rig", "normalize_camera_rig"),
    ("_active_cameras", "active_cameras"),
    ("_pose_of", "camera_pose"),
    ("_vertical_fov", "vertical_fov_degrees"),
    ("_camera_model", "resolve_camera_model"),
    ("_camera_names", "camera_name_set"),
    ("_camera_channels", "camera_channel_pair"),
    ("_abs_delta", "absolute_angle_delta"),
    ("_wrap", "wrap_yaw"),
    ("_mount", "mount_offset"),
    ("_pose", "layout_pose"),
    ("_xyz", "as_xyz"),
]


def rename_source(text: str) -> str:
    # Protect dunder names by temporarily masking.
    dunders: list[str] = []

    def stash(match: re.Match[str]) -> str:
        dunders.append(match.group(0))
        return f"__DUNDER_{len(dunders) - 1}__"

    masked = re.sub(r"__\w+__", stash, text)
    for old, new in RENAMES:
        masked = re.sub(rf"\b{re.escape(old)}\b", new, masked)
    for index, value in enumerate(dunders):
        masked = masked.replace(f"__DUNDER_{index}__", value)
    return masked


def main() -> None:
    changed = 0
    for path in sorted(ROOT.rglob("*.py")):
        if "__pycache__" in path.parts:
            continue
        original = path.read_text(encoding="utf-8")
        updated = rename_source(original)
        if updated != original:
            path.write_text(updated, encoding="utf-8")
            changed += 1
            print(path.relative_to(ROOT.parent))
    print(f"updated {changed} files")


if __name__ == "__main__":
    main()
