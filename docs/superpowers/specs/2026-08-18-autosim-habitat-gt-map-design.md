# Autosim Habitat ground-truth map (design)

**Date:** 2026-08-18  
**Status:** Approved (approach A — panoramic cast_ray)  
**Scope:** Remove Mock; publish global `PointCloud2` + `OccupancyGrid` from Habitat geometry.

## Summary

Autosims opens Habitat only (no mock fallback). When `habitat.map.enabled`, sample a panoramic lidar-style cloud in the `map` frame via `cast_ray`, publish `/map/points`, and project a height slice into `/map` (`map_msgs.OccupancyGrid`).

## Remove Mock

- Delete `use_mock` path and constant-range / green-image fallbacks used as a backend.
- `Simulator.create` always opens Habitat; missing `habitat-sim` or scene open failure → hard error.
- Empty `habitat.path` still allowed (`STAGE_EMPTY_SCENE`) but requires Habitat installed.
- Unit tests inject a fake simulator or skip Habitat-only cases; no production mock backend.

## Config (`habitat.map`)

| Key | Default | Meaning |
|-----|---------|---------|
| `enabled` | `true` | Publish map streams |
| `cloud_channel` | `/map/points` | Global cloud |
| `grid_channel` | `/map` | Occupancy grid |
| `frame` | `map` | `header.frame_id` |
| `rate_hz` | `0.2` | Republish rate; `0` = once at start |
| `range_max` | `30.0` | Cast budget (m) |
| `horizontal` / `vertical` | 720 beams / 32 rings | Spherical FOV like lidar_3d |
| `grid.resolution` | `0.05` | m/cell |
| `grid.occupied_thresh` | `1` | Hits per cell → occupied |
| `grid.z_min` / `z_max` | `0.1` / `1.8` | Height window (map z = Habitat y) |

## Pipeline

1. Origin: planar spawn `(x,y)` → Habitat `(x, eye_y, y)`; `eye_y` from URDF laser height or `0.5`.
2. Dense spherical `cast_ray` → Nx3 points in **map/world** frame.
3. Encode `PointCloud2` → `cloud_channel`.
4. Bin points with `z ∈ [z_min,z_max]` into grid; cell ≥ thresh → `100`, else `-1` (unknown). No free-space carving in v1.
5. `info.origin` = lower-left of AABB padded to resolution; encode `OccupancyGrid` → `grid_channel`.

## Modules

- `autosim/mapping.py` — `Mapping` sample + project
- `messages.encode_occupancy_grid`
- `runner` publish timers; `config` validate; README

## Non-goals

SLAM, free-space ray carving, TF tree, mesh export, dynamic obstacles.

## Self-review

- Channels unique vs existing sensor topics.
- Occupancy uses `map_msgs` (repo convention), not `nav_msgs`.
- Stubs need OccupancyGrid + MapMetaData if missing for tests.
