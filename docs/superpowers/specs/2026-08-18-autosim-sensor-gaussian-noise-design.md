# Autosim sensor Gaussian noise (design)

**Date:** 2026-08-18  
**Status:** Approved (approach A)  
**Scope:** Full exteroceptive + proprioceptive observation noise for realistic SLAM/nav bridging.

## Summary

All configured `noise*` fields are **zero-mean Gaussian stddev σ**. `0` disables. Defaults match mid-grade indoor sensors for Cartographer / navigation bring-up.

## Config

| Sensor | Field | Default σ | Unit |
|--------|-------|-----------|------|
| lidar_2d | `noise` | `0.01` | m (range) |
| lidar_3d | `noise` | `0.015` | m (radial) |
| odom | `noise` | `0.02` | m (XY on published pose) |
| imu | `noise.gyro` / `noise.accel` | `0.01` / `0.05` | rad/s · m/s² |
| camera | `noise.depth` | `0.01` | m (per-pixel depth) |

Missing `noise` / nested keys → treat as `0`. Validation: all σ ≥ 0.

## Behavior

- **Lidar 2D/3D / odom:** existing `N(0, σ)` paths; only defaults change.
- **IMU:** after finite-difference estimate, add independent gyro_z / accel_x / accel_y noise (`accel` σ shared for x/y; z gravity term unperturbed except optional future).
- **Camera depth:** `depth += N(0, σ)` then `clip(depth, 0, +inf)`; RGB unchanged.

## Non-goals

Bias random-walk, range-dependent σ, RGB photon noise, correlated IMU bias.

## Self-review

- No placeholders; defaults numeric and documented.
- Odom noise remains white on absolute pose (existing semantics), not integrated wheel slip.
- Accel z kept as constant `9.81` in publisher (no vertical accel model); only x/y get `noise.accel`.
