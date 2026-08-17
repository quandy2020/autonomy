# Autosim sensor Gaussian noise — Implementation Plan

> **For agentic workers:** Implement task-by-task. Steps use checkbox syntax.

**Goal:** Wire realistic zero-mean Gaussian σ for lidar, odom, IMU, and camera depth.

**Architecture:** Scalar / nested `noise` in YAML; `Sensors` / `Robot` apply `rng.normal`; `Config` validates σ ≥ 0.

**Tech Stack:** Python, NumPy, PyYAML, pytest

---

### Task 1: Config defaults + validation

**Files:** `autosim/config/default.yaml`, `autosim/autosim/config.py`, tests

- [ ] Set lidar/odom/imu/camera noise defaults per design
- [ ] Validate σ ≥ 0 (and imu/camera nested maps)
- [ ] Tests for defaults and reject negative σ

### Task 2: IMU + depth noise application

**Files:** `autosim/autosim/robot.py`, `autosim/autosim/sensors.py`, `autosim/autosim/runner.py`

- [ ] Robot: `gyro_noise` / `accel_noise`; apply in `update_inertial`
- [ ] Sensors: `depth_noise`; apply in `sample_camera`
- [ ] Runner: pass config into Robot / Sensors
- [ ] Unit tests with fixed seed

### Task 3: Docs

**Files:** `autosim/README.md`

- [ ] Note that `noise` fields are Gaussian σ
