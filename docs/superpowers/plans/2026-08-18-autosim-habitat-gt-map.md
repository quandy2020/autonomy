# Autosim Habitat GT map — Implementation Plan

> **For agentic workers:** Implement task-by-task.

**Goal:** Remove Mock; publish Habitat panoramic `/map/points` + `/map` OccupancyGrid.

**Architecture:** `Mapping` owns spherical cast + grid project; `Simulator` Habitat-only; `Runner` republishes at `rate_hz`.

**Tech Stack:** Python, NumPy, habitat-sim, automsgs map_msgs

---

### Task 1: Remove Mock from Simulator

- [ ] Drop `use_mock`, mock laser/camera/cloud paths
- [ ] `create()` always `open_habitat()`; fix/skip tests

### Task 2: Mapping + OccupancyGrid codec

- [ ] `mapping.py`, `encode_occupancy_grid`, stubs if needed
- [ ] Unit tests with fake cast backend

### Task 3: Config + Runner + README

- [ ] `habitat.map` in default.yaml + validation
- [ ] Wire publish; update README
