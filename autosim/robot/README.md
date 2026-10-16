# Robot assets (autosim)

Per-robot package layout: URDF + optional meshes under one folder.

| Directory | Model |
|-----------|--------|
| `turtlebot3_burger/` | ROBOTIS TurtleBot3 Burger |
| `husky/` | Clearpath Husky A200 |
| `beni/` | Open Beni wheeled-biped (RGB-D) |

```
robot/<name>/
  <name>.urdf          # primary kinematic description
  *_bev.urdf           # optional variants (e.g. surround cameras)
  meshes/              # STL / DAE / OBJ referenced by the URDF
```

## Frames (common)

- `base_link` — body
- `laser_link` / `base_scan` — 2D lidar
- `imu_link` — IMU

## Sources

- TurtleBot3: [turtlebot3_description](https://github.com/ROBOTIS-GIT/turtlebot3/tree/humble/turtlebot3_description)
- Husky: [husky_description](https://github.com/husky/husky) (A200 approximate dimensions)

## Config

```yaml
habitat:
  robot:
    profile: beni   # or turtlebot3_burger / husky
    # urdf: robot/beni/beni.urdf
```

Paths are relative to the `autosim/` project root. Visual meshes live under each robot’s `meshes/` (ROBOTIS STL / Clearpath DAE / Open Beni OBJ). Collision geometry stays primitive for simulation speed. Autoviz resolves mesh paths relative to the URDF directory.

Motion is always planar `/cmd_vel` (unicycle). BENI’s native RL balancer stays in Open Beni; autosim uses wheel track ≈ 0.16 m.