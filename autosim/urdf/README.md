# Robot URDFs (autosim)

Self-contained primitive URDFs (no `package://` meshes) for Habitat / CI.

| File | Model |
|------|--------|
| `turtlebot3_burger.urdf` | ROBOTIS TurtleBot3 Burger |
| `husky.urdf` | Clearpath Husky A200 |

## Frames (common)

- `base_link` — body
- `laser_link` / `base_scan` — 2D lidar
- `imu_link` — IMU

## Sources

- TurtleBot3: [turtlebot3_description](https://github.com/ROBOTIS-GIT/turtlebot3/tree/humble/turtlebot3_description)
- Husky: [husky_description](https://github.com/husky/husky) (A200 approximate dimensions)

To use from autosim config:

```yaml
habitat:
  robot:
    urdf: urdf/husky.urdf   # or urdf/turtlebot3_burger.urdf; empty = off
```

Paths are relative to the `autosim/` project root. Habitat loads the URDF as a kinematic articulated object when `habitat-sim` is available; ray origins use `laser_link` / `base_scan` mounts either way.

To use official mesh URDFs, copy meshes under `urdf/meshes/` and update `filename=` paths.
