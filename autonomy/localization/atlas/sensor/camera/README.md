# Atlas camera models (`sensor/camera`)

Unified interface: `sensor::GeometricCamera` (`Project` / `Unproject`).

Create via `camera::CameraFactory` by name or YAML.

## Supported models (common SLAM / calibration stack)

| Atlas `model` | Aliases | Toolchain | Parameters |
|---------------|---------|-----------|------------|
| `pinhole` | `ideal` | ORB-SLAM3 Pinhole / Kalibr pinhole-none | fx fy cx cy |
| `radtan` | `opencv` `brown` `perspective` | OpenCV / Kalibr pinhole-radtan | + k1 k2 p1 p2 [k3] |
| `kannala_brandt` | `fisheye` `equi` `kb8` | OpenCV fisheye / ORB KB8 / Kalibr pinhole-equi | + k1..k4 |
| `fov` | | Kalibr pinhole-fov (Devernay) | + w |
| `ucm` | `mei` `omni` | Kalibr omni / Mei UCM | + xi |
| `eucm` | | Kalibr/Basalt EUCM | + alpha beta |
| `double_sphere` | `ds` | Kalibr/Basalt Double Sphere | + xi alpha |
| `equirectangular` | `equirect` `panorama` | 360° panorama | fx≈W/2π fy≈H/π |
| `radial_division` | `division` | Fitzgibbon / OpenVSLAM | + k |

## YAML example

```yaml
Camera:
  model: eucm
  fx: 458.0
  fy: 457.0
  cx: 367.0
  cy: 248.0
  alpha: 0.6
  beta: 1.1
  width: 752
  height: 480
```

```cpp
auto cam = sensor::camera::CameraFactory::CreateFromYaml(node);
Vec2 uv = cam->Project(Vec3(0.1, 0.0, 1.0));
Vec3 ray = cam->Unproject(uv, 1.0);
```

## Notes

- "Common" means projection models covered by visual SLAM / calibration stacks
  (Kalibr, OpenCV, Basalt, ORB-SLAM3, OpenVSLAM), not consumer lens brand lists.
- Scaramuzza OCamCalib polynomial is not implemented separately; use
  `ucm` / `eucm` / `double_sphere` for similar wide-angle / fisheye coverage.
- ORB aliases: `sensor::PinholeCamera`, `sensor::KannalaBrandt8`.
