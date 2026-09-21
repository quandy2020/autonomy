# Atla2 third-party policy

Prefer **system / workspace packages** over vendoring. Parent
`autonomy/localization` already links SLAM features (Ceres, Eigen, OpenCV)
via `FEATURES slam`.

| Dir | Package | Status |
|-----|---------|--------|
| `ceres/` | Ceres Solver | use workspace / system |
| `eigen/` | Eigen3 | use workspace / system |
| `opencv/` | OpenCV | use workspace / system |
| `sophus/` | Sophus | optional; not required yet |
| `g2o/` | g2o | optional; graph backend may use later |
| `gtsam/` | GTSAM | optional; not required yet |

Only place a pinned fork under these directories when a workspace package is
unavailable or a specific patch is required. Keep `thirdparty/` out of
`autonomy_glob_srcs` library sources (no `*.cpp` here by default).
