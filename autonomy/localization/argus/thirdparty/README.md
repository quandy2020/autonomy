# Argus third-party (git submodules)

| Path | Upstream | Role |
|------|----------|------|
| `gtsam/` | [borglab/gtsam](https://github.com/borglab/gtsam) | Pose-graph / Voxel PGO (`ARGUS_USE_GTSAM`) |

## Init

```bash
git submodule update --init --recursive \
  autonomy/localization/argus/thirdparty/gtsam
```

## CMake

| Option | Default | Effect |
|--------|---------|--------|
| `ARGUS_USE_GTSAM` | `ON` | Prefer `find_package(GTSAM)` |
| `ARGUS_BUILD_VENDORED_GTSAM` | `OFF` | Build from `thirdparty/gtsam` when system GTSAM missing |
