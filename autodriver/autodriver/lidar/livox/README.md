# Livox 3D lidar (`backend: livox`)

Covers the full Livox lineup via system-installed SDKs (same pattern as RPLidar):

| Generation | SDK | Models |
|---|---|---|
| SDK2 | [Livox-SDK2](https://github.com/Livox-SDK/Livox-SDK2) | HAP, Mid-360, Mid360s, Avia2 |
| SDK1 | [Livox-SDK](https://github.com/Livox-SDK/Livox-SDK) | Mid-40, Mid-70, Horizon, Avia, Tele |

Do **not** copy `livox_ros_driver` / `livox_ros_driver2` into the tree — only link the C SDK and publish `sensor_msgs/PointCloud2` on Autolink.

## Install

```bash
./scripts/install_livox_sdk2.sh   # Mid-360 / HAP / …
./scripts/install_livox_sdk.sh    # Mid-40 / Horizon / Avia / …
```

Rebuild with `-DAUTODRIVER_WITH_LIVOX=ON` (default). CMake defines
`AUTODRIVER_HAVE_LIVOX_SDK2` / `AUTODRIVER_HAVE_LIVOX_SDK1` when found.

## Config

```yaml
lidar_3d:
  - name: mid360
    enable: true
    backend: livox
    channel: /lidar/mid360/points
    params_file: lidar/livox/mid360.yaml
```

| Param | SDK | Notes |
|---|---|---|
| `model` | both | Selects SDK when `sdk` unset |
| `sdk` | both | Force `1`/`sdk1` or `2`/`sdk2` |
| `host_ip` / `lidar_ip` | SDK2 | Or pass `config_path` JSON |
| `broadcast_code` | SDK1 | Comma/`&` list; empty = accept all |
| `publish_freq` / `fps` | both | Frame assemble rate (default 10 Hz) |
| `frame_id` | both | Default sensor id |

Vendor templates: `config/lidar/livox/*.yaml`.
