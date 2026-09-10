# Livox（3D）

链官方 C SDK，不拷 `livox_ros_driver*`。`backend: livox`；按 `model`/`sdk` 选代。

| 代 | 安装 | 机型 |
|---|---|---|
| SDK2 | `scripts/install_livox_sdk2.sh` | HAP、Mid-360、Mid360s、Avia2 |
| SDK1 | `scripts/install_livox_sdk.sh` | Mid-40/70、Horizon、Avia、Tele |

源码：`autodriver/lidar/livox/`。CMake：`AUTODRIVER_WITH_LIVOX` → `HAVE_LIVOX_SDK2`/`SDK1`。

## params_file（`config/lidar/livox/`）

| 文件 | 代 | 机型 |
|---|---|---|
| `mid360.yaml` / `mid360s.yaml` / `hap.yaml` / `avia2.yaml` | 2 | Mid-360 / Mid360s / HAP / Avia2 |
| `mid40.yaml` / `mid70.yaml` / `horizon.yaml` / `avia.yaml` / `tele.yaml` | 1 | Mid-40/70 / Horizon / Avia / Tele |

```yaml
lidar_3d:
  - name: mid360
    enable: true
    channel: /lidar/mid360/points
    backend: livox
    params_file: lidar/livox/mid360.yaml
```

SDK2 文件含：`model`、`sdk: "2"`、`host_ip`、`lidar_ip`、`pcl_data_type`、`publish_freq`、`frame_id`；可选 `config_path` 覆盖自动 JSON。  
SDK1：`broadcast_code` / `broadcast_codes`（`,` 或 `&`）；空=接受全部非 Hub。

## 参数速查

| 键 | 代 | 说明 |
|---|---|---|
| `sdk` | 双 | 强制 `1`/`sdk1` 或 `2`/`sdk2` |
| `host_ip`/`lidar_ip` | 2 | 同网段 |
| `config_path` | 2 | 官方 JSON |
| `broadcast_code` | 1 | 白名单 |
| `publish_freq`/`fps` | 双 | 组帧 Hz，默认 10 |

点云与 Velodyne 同布局（`point_step=24`）。进程内各代 SDK 单实例。