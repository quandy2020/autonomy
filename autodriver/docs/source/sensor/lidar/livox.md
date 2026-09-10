# Livox（3D）

非重复扫描固态/混合固态激光。对接官方两代 C SDK（**不** vendoring
`livox_ros_driver`），按 `model` 自动选 SDK。

## 简介

| 代际 | SDK | 机型 |
|---|---|---|
| SDK2 | [Livox-SDK2](https://github.com/Livox-SDK/Livox-SDK2) | HAP、Mid-360、Mid360s、Avia2 |
| SDK1 | [Livox-SDK](https://github.com/Livox-SDK/Livox-SDK) | Mid-40/70、Horizon、Avia、Tele |

- **YAML 键**：`lidar_3d`，`backend: livox`
- **源码**：`autodriver/lidar/livox/`
- **厂商参数**：`config/lidar/livox/*.yaml`

## 依赖

```bash
./scripts/install_livox_sdk2.sh    # Mid-360 / HAP / …
./scripts/install_livox_sdk.sh     # Mid-40 / Horizon / …
```

| CMake | 宏 |
|---|---|
| `AUTODRIVER_WITH_LIVOX=ON` | `AUTODRIVER_HAVE_LIVOX_SDK2` / `SDK1` |

未安装对应 SDK 时该代驱动 `Create`→`nullptr`。

## 配置

**Mid-360（SDK2）**：

```yaml
lidar_3d:
  - name: mid360
    enable: true
    channel: /lidar/mid360/points
    backend: livox
    params_file: lidar/livox/mid360.yaml
```

编辑 `mid360.yaml`：

```yaml
model: Mid-360
sdk: "2"
host_ip: "192.168.1.5"      # 工控机网口 IP
lidar_ip: "192.168.1.12"    # 雷达 IP
publish_freq: 10
frame_id: livox_frame
# config_path: /path/to/MID360_config.json   # 可选，覆盖自动生成 JSON
```

**Mid-40（SDK1）**：

```yaml
params_file: lidar/livox/mid40.yaml
# broadcast_code: "0A..."   # 空=接受全部非 Hub 设备
```

| 参数 | 说明 |
|---|---|
| `sdk` | 强制 `1`/`sdk1` 或 `2`/`sdk2` |
| `publish_freq` / `fps` | 组帧频率（默认 10 Hz） |
| `pcl_data_type` | SDK2 点云类型 |

## 使用要点

1. **网段**：主机 `host_ip` 与雷达同子网；端口见官方 JSON（Mid-360 点云约 56300/56301）。
2. SDK1 / SDK2 **不可混链同一进程多代初始化**（当前各代进程内单实例）。
3. 点云布局与 Velodyne/Hesai 一致，便于后续接运动补偿。

## 相关

- [配置 · lidar_3d](../guide/configuration.md#lidar_3d)
- 驱动 README：`autodriver/lidar/livox/README.md`
