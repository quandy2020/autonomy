# Slamtec RPLidar（2D）

A1 / A2 / A3 等 2D 激光。封装官方 [rplidar_sdk](https://github.com/slamtec/rplidar_sdk)，
发布 `sensor_msgs/LaserScan`（对齐 rplidar_ros 的 `publish_scan` 逻辑）。

## 简介

- **YAML 键**：`lidar_2d`
- **Backend**：`rplidar`（别名 `slamtec`）
- **源码**：`autodriver/lidar/rplidar/`
- **厂商参数**：`config/lidar/slamtec/{a1,a2,a3}.yaml`

## 依赖

```bash
./scripts/install_rplidar_sdk.sh          # 从 GitHub 拉取并装到 /usr/local
./scripts/create_udev_rules.sh            # 可选 → /dev/rplidar
```

| 项 | 说明 |
|---|---|
| CMake | `AUTODRIVER_WITH_RPLIDAR=ON` |
| 库 | `libsl_lidar_sdk`（`FindRplidarSDK.cmake`） |
| 串口 | `/dev/ttyUSB*` 或 `/dev/rplidar`；用户属 `dialout` |

## 配置

```yaml
lidar_2d:
  - name: front
    enable: true
    channel: /lidar/front/scan
    backend: rplidar
    port: /dev/ttyUSB0          # 或 /dev/rplidar
    params_file: lidar/slamtec/a1.yaml
```

| 型号 | params_file | 波特率 |
|---|---|---|
| A1 | `a1.yaml` | 115200 |
| A2 / A2M8 | `a2.yaml` | 115200（A2M7/M12→256000） |
| A3 | `a3.yaml` | 256000 + `scan_mode: Sensitivity` |

常用 params：`frame_id`、`angle_compensate`、`inverted`、`scan_mode`、`range_min`。

## 使用要点

1. 先装 SDK 再编 autodriver；未找到 SDK 时 backend stub。
2. A3 务必用 `a3.yaml` 波特率，否则连不上。
3. 与其它串口设备错开 `ttyUSB` 编号，或固定 udev 符号链接。

## 相关

- [配置 · lidar_2d](../guide/configuration.md#lidar_2d)
- 驱动 README：`autodriver/lidar/rplidar/README.md`
