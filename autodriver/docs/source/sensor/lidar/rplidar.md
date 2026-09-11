# Slamtec RPLidar（2D）

封装 [rplidar_sdk](https://github.com/slamtec/rplidar_sdk)；LaserScan 与 rplidar_ros 对齐。

| | |
|---|---|
| YAML | `lidar_2d` |
| backend | `rplidar`（alias `slamtec`） |
| 源码 | `autodriver/lidar/rplidar/` |
| params | `config/lidar/slamtec/{a1,a2,a3}.yaml` |
| 安装 | `scripts/install_rplidar_sdk.sh`；可选 `create_udev_rules.sh` 生成 `/dev/rplidar` |
| CMake | `AUTODRIVER_WITH_RPLIDAR` + `FindRplidarSDK` |

## 配置

```yaml
lidar_2d:
  - name: front
    enable: true
    channel: /lidar/front/scan
    backend: rplidar
    port: /dev/ttyUSB0   # 或 /dev/rplidar
    params_file: lidar/slamtec/a1.yaml
```

| 型号 | 文件 | baud | 备注 |
|---|---|---|---|
| A1 | a1.yaml | 115200 | |
| A2/A2M8 | a2.yaml | 115200 | A2M7/M12→256000 |
| A3 | a3.yaml | 256000 | `scan_mode: Sensitivity` |

其它 params：`frame_id`、`angle_compensate`、`inverted`、`range_min`、`scan_mode`（空字符串表示 typical）。用户须加入 `dialout` 组。
