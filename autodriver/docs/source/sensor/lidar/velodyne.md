# Velodyne（3D）

UDP 自研协议栈，以 VLP-16 为主；不依赖闭源 SDK。

| | |
|---|---|
| YAML | `lidar_3d` |
| backend | `velodyne`（alias `udp`） |
| 源码 | `autodriver/lidar/velodyne/` |
| params | `config/lidar/velodyne/vlp16.yaml` |
| 校准 | `config/lidar/velodyne/vlp16_calibration.yaml`（rad） |
| 外参例 | `config/lidar/velodyne/vlp16_extrinsics.yaml` |

## 管线

UDP → PacketQueue → 方位角切帧 → Convert → PointCloud2（`point_step=24`）；可选 Scan / 运动补偿。

```yaml
lidar_3d:
  - name: vlp16
    enable: true
    channel: /lidar/vlp16/points
    backend: velodyne
    params_file: lidar/velodyne/vlp16.yaml
    # 覆盖内置仰角时（文件系统路径）：
    # params:
    #   calibration_path: $AUTODRIVER_PATH/config/lidar/velodyne/vlp16_calibration.yaml
```

回放：`PushRawPacket` / `PushScan`。主机须与雷达同网段，并放行 UDP。
