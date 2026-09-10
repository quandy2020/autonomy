# Velodyne（3D）

机械式多线激光（以 **VLP-16** 为主）。纯 UDP 收包，不依赖厂商闭源 SDK。

## 简介

- **YAML 键**：`lidar_3d`
- **Backend**：`velodyne`（别名 `udp`）
- **源码**：`autodriver/lidar/velodyne/`
- **校准**：`config/params/VLP16_calibration.yaml`

## 管线

```
UDP → PacketQueue → 方位角切帧 → Convert → PointCloud2
                 ↘ optional LidarPacketScan
                 ↘ optional MotionCompensator
```

点云字段：`x,y,z,intensity,timestamp`（`point_step=24`）。

## 配置

```yaml
lidar_3d:
  - name: vlp16
    enable: true
    channel: /lidar/vlp16/points
    backend: velodyne
    fps: 10
    params:
      data_port: 2368
      packets_per_scan: 75
      model: VLP-16
      frame_id: velodyne
      source_type: online
      use_azimuth_cut: true
      calibration_path: params/VLP16_calibration.yaml   # 相对 config/
```

| 参数 | 说明 |
|---|---|
| `data_port` | 默认 2368 |
| `use_azimuth_cut` | 按方位角切帧（默认 true） |
| `enable_compensator` | 运动补偿；配合顶层 `compensator.pose_channel` |
| `source_type: raw_packet` | 回放：`PushRawPacket` / `PushScan` |

## 使用要点

1. 主机与雷达同网段；防火墙放行 UDP `data_port`。
2. 外参示例：`config/params/lidar_vlp16_extrinsics.yaml`。
3. 扩展其它 Velodyne 型号：补校准表即可，一般不必改 backend 名。

## 相关

- [配置 · lidar_3d](../guide/configuration.md#lidar_3d)
- [后端 · Velodyne](../guide/backends.md#velodyne-lidar_3d)
