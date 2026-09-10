# Hesai（3D）

速腾聚创机械式激光；当前实现以 **PandarXT / XT32** 为主。

## 简介

- **YAML 键**：`lidar_3d`
- **Backend**：`hesai`（别名 `pandar`）
- **源码**：`autodriver/lidar/hesai/`
- **校准**：`config/params/XT32_calibration.yaml`（仰角为**度**）

## 管线

与 Velodyne 相同：UDP → 队列 → 方位角切帧 → Convert → `PointCloud2`
（`point_step=24`）。XT32 包长 1080 字节，距离单位 4 mm。

## 配置

```yaml
lidar_3d:
  - name: xt32
    enable: true
    channel: /lidar/xt32/points
    backend: hesai
    params:
      data_port: 2368
      packets_per_scan: 180
      model: XT32
      frame_id: hesai
      source_type: online
      use_azimuth_cut: true
      calibration_path: params/XT32_calibration.yaml
```

| 参数 | 默认 / 说明 |
|---|---|
| `packets_per_scan` | 180（上限） |
| `model` | `XT32` / `PandarXT` |
| `enable_compensator` | 同 Velodyne |

**未覆盖**：XT32M2X 等其它包格式。

## 使用要点

1. 确认雷达输出为 XT32 兼容包；否则 Convert 会跳过非法包。
2. 校准 YAML 与 Velodyne 不同（度 vs rad），勿混用。

## 相关

- [配置 · lidar_3d](../guide/configuration.md#lidar_3d)
- 驱动 README：`autodriver/lidar/hesai/README.md`
