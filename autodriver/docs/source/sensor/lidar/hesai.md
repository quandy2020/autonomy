# Hesai（3D）

当前支持：**PandarXT / XT32**。backend 为 `hesai`（alias `pandar`）。

| | |
|---|---|
| 源码 | `autodriver/lidar/hesai/` |
| params | `config/lidar/hesai/xt32.yaml` |
| 校准 | `config/lidar/hesai/xt32_calibration.yaml`（仰角单位为**度**） |
| 包 | 1080B；距离单位 4mm |

管线与 Velodyne 相同（UDP→队列→切帧→Convert）。

```yaml
lidar_3d:
  - name: xt32
    enable: true
    channel: /lidar/xt32/points
    backend: hesai
    params_file: lidar/hesai/xt32.yaml
    # params:
    #   calibration_path: $AUTODRIVER_PATH/config/lidar/hesai/xt32_calibration.yaml
```

未覆盖 XT32M2X 等其它包格式。请勿与 Velodyne 校准文件混用（单位分别为度与 rad）。
