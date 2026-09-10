# Hesai（3D）

当前：**PandarXT / XT32**。backend `hesai`（alias `pandar`）。

| | |
|---|---|
| 源码 | `autodriver/lidar/hesai/` |
| params | `config/lidar/hesai/xt32.yaml` |
| 校准 | `config/lidar/hesai/xt32_calibration.yaml`（仰角**度**） |
| 包 | 1080B；距离单位 4mm |

管线同 Velodyne（UDP→队列→切帧→Convert）。

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

未覆盖 XT32M2X 等其它包格式。勿与 Velodyne 校准文件混用（度/rad）。
