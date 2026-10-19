# Hesai（3D）

当前支持：**PandarXT / XT32**。backend 为 `hesai`（alias `pandar`）。

| | |
|---|---|
| 源码 | `autodriver/lidar/hesai/` |
| params | `config/lidar/hesai/xt32.yaml` |
| 校准 | `config/lidar/hesai/xt32_calibration.yaml`（仰角单位为**度**） |
| 包 | 1080B；距离单位 4mm |

管线与 Velodyne 相同（UDP→队列→切帧→Convert）。XT32 驱动为**自研 UDP 解析**，不依赖官方 SDK。

可选安装官方 [HesaiLidar_SDK_2.0](https://github.com/HesaiTechnology/HesaiLidar_SDK_2.0)（多机型 / PTC / 官方工具）：

```bash
./scripts/install_hesai_lidar_sdk.sh
# PREFIX=/opt/hesai HESAI_WITH_PTCS=1 ./scripts/install_hesai_lidar_sdk.sh
```

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
