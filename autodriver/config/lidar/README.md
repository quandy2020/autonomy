# Lidar 厂商参数（`config/lidar/<vendor>/`）

主配置：`config/autodriver_hardware.yaml`。用 `params_file:` 引用本目录下型号文件；条目内 `params:` 覆盖文件。

| 目录 | backend | 内容 |
|---|---|---|
| `slamtec/` | `rplidar` | A1/A2/A3 串口 params |
| `velodyne/` | `velodyne` | `vlp16.yaml` + 校准/外参例 |
| `hesai/` | `hesai` | `xt32.yaml`（自研 UDP）、`jt128.yaml`（官方 SDK）、`sdk.yaml` |
| `livox/` | `livox` | Mid-360 / HAP / Mid-40 … |

`calibration_path` / `extrinsic_path` / `correction_file_path` 为**文件系统路径**（建议绝对路径）。包内示例：

- `$AUTODRIVER_PATH/config/lidar/velodyne/vlp16_calibration.yaml`
- `$AUTODRIVER_PATH/config/lidar/hesai/xt32_calibration.yaml`
- JT128 验证整机配置：`$AUTODRIVER_PATH/config/examples/jt128_verify.yaml`

XT32 无 `calibration_path` 时用内置仰角表。JT128 必须配置角度校正（PTC 或 `correction_file_path`），见 [docs · Hesai](../../docs/source/sensor/lidar/hesai.md)。
