# Lidar 厂商参数（`config/lidar/<vendor>/`）

主配置：`config/autodriver_hardware.yaml`。用 `params_file:` 引用本目录下型号文件；条目内 `params:` 覆盖文件。

| 目录 | backend | 内容 |
|---|---|---|
| `slamtec/` | `rplidar` | A1/A2/A3 串口 params |
| `velodyne/` | `velodyne` | `vlp16.yaml` + 校准/外参例 |
| `hesai/` | `hesai` | `xt32.yaml` + 校准 |
| `livox/` | `livox` | Mid-360 / HAP / Mid-40 … |

`calibration_path` / `extrinsic_path` 为**文件系统路径**（建议绝对路径，或相对进程 cwd）。包内示例：

- `$AUTODRIVER_PATH/config/lidar/velodyne/vlp16_calibration.yaml`
- `$AUTODRIVER_PATH/config/lidar/hesai/xt32_calibration.yaml`

无 `calibration_path` 时驱动使用内置默认仰角表。
