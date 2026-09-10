# Orbbec

Gemini 330 为主。OrbbecSDK；多路共用 hub。

| | |
|---|---|
| 消息 | Image、PointCloud2；板载 IMU |
| backend | `orbbec` |
| 源码 | `autodriver/camera/orbbec/` |
| params | `config/camera/orbbec/gemini_330.yaml` |
| 示例折叠 | `config/examples/orbbec_gemini_330.yaml` |
| CMake | `AUTODRIVER_WITH_ORBBEC` + OrbbecSDK |

## 配置

折叠语法同 RealSense。通道对齐 OrbbecSDK_ROS2：  
`/camera/{color,depth,left_ir,right_ir}/image_raw`；点云 `/camera/depth/points` 或 `depth_registered/points`；IMU `/camera/gyro_accel/sample`。

| 参数 | 建议 |
|---|---|
| `width`/`height`/`fps` | `0` = SDK 默认 |
| `device_preset` | `Default` |
| `disparity_to_depth_mode` | `HW` |
| `enable_laser` | `true` |
| `enable_disparity_to_depth` | `true` |
| HW 去噪 / spatial | 官方默认关；软去噪开 |

## 注意

无 SDK → stub。与 RealSense 默认话题冲突时只 enable 其一。
