# Hesai lidar

Backend: `hesai`（alias `pandar`）。

两条路径由工厂 `CreateHesaiLidarDriver` 分发：

| 路径 | 何时选用 | 依赖 |
|---|---|---|
| **UDP（默认）** | `model` 为 XT32 / PandarXT 族，且未设 `use_sdk` | 无第三方 SDK |
| **SDK** | `use_sdk: true` / `parser: sdk` / 非 XT32（如 **JT128**） | [HesaiLidar_SDK_2.0](https://github.com/HesaiTechnology/HesaiLidar_SDK_2.0) |

点云输出统一为 `PointCloud2`（`point_step=24`，x/y/z/intensity/timestamp）。

完整使用说明（JT128 网络 / 校正 / autoviz）：[`docs/source/sensor/lidar/hesai.md`](../../../docs/source/sensor/lidar/hesai.md)。

## UDP（PandarXT / XT32）

Pipeline：UDP → `PacketQueue` → 方位角切帧 → Convert → optional compensator。

| 文件 | 职责 |
|---|---|
| `packet.hpp` | 1080B XT32 UDP 布局 |
| `convert.*` / `calibration.*` | 仰角表（度） |
| `udp_driver.*` | online / `raw_packet` / `PushScan` |

设备示例：`config/lidar/hesai/xt32.yaml`。

## SDK（JT128 · 兼容 HesaiLidar_ROS_2.0）

| 文件 | 职责 |
|---|---|
| `sdk_driver.*` | `HesaiLidarSdk` 回调 → XYZIT PointCloud2 |

```bash
./scripts/install_hesai_lidar_sdk.sh
# CMake: -DAUTODRIVER_WITH_HESAI_SDK=ON

autodriver --config-dir "$AUTODRIVER_PATH" --config-file examples/jt128_verify.yaml
```

| 配置 | 型号 |
|---|---|
| `config/lidar/hesai/sdk.yaml` | 通用 SDK 模板 |
| `config/lidar/hesai/jt128.yaml` | JT128 |
| `config/examples/jt128_verify.yaml` | 最小验证 |

要点：广播收包时 `host_ip_address` 留空；必须配置角度校正（PTC 或 `correction_file_path`）；autoviz Fixed Frame = `hesai_jt128`。
