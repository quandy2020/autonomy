# Proto 完备性检测报告

检测对象：`atla2/proto/**/*.proto`  
对照：C++ `SensorData` / `OdometryResult` / `HealthReport` / `MapManager` / `Atla2Config`，以及 VO·VIO·LO·LIO·LIVO 数据流。

## 1. 结论（摘要）

| 维度 | 结论 |
|------|------|
| 五模式 I/O | **完备** |
| 与 C++ 核心类型映射 | **完备**（含 deskew / GPS / Baro / 外参 / 降级枚举） |
| 本地 import 图 | **完备** |
| automsgs 复用 | **良好** |
| P2 扩展 | **已补**（预积分、因子边、标定结果、体素元数据） |

当前约 **45** 个 proto 类型文件；模式覆盖 5/5；C++ `FrontendMode::kLo` + `LoFrontend` 已接入。

## 2. 模式覆盖矩阵

| 模式 | 输入 | 输出 | C++ Frontend | 平台 YAML |
|------|------|------|--------------|-----------|
| VO | `VoInput` | `VoResult` | `VoFrontend` | `drone_vo` |
| VIO | `VioInput` | `VioResult` | `VioFrontend` | `drone_vision_only` |
| LO | `LoInput` | `LoResult` | **`LoFrontend`** | **`drone_lo`** |
| LIO | `LioInput` | `LioResult` | `LioFrontend` | `drone_lidar_imu` |
| LIVO | `LivoInput` | `LivoResult` | `LivoFrontend` | `drone_livo` / `vtol_livo_gps` |

## 3. C++ ↔ Proto 对齐（关键）

| C++ | Proto | 状态 |
|-----|-------|------|
| `SensorData` | `SyncedSensorPacket` + `LidarScan`/`GpsSample`/`BarometerSample` | OK |
| `OdometryResult` | `OdometryState` + 模式 Result | OK |
| `DegradationLevel` | 含 Full/LIO/VIO/**LO**/**VO**/ImuCoast | **已对齐** |
| `HealthReport` | gps/baro/cov/recommended_mode | **已对齐** |
| `Atla2Config` | `Atla2ConfigMsg` | OK |
| `ImuPreintegration` | `ImuPreintegration` | OK |
| 标定 CLI | `CalibrationResult` | OK |
| 图优化调试 | `GraphFactorEdge` | OK |
| Voxel 地图 | `VoxelMapMeta` | OK |

## 4. 已关闭的缺口清单

1. Lidar deskew `t_begin` → `LidarScan`  
2. GPS ENU/HDOP → `GpsSample`  
3. 气压温/高 → `BarometerSample`  
4. 外参 → `ExtrinsicSet`  
5. 占据/高程/轨迹 → Occupancy/Elevation/Trajectory  
6. 配置序列化 → `Atla2ConfigMsg`  
7. 预积分 / 因子边 / 标定结果 / 体素元数据（P2）  
8. C++ LO 前端 + 降级枚举对齐  

## 5. 剩余可选（P3）

| 项 | 说明 |
|----|------|
| `ATLA2_GENERATE_PROTO=ON` | 工作区启用 codegen + 链接 `atla2_proto` |
| SensorData ↔ SyncedSensorPacket 往返单测 | 需生成 pb 后编写 |
| PointCloud2 点时间戳字段约定 | 文档化 `timestamp`/`time` 字段名 |
| 运行时按 `recommended_mode` 热切换 Frontend | 编排层能力 |

## 6. 验收

```bash
# 冒烟 LO
./build/bin/autonomy.localization.atla2_offline \
  --config .../config/platforms/drone_lo.yaml --steps 50

# 单测含 lo
ctest -R atla2 --output-on-failure
```
