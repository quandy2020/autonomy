# Atla2 Protobuf 消息

为 **VO / VIO / LO / LIO / LIVO** 设计的数据结构。原则：

1. **尽量复用 [automsgs](../../../../../../automsgs)**：`Header`、`Odometry`、`Imu`、`Image`、`PointCloud2`、`NavSatFix`、`FluidPressure`、`PoseWithCovariance`、`OccupancyGrid`、`Path` 等  
2. **每个 message（及独立 enum）一个 `.proto` 文件**  
3. 仅 Atla2 特有概念才新增（模式枚举、路标、偏置、健康、模式 I/O、deskew 扫描）

完备性检测见 [COMPLETENESS.md](COMPLETENESS.md)。

## 布局

```text
proto/
├── common/     # 枚举 + OdometryState / Landmark / ExtrinsicSet / Atla2ConfigMsg
├── sensor/     # SyncedSensorPacket、LidarScan、GpsSample、BarometerSample、OpticalFlow
├── mode/       # 各模式 *Input / *Result + FrontendInput/Output
├── status/     # SlamStatus、HealthReport、LoopClosureEvent
└── map/        # 点云 / 路标 / 关键帧 / 占据 / 高程 / 轨迹 / MapSnapshot
```

`package autonomy.localization.atla2.proto;`

## 模式 ↔ 消息

| 模式 | 输入 | 输出 | 复用的 automsgs |
|------|------|------|-----------------|
| VO | `VoInput` | `VoResult` | Image, CameraInfo, Odometry |
| VIO | `VioInput` | `VioResult` | Image, Imu, Odometry |
| LO | `LoInput` | `LoResult` | PointCloud2, Odometry |
| LIO | `LioInput` | `LioResult` | PointCloud2, Imu, Odometry |
| LIVO | `LivoInput` | `LivoResult` | Image, PointCloud2, Imu, NavSatFix, FluidPressure |

统一封装：`FrontendInput` / `FrontendOutput`（`oneof`）。

## 与 C++ 类型对应

| Proto | C++ |
|-------|-----|
| `SyncedSensorPacket` | `SensorData` |
| `OdometryState` | `OdometryResult` 核心（pose/twist/cov/bias） |
| `Landmark` | `Landmark` |
| `SlamStatus` / `HealthReport` | `SlamState` + `HealthReport` |
| `FrontendMode` | `FrontendMode`（含新增 LO） |

## 生成（建议）

```bash
# -I1: atla2/proto  -I2: automsgs proto include
protoc -I proto -I ${AUTOMSGS_PROTO_INCLUDE} \
  --cpp_out=gen \
  $(find proto -name '*.proto')
```

当前目录以 **消息设计** 为主；接入 `automsgs_proto` 代码生成可后续挂到 `atla2/CMakeLists.txt`。

## 文件清单

见各子目录；枚举文件无 `message`，但仍一文件一类型，便于独立演进。
