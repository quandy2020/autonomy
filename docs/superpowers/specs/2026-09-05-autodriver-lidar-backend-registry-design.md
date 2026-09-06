# autodriver Lidar 厂商后端注册表 Design

**Date:** 2026-09-05  
**Status:** Approved (chat) — awaiting written-spec review before implementation plan  
**Scope:** 方案 B — 厂商扩展框架（静态注册表 + Velodyne typed proto）；不拆 Driver/Convert/Compensator

## 背景

当前 `Lidar3dModule::MakeDriver` 用 `if (backend == "velodyne" || backend == "udp")` 硬编码创建驱动。新增 Hesai 等厂商必须改 `modules.cpp`。Apollo `modules/drivers/lidar` 按厂商分包、按名实例化；autodriver 保留单进程 `SensorManager`、不引入 Cyber DAG，用**进程内静态注册表**对齐「按名扩展」这一点。

## 目标

1. `lidar/<vendor>/` 自注册工厂；`MakeDriver` 只查表。
2. Proto 增加 `VelodyneLidarConf`（及 `SensorConfig.lidar_vendor` oneof）；加载时摊平进现有 `DriverParams`。
3. YAML 路径不变：`backend` + `params` 字符串表仍可用。
4. 为日后 Hesai 预留注册位与文档步骤；本轮**不实现** Hesai Convert。

## 非目标

- 拆分独立 Convert / Compensator 组件或中间 Scan 话题（属方案 A，另开）
- 外置 `.so` 厂商插件（现有 `library` 机制保留，本轮不用）
- 强制 IMU/GPS/Camera 也上同一注册表
- 删除或重构 RealSense `PointCloudModule` 分支

## 架构

```
YAML / .pb.txt
    ↓ LoadConfig / ConfigFromProto
Config::Sensor { backend, params }
    ↓ Lidar3dModule::MakeDriver
LidarBackendRegistry::Create(backend, id, params)
    ↓
CreateVelodyneUdpDriver / (future) CreateHesai…
```

静态注册发生在 `.so` 加载时（与 `CLASS_LOADER_REGISTER_CLASS` 同套路）：各厂商 translation unit 内 `REGISTER_LIDAR_BACKEND(...)` 构造全局对象，向单例 map 插入工厂。

## 组件

### `lidar/backend_registry.hpp` + `.cpp`

| API | 行为 |
|-----|------|
| `using LidarDriverFactory` | `(const SensorId&, const DriverParams&) → shared_ptr<SensorDriver>` |
| `Register(name, factory)` | 主名；重复注册同名 → 覆盖并 `AWARN`（便于测试） |
| `RegisterAlias(alias, canonical)` | 如 `"udp"` → `"velodyne"`；或 `Register` 接受 aliases 列表 |
| `Create(backend, id, params)` | 解析别名后调工厂；未知 → `nullptr` + `AERROR` |
| `Has(backend)` / `List()` | 测试与诊断用 |

线程：注册仅在静态初始化 / 测试中调用；运行时 `Create` 只读。用 `std::mutex` 保护 map 即可。

### `lidar/backend_register.hpp`

宏展开为命名空间内静态 `Registrar` 对象，构造函数调用 `Register`。避免 ODR 问题：宏参数含唯一 token（文件+行或显式 tag）。

示例：

```cpp
REGISTER_LIDAR_BACKEND(velodyne, "velodyne",
    autodriver::hardware::CreateVelodyneUdpDriver, "udp");
```

### Velodyne 接线

- 在 `lidar/velodyne/udp_driver.cpp` 末尾（或新建 `register.cpp`）调用注册宏。
- CMake 已编译该 TU 即可；无需在 `modules.cpp` include 厂商头（为保证静态注册被链接，仍需厂商 `.cpp` 链入 `libautodriver`；与今相同）。
- `modules.cpp`：`MakeDriver` 改为 `LidarBackendRegistry::Create(...)`；删除对 `CreateVelodyneUdpDriver` 的直接依赖（可去掉 `#include` velodyne 头，依赖链接侧注册）。

**链接注意：** 若未来拆成静态库且无引用厂商符号，注册可能被 GC。当前 `SHARED libautodriver` 直接编入各 `.cpp`，无此问题。若日后拆库，用显式 `ForceLinkVelodyne()` 或 `--whole-archive`（记入后续风险，本轮不处理）。

### Proto

```protobuf
message VelodyneLidarConf {
  int32 data_port = 1;             // default applied in driver if 0
  int32 packets_per_scan = 2;
  string model = 3;
  string frame_id = 4;
  string bind_host = 5;
  int32 reconnect_attempts = 6;
  bool enable_compensator = 7;
  string world_frame_id = 8;
  string extrinsic_path = 9;
}

message SensorConfig {
  // ... existing fields ...
  LidarConfigBase lidar = 23;
  oneof lidar_vendor {
    VelodyneLidarConf velodyne = 24;
    // reserved for HesaiLidarConf hesai = 25;
  }
}
```

`ConfigFromProto`：若 `has_velodyne()`：

1. 若 `backend` 空，设为 `"velodyne"`。
2. 将各非默认字段写入 `params`（与现有 `SetParamIf*` 风格一致；bool 用 `"true"`/`"false"`）。
3. 不覆盖已在 `params` map 中显式给出的同名键（`params` 优先）。

`ConfigToProto`：若 `backend` 为 `velodyne`/`udp` 且 params 含已知键，可填充 `velodyne` oneof（尽力而为；测试覆盖 From 路径即可）。

YAML：无强制改动；文档注明 pb.txt 可用 typed `velodyne { ... }`。

### 驱动与 Compensator

不变。`extrinsic_path` 若进入 params，本轮**不强制**驱动自动 `LoadExtrinsicYaml`（可选后续小项）；字段先进入 schema 以免再破 field number。

## 测试

| 用例 | 断言 |
|------|------|
| Registry 假工厂 | `Register("fake", …)` → `Create("fake")` 非空 |
| 别名 | `"udp"` 解析到与 `"velodyne"` 同一工厂 |
| 未知 backend | `Create("nope") == nullptr` |
| 模块路径 | 现有 `test_stream_lidar_base` raw_packet 仍绿 |
| Proto | `VelodyneLidarConf.data_port=2369` → params[`data_port`]==`"2369"`；backend 默认 `velodyne` |

## 文档

更新：`backends.md`（注册宏扩展步骤）、`api/overview.md`（registry 头文件）、`configuration.md`（pb.txt velodyne 块示例）、对齐计划勾选「厂商注册表」或新开计划。

## 成功标准

- 新增厂商（概念上）只需：目录 + Create* + `REGISTER_LIDAR_BACKEND` + CMake 源；**不改** `Lidar3dModule`。
- 现有 `backend: velodyne|udp` YAML 行为不变。
- 注册表与 proto 摊平有单测。

## 风险与缓解

| 风险 | 缓解 |
|------|------|
| 静态注册顺序 / 未链接 TU | 厂商源继续直接编入 shared lib |
| oneof 与 map params 双源 | 文档：params 优先；FromProto 不覆盖已有键 |
| 范围膨胀到拆流水线 | 本 spec 明确非目标 |
