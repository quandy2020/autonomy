# 术语

本表为 **autodriver/docs 用语的唯一权威**。其它页面须与此一致；代码标识、YAML 键、消息类型保留英文，不译。

> 定义依据源码与配置的静态约定。展开说明见右列链接。

| 相关 | 链接 |
|---|---|
| 分层与 Registry | [架构](architecture.md) |
| 样本路径 | [数据流](dataflow.md) |
| YAML 字段 | [配置](configuration.md) |
| Attach / udev | [生命周期](lifecycle.md) |

---

## 1. 架构与边界

| 术语 | 英文 / 代码 | 含义 |
|---|---|---|
| **模态** | modality / `SensorType` | 传感器类别，对应一个 `*Module`（如 IMU、相机、3D 激光） |
| **传感** | sensing | `SensorManager` + `SensorDriver`：硬件 → `SensorSample` → Autolink |
| **本体** | body / chassis | `ChassisManager` + `ChassisDriver`：`/cmd_vel` 与状态 / odom / event；与传感同进程、互不 `#include` 对方 SDK |
| **Module** | `SensorModule` | 按模态固定编入 `libautodriver`；查 Registry 建驱动，通常不加新 Module |
| **Driver** | `SensorDriver` / `ChassisDriver` | 厂商实现；经 Registry 插拔 |
| **Registry** | `*BackendRegistry` | `backend` 名 → Creator；封装 `NamedProductFactory` / Autolink Factory |
| **Creator** | `CreateXxx` | 返回 owning raw 指针；Registry 再包成 `SharedPtr`；stub 可返回 `nullptr` |
| **backend** | YAML 键 | 厂商名字符串（如 `velodyne`、`serial`）；省略时由 loader / Registry 填入模态默认值 |

---

## 2. 配置

| 术语 | 英文 / 代码 | 含义 |
|---|---|---|
| **折叠配置** | folded camera config | 一条 `camera` 下嵌套 `streams` / `point_clouds` / `imu`；loader 展开为多条 `Config::Sensor` |
| **板载 IMU** | onboard IMU | 相机模组内置 IMU；可写折叠 `camera.imu`，或独立 `imu` + `backend: realsense` 等 |
| **params_file** | — | 相对 `config/` 的厂商参数文件；条目内 `params:` 覆盖文件同名键 |
| **YAML 默认 backend** | — | 条目未写 `backend` 时 loader 写入的值（与 Registry 空串默认对齐） |
| **enable** | — | 仅 `true`（或旧别名 `attach_on_start`）的条目进入 `Config.sensors` 并 `autostart`；否则无法 Attach |

---

## 3. 数据路径

| 术语 | 英文 / 代码 | 含义 |
|---|---|---|
| **热路径** | hot path | 默认链：Driver → Module → Sink → Publisher；**不经** `SensorHub` |
| **对齐旁路** | alignment tap | `alignment.enable` 时额外 `hub.PushSample`；由 `publish_raw` / `publish_aligned` 控制是否仍发原始流、是否发对齐快照 |
| **SampleSink** | `SampleSink` | 样本出口抽象；进程内通常为 `bridge::Publisher` |
| **切帧** | scan cut | 3D UDP 激光按方位角或包数切出完整扫描（`scan_cut`） |
| **运动补偿** | motion compensation | `PoseFeeder` 订 Odometry → `PushLidarPose` → `MotionPoseSink`；仅 `enable_compensator` 的 3D 驱动使用 |
| **语句解析** | sentence parsing | GNSS NMEA 等按句解析（`GnssParserRegistry`）；与串口/CAN **传输后端**正交 |

对齐开关真值表见 [数据流 §2.2](dataflow.md#22-对齐旁路可选)。

---

## 4. 生命周期与占位

| 术语 | 英文 / 代码 | 含义 |
|---|---|---|
| **Attach / Detach** | — | 挂载 / 卸载 Module 与 Writer；幂等；失败返回 `false`，不终止进程 |
| **仅 Attach** | attach-only | Module 可挂载，但无真实采集驱动（如 `RangeModule`） |
| **stub / 占位** | stub | Registry 已注册，但 Create 常返回 `nullptr`，或消息类型仅为占位（Radar / Mic 等） |
| **看门狗** | watchdog | 底盘 `watchdog_ms` 内无新 `cmd_vel` 则下发零速（soft stop）；`0` 关闭 |
| **DeviceMatch** | `match` | udev 匹配规则（subsystem / device / vendor / product / serial） |

---

## 5. 样本类型

须与 `types/sensor_sample.hpp` 一致；**禁止杜撰**类型名。

| SensorType | 样本类型 | 典型消息 |
|---|---|---|
| `kImu` | `ImuSample` | `sensor_msgs/Imu` |
| `kGps` | `GpsSample` | `sensor_msgs/NavSatFix` |
| `kCamera` | `CameraFrame`（勿写 `ImageSample`） | `sensor_msgs/Image`（+ camera_info） |
| `kLidar2d` | `LidarScan` | `sensor_msgs/LaserScan` |
| `kLidar3d` | `LidarCloud` / `LidarPacketScan` | `PointCloud2` / 原始 Scan |
| `kRadar` | `RadarSample` | PointCloud2（占位） |
| `kMicrophone` | `MicrophoneSample` | Image 承载 PCM（占位） |
| `kRangeFinder` | — | 仅 Attach，无样本 |

---

## 6. 宜 / 不宜（文档用语）

| 不宜 | 宜 |
|---|---|
| Create 空 | Create 返回 `nullptr` |
| 真驱动 / 无真数据 | 已实现驱动 / 无真实采集数据 |
| 占名 | 占位注册 |
| 装 PCM | 以 Image 消息承载 PCM |
| 句解析 | 语句解析 |
| 进程胶水 | 进程启动顺序 |
| attach-only（正文叙述） | 仅 Attach（首次可括注 attach-only） |
| `ImageSample` | `CameraFrame` |
