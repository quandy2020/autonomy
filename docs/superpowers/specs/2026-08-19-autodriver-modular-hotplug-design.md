# autodriver：分模态模块、class_loader 热插拔与 autolink 发布

**日期：** 2026-08-19  
**状态：** 待评审  
**范围：** `autodriver/` 重构为按传感器模态拆分的插件；经 `autolink::class_loader` 动态加载；用 autolink Writer 发布 automsgs；Linux udev 热插拔；删除生产 mock。

## 1. 目标与边界

### 1.1 目标

- 分别管理 IMU、Range、Lidar、Camera、GPS：每种模态一个 `.so` 插件
- 采集数据经 **autolink + automsgs** 发布（主路径）
- 运行时 **Attach / Detach**；Linux 上 **udev** 驱动热插拔
- 模块化、Google C++ 风格（仓库 `.clang-format`：Google，4 空格）
- 生产代码删除 `mock_*` 与周期假数据

### 1.2 非目标（本轮不做）

- 不做 Wheel Odometry 插件（类型文件可保留，不进配置与热插拔）
- 不实现真实 Lidar / Range 硬件驱动（仅骨架：Writer 在线、零样本）
- macOS 不做 IOKit / 设备自动发现
- 不对齐窗口作为发布前置条件（对齐为可选旁路）
- 不把插件做成 autolink Component / DAG / mainboard
- 不引入每模态独立进程

### 1.3 已确认决策

| 项 | 选择 |
|----|------|
| 架构 | 独立模块发布为主路径；对齐可选旁路 |
| 热插拔 | Attach/Detach API（全平台）+ Linux udev；macOS 监视器空实现 |
| mock | 生产删除；测试用进程内 `FakeModule`；Lidar/Range 骨架不发假数据 |
| 模态 | 仅 IMU / Range / Lidar / Camera / GPS |
| 进程 | 一个进程、一个 `autolink::Node`（名 `autodriver`） |
| 加载 | 方案 1 + `ClassLoaderManager` 加载 `.so` |
| 插件粒度 | 一种模态一个库；`serial` / `can` / `realsense` 为模块内 backend |

## 2. 架构

```
                  ┌─────────────────────────────────────┐
  udev ADD/REMOVE │ DeviceMonitor (Linux) / no-op (mac) │
  Attach/Detach   └─────────────────┬───────────────────┘
                                    ▼
                           SensorManager
                    LoadLibrary / CreateClassObj
                    UnloadLibrary (refcount == 0)
                                    │
         ┌────────────┬─────────────┼─────────────┬────────────┐
         ▼            ▼             ▼             ▼            ▼
   ImuModule    GpsModule    CameraModule   LidarModule   RangeModule
   (plugin .so) (plugin .so) (plugin .so)   (skeleton)    (skeleton)
         │            │             │             │            │
         └────────────┴──────► Writer.Write(automsgs) ◄────────┘
                                    │
                                    ▼
                          autolink Node "autodriver"
                                    │
                         可选 tap ──► SensorHub（对齐旁路）
```

主路径：设备事件 → 加载插件 → `Init` 建 Writer → `Start` 采集 → 转 automsgs → `Write`。  
对齐失败或未启用 **不得** 阻塞或丢弃主路径发布。

### 2.1 单元职责

| 单元 | 做什么 | 怎么用 | 依赖 |
|------|--------|--------|------|
| `SensorModule` | 插件基类：默认构造 + `Init` / `Start` / `Stop` | `CLASS_LOADER_REGISTER_CLASS(ImuModule, SensorModule)` | `libautodriver`、autolink Node |
| `SensorManager` | 持有 Node、配置、实例表；幂等 Attach/Detach | 进程入口调用 `Initialize` / `Start` / `Stop` | `ClassLoaderManager`、`DeviceMonitor` |
| `DeviceMonitor` | Linux udev 匹配规则；回调只入队 | `Start(rules, callback)` | libudev（仅 Linux） |
| `SensorDriver` | 硬件采集（串口/CAN/RealSense） | 仅被对应 `*Module` 内部持有 | io / protocol |
| `ToAutonomy*` | HAL sample → automsgs | 模块回调里调用后 `Write` | automsgs proto |
| `SensorHub` | 可选时间对齐 | `alignment.enable=true` 时由 Manager 注入 tap | 不依赖 Writer |

`class_loader` 要求默认构造，因此 **插件类型不得** 把 `SensorId` / `DriverParams` 放进构造函数。硬件 `SensorDriver` 仍可用带参构造，因为它们不经 `CreateClassObj`。

删除：`SensorFactory` 单例、`drivers/driver_registry.*`、`drivers/mock/`、`DefaultSimulationHubConfig`、`register_builtin_mocks`。

## 3. 目录与构建

```
autodriver/
├── autodriver/                 # libautodriver（无 mock、无具体模态实现）
│   ├── app/                    # SensorManager
│   ├── module/                 # SensorModule、SensorModuleContext
│   ├── publish/                # 通道名解析、Writer 生命周期约定
│   ├── hotplug/                # DeviceMonitor 接口；udev 实现；匹配函数
│   ├── config/                 # AutodriverConfig + SensorConfig（替换 HubConfig / DriverConfig）
│   ├── hal/                    # SensorDriver（硬件基类，非插件基类）
│   ├── io/  protocol/  types/  sync/   # serial/can 留核心；realsense 见下
│   └── convert/                # 现 bridge/autonomy/sample_converter 迁入核心
├── plugins/
│   ├── imu/                    # libautodriver_imu.so
│   ├── gps/                    # libautodriver_gps.so
│   ├── camera/                 # libautodriver_camera.so
│   ├── lidar/                  # libautodriver_lidar.so（骨架）
│   └── range/                  # libautodriver_range.so（骨架）
├── examples/
├── test/                       # 含 FakeModule，不链入 libautodriver
└── CMakeLists.txt
```

- `libautodriver` **PUBLIC** 链接 `autolink` 与 automsgs 生成库，**不**链接 `realsense2`。
- 每个插件为 `SHARED`，链接 `autodriver`，导出 `CLASS_LOADER_REGISTER_CLASS`。
- `io/realsense_device.*` 移出核心，编进 OBJECT 库 `autodriver_realsense_io`，仅 `imu` / `camera` 插件链接；`AUTODRIVER_WITH_REALSENSE` 只作用于这两个插件。未找到 librealsense2 时，`backend=realsense` 的 `Init` 返回 false 并打日志。
- `bridge/autonomy/collator_sink.*` 仍为可选 `AUTODRIVER_BUILD_AUTONOMY_BRIDGE`；本轮只保证它改用新 `SensorConfig` / 转换函数，不作为发布主路径。
- 安装：核心库 + 五个插件 `.so` 到同一 lib 目录；`plugin_dir` 为空时用该目录。

注册类名（与 Lua `class_name` 逐字一致）：

| 库 | 类名 |
|----|------|
| `libautodriver_imu.so` | `ImuModule` |
| `libautodriver_gps.so` | `GpsModule` |
| `libautodriver_camera.so` | `CameraModule` |
| `libautodriver_lidar.so` | `LidarModule` |
| `libautodriver_range.so` | `RangeModule` |

## 4. 接口

### 4.1 SensorModule

```cpp
class SensorModule {
 public:
  virtual ~SensorModule() = default;
  virtual SensorType GetType() const = 0;
  virtual const SensorId& GetSensorId() const = 0;
  virtual bool Init(const SensorModuleContext& context) = 0;
  virtual bool Start() = 0;
  virtual void Stop() = 0;
  virtual bool IsRunning() const = 0;

 protected:
  SensorModule() = default;
};
```

`SensorModuleContext`：共享 `Node`、本条 `SensorConfig`（原 `DriverConfig`）、可选 `SampleTap`（对齐旁路）。

约定：

- `Init`：按 `channel` 创建 Writer（QoS：`KEEP_LAST`，depth 10）；打开设备资源但未开始采集。失败时不得留下 Writer。
- `Start`：启动采集线程；将 sample 转为 automsgs 后 `Write`；可选调用 tap。
- `Stop`：停线程、关设备、释放 Writer（`shared_ptr` 置空，通道从发现中消失）。
- `Init`/`Start` 不得抛出到 Manager 以外；内部异常转为 `false` + `AERROR`。

### 4.2 SensorManager

```cpp
bool Initialize();                 // 建 Node、读配置、启动监视器
bool Attach(const SensorId& id);   // 按配置加载插件并 Start
void Detach(const SensorId& id);
bool Start();                      // attach_on_start==true 的条目执行 Attach
void Stop();                       // Detach 全部、停监视器
```

- `Attach` / `Detach` **幂等**：已运行再 Attach 返回 true；未 Attach 再 Detach 为空操作。
- 同一 `sensor_id` 配置重复：`Initialize` 失败。
- 未知 `sensor_id` 的 `Attach`：返回 false，日志，进程继续。
- 库加载失败 / 类名无效 / `Init` 或 `Start` 失败：不保留实例，返回 false。
- 该 `library` 的存活实例数为 0 时 `UnloadLibrary`。
- udev 回调 **只入队**；`Attach`/`Detach`/`dlopen` 在 Manager 工作线程执行。

### 4.3 模态与 backend

| 模块 | backend | 行为 |
|------|---------|------|
| IMU | `serial` | 现有 WIT-motion 串口驱动 |
| IMU | `can` | 现有 CAN IMU |
| IMU | `realsense` | 现有 RealSense IMU；未编译则 `Init` 失败 |
| GPS | `serial` | 现有 NMEA 串口 |
| GPS | `can` | 现有 CAN GPS |
| Camera | `realsense` | `stream=color` 或 `depth`；可选第二 Writer `camera_info` |
| Lidar | 忽略 | 骨架：`Init`/`Start` 成功，**零 `Write`** |
| Range | 忽略 | 同上 |

未知 `backend`：`Init` 返回 false。本轮不实现其它 backend。

## 5. 通道与消息

默认通道可被每条 `SensorConfig.channel` 覆盖。多实例时必须在配置里改成不冲突的名字（例如 `/imu/serial`）；Manager **不**自动加前缀。

| 模态 | 默认 channel | automsgs |
|------|----------------|----------|
| IMU | `/imu` | `sensor_msgs.Imu` |
| GPS | `/gps/fix` | `sensor_msgs.NavSatFix` |
| Camera color | `/camera/image_raw` | `sensor_msgs.Image` |
| Camera depth | `/camera/depth/image_raw` | `sensor_msgs.Image` |
| Camera 标定 | `/camera/camera_info` | `sensor_msgs.CameraInfo`（配置 `publish_camera_info=true` 时） |
| Lidar 2D | `/scan` | `sensor_msgs.LaserScan` |
| Lidar 3D | `/points` | `sensor_msgs.PointCloud2`（本轮骨架不创建该 Writer） |
| Range | `/range` | `sensor_msgs.Range` |

`header.stamp` 使用 sample 的 `host_time()`（无对齐时等于设备时间）。`frame_id` 使用 `sensor_id`。

## 6. 热插拔匹配

每条传感器配置可带 `match`。Linux `DeviceMonitor` 在 `ADD`/`REMOVE` 时用纯函数 `MatchUdevDevice(attrs, match) → sensor_id` 查找配置，再入队 Attach/Detach。

匹配字段（全部可选，同时给出的必须全部命中）：

- `subsystem`（如 `tty`、`usb`）
- `devnode`（如 `/dev/ttyUSB0`）
- `vendor` / `product`（十六进制字符串，比较 USB idVendor/idProduct）
- `serial`

无 `match` 的条目 **不**参与 udev，只响应 `attach_on_start` 或显式 `Attach`。

采集线程 `read` 失败 / EOF：驱动 `Stop`，通知 Manager `Detach`（幂等）。随后 `ADD` 可再次 Attach。

macOS：`DeviceMonitor::Start` 立即返回成功且不产生事件。需要设备时把 `attach_on_start = true`。

## 7. 配置

仍用 Lua 表 `AUTODRIVER`。加载函数改名为 `LoadAutodriverConfigFromDirectory`（替换 `LoadHubConfigFromDirectory`）；键名如下；删除 `register_builtin_mocks` 与 `factory_name`。

```lua
AUTODRIVER = {
  node_name = "autodriver",
  plugin_dir = "",
  hotplug = { enable_udev = true },
  alignment = {
    enable = false,
    alignment_window_ms = 50,
    publish_period_ms = 20,
    buffer_capacity = 32,
  },
  sensors = {
    {
      class_name = "ImuModule",
      library = "libautodriver_imu.so",
      sensor_id = "imu/serial",
      channel = "/imu",
      backend = "serial",
      attach_on_start = false,
      match = { subsystem = "tty", devnode = "/dev/ttyUSB0" },
      params = { device = "/dev/ttyUSB0", baud = 115200 },
    },
  },
}
```

- `config/driver/autodriver.lua`：默认 **空 `sensors`**（不再挂 mock）。
- `autodriver_hardware.lua` / `autodriver_realsense.lua`：改为上表字段；去掉 mock 开关。
- `attach_on_start`：Linux + udev 匹配的设备建议 `false`；macOS 或固定路径建议 `true`。
- `enable_udev` 在非 Linux 上被忽略。
- `alignment.enable=false`（默认）：不建对齐线程。

## 8. 错误处理

| 情况 | 行为 |
|------|------|
| 插件路径不存在 / `dlopen` 失败 | Attach 失败，`AERROR`，进程继续 |
| `class_name` 未注册 | 同上 |
| `Init` 失败 | 不 Start、无 Writer、无实例表条目 |
| `Start` 失败 | `Stop` 清理后移除实例 |
| 重复 `sensor_id` 配置 | `Initialize` 返回 false |
| udev 回调中的工作 | 入队；工作线程串行处理同一 `sensor_id` |
| `Write` 失败 | 丢帧；模块内计数；不重启驱动 |
| 设备中途消失 | Stop + Detach；不 abort |
| RealSense 未编译却选该 backend | `Init` false |
| Lidar/Range 骨架 | Start 成功，永不 `Write` |

日志一律 autolink `AINFO` / `AWARN` / `AERROR`。生产路径不 `abort`、不把异常甩出 Manager API（API 返回 bool）。

## 9. 测试

生产库 **零** 周期假 IMU/点云/图像。`FakeModule` 只编进 `autodriver` 测试目标，用 `CLASS_LOADER_REGISTER_CLASS` 进程内注册（无需 `.so`）。

| 用例 | 期望 |
|------|------|
| Attach 两次同一 id | 第二次成功且仍单实例 |
| Detach 未 Attach 的 id | 空操作 |
| 未知 library / class_name | Attach false |
| 加载 `libautodriver_lidar.so` | Init+Start 成功；测试 Reader 在超时内收到 **0** 条 |
| `MatchUdevDevice` | 给定 attrs 命中/不命中对应 `match` |
| `ToAutonomy*` | 手造 HAL sample，字段与现转换一致 |
| NMEA / WIT / RealSense types | 保留现有协议单测 |
| 对齐旁路（可选） | `FakeModule` 推 sample，Hub 产出 snapshot；不经过 mock 驱动 |

删除依赖 `mock_*` 的 `test_sensor_manager` / `test_sensor_hub` 用例。  
`examples/demo_main.cpp`：Attach Lidar 骨架，运行短时间，确认无 mock 输出。  
`examples/hub_main.cpp`：Lua 配置 + Manager；无传感器时仍保持进程存活，等待 Attach/udev。

## 10. 风格与命名

- 格式：仓库根 `.clang-format`（Google，缩进 4，列宽 80）。本轮 **改动过的** 文件必须格式化；不借机全库重排未触及文件。
- 命名（Google）：类型 PascalCase；函数 PascalCase；变量与数据成员 snake_case，成员尾下划线；常量 `kFoo`；命名空间 `autodriver`、`autodriver::imu` 等。
- 插件源文件放在 `plugins/<modality>/`，头文件不安装到核心 public API（外部只依赖 `SensorModule` + `SensorManager`）。

## 11. 成功标准

1. 生产目标中不存在 `mock_*` 符号与 `DefaultSimulationHubConfig`。
2. 五种插件均可被 `ClassLoaderManager` 加载；Lidar/Range 不发布任何样本。
3. IMU/GPS/Camera 在对应硬件与配置下，autolink 上可订阅到第 5 节所列 proto。
4. `Attach`/`Detach` 幂等；Linux 上 udev ADD/REMOVE（或测试注入的等价事件）会启动/停止对应模块。
5. macOS 构建不链接 libudev；监视器为空实现。
6. 现有协议单测与新的 Manager/匹配/骨架加载测试通过。
