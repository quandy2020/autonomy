# autodriver Modular Hotplug Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 把 `autodriver` 拆成五种传感器插件（IMU/GPS/Camera/Lidar/Range），经 `class_loader` 热加载，用 autolink 发布 automsgs，并删除生产 mock。

**Architecture:** 核心库只保留 `SensorModule` / `SensorManager` / 热插拔匹配 / 转换；每种模态一个 `.so`。一个进程一个 `Node`。`Attach`/`Detach` 幂等；Linux udev 只入队。对齐是可选 tap，不挡发布。`ClassLoaderManager::UnloadLibrary` 为 private，因此 Manager 按库路径持有 `ClassLoader`，最后一实例 Detach 后销毁 loader（析构即 unload）。

**Tech Stack:** C++17、GTest、autolink（Node / class_loader / log）、automsgs protobuf、libudev（仅 Linux）、仓库 `.clang-format`（Google，4 空格）

**Spec:** `docs/superpowers/specs/2026-08-19-autodriver-modular-hotplug-design.md`

**Build / test（后续步骤一律用这组命令，除非另写）：**

```bash
# 在仓库根目录。embed 默认关掉了 autodriver 测试，Task 6 会改掉。
cmake -S . -B build -DBUILD_AUTODRIVER=ON
cmake --build build -j --target autodriver
ctest --test-dir build -R 'test_(sensor_manager|udev_match|default_channel|plugin_lidar|nmea0183|wit_motion|realsense_types|time_sync|sample_converter)' --output-on-failure
```

未改 embed 之前，可在 `autodriver/` 独立构建（Task 6 之后改为根工程）。

---

## File Map

| 路径 | 职责 |
|------|------|
| `autodriver/autodriver/autodriver_config.hpp` | `DeviceMatch` / `SensorConfig` / `AutodriverConfig` |
| `autodriver/autodriver/config/autodriver_config.cpp` | `HasDuplicateSensorId` |
| `autodriver/autodriver/publish/channel.hpp` | 默认 channel 与 Writer RoleAttributes（depth 10） |
| `autodriver/autodriver/publish/channel.cpp` | 实现 |
| `autodriver/autodriver/hotplug/udev_match.hpp` | `UdevAttrs` + `MatchUdevDevice` |
| `autodriver/autodriver/hotplug/udev_match.cpp` | 纯函数匹配 |
| `autodriver/autodriver/hotplug/device_monitor.hpp` | 监视器接口 |
| `autodriver/autodriver/hotplug/noop_device_monitor.cpp` | macOS / 默认空实现 |
| `autodriver/autodriver/hotplug/udev_device_monitor.cpp` | Linux udev（`#if defined(__linux__)`） |
| `autodriver/autodriver/sensor_module.hpp` | 插件基类 |
| `autodriver/autodriver/sensor_manager.hpp` | Attach/Detach |
| `autodriver/autodriver/app/sensor_manager.cpp` | class_loader + 工作队列 |
| `autodriver/autodriver/convert/sample_converter.hpp` | 从 bridge 迁入 |
| `autodriver/plugins/lidar/lidar_module.cpp` | 骨架 |
| `autodriver/plugins/range/range_module.cpp` | 骨架 |
| `autodriver/plugins/gps/gps_module.cpp` | serial/can |
| `autodriver/plugins/imu/imu_module.cpp` | serial/can/realsense |
| `autodriver/plugins/camera/camera_module.cpp` | realsense |
| `autodriver/test/fake_module.hpp` / `fake_module.cpp` | 仅测试目标（进程内 class_loader） |
| `config/driver/autodriver.lua` | 空 sensors |
| 删除 | `drivers/mock/`、`hal/sensor_factory.*`、`drivers/driver_registry.*`、`DefaultSimulationHubConfig` |

硬件驱动文件 **git mv** 到对应 `plugins/<modality>/`（不改协议逻辑）。`io/realsense_device.*` 编进 OBJECT `autodriver_realsense_io`。

新文件版权头与现有 `autodriver` 源文件相同（Apache-2.0, Autodriver contributors）。

---

### Task 1: AutodriverConfig

**Files:**
- Create: `autodriver/autodriver/autodriver_config.hpp`
- Create: `autodriver/autodriver/config/autodriver_config.cpp`
- Create: `autodriver/test/test_autodriver_config.cpp`
- Modify: `autodriver/autodriver/CMakeLists.txt`（先把新 cpp 加进 `AUTODRIVER_SOURCES`）
- Modify: `autodriver/CMakeLists.txt`（注册测试；若 `GTest` 已有 `autodriver_add_test`）

- [ ] **Step 1: 写失败测试**

```cpp
#include <gtest/gtest.h>
#include "autodriver/autodriver_config.hpp"

TEST(AutodriverConfig, DetectsDuplicateSensorId) {
    autodriver::AutodriverConfig config;
    autodriver::SensorConfig a;
    a.sensor_id = "imu/a";
    autodriver::SensorConfig b;
    b.sensor_id = "imu/a";
    config.sensors = {a, b};
    EXPECT_TRUE(autodriver::HasDuplicateSensorId(config));
}

TEST(AutodriverConfig, UniqueIdsAreOk) {
    autodriver::AutodriverConfig config;
    autodriver::SensorConfig a;
    a.sensor_id = "imu/a";
    autodriver::SensorConfig b;
    b.sensor_id = "gps/b";
    config.sensors = {a, b};
    EXPECT_FALSE(autodriver::HasDuplicateSensorId(config));
}
```

- [ ] **Step 2: 跑测试确认失败**

```bash
# 先把测试文件编进去会编译失败（缺头文件），符合 TDD
```

Expected: compile error `autodriver_config.hpp` not found.

- [ ] **Step 3: 实现配置类型**

`autodriver/autodriver/autodriver_config.hpp`:

```cpp
#ifndef AUTODRIVER_CONFIG_AUTODRIVER_CONFIG_HPP_
#define AUTODRIVER_CONFIG_AUTODRIVER_CONFIG_HPP_

#include <string>
#include <vector>

#include "autodriver/sensor_id.hpp"
#include "autodriver/driver_params.hpp"
#include "autodriver/sync/sensor_hub.hpp"

namespace autodriver {

struct DeviceMatch {
    std::string subsystem;
    std::string devnode;
    std::string vendor;
    std::string product;
    std::string serial;
};

struct SensorConfig {
    std::string class_name;
    std::string library;
    SensorId sensor_id;
    std::string channel;
    std::string backend;
    bool attach_on_start = false;
    DeviceMatch match;
    hardware::DriverParams params;
};

struct HotplugOptions {
    bool enable_udev = true;
};

struct AlignmentConfig {
    bool enable = false;
    SensorHubOptions hub_options;
};

struct AutodriverConfig {
    std::string node_name = "autodriver";
    std::string plugin_dir;
    HotplugOptions hotplug;
    AlignmentConfig alignment;
    std::vector<SensorConfig> sensors;
};

bool HasDuplicateSensorId(const AutodriverConfig& config);

}  // namespace autodriver

#endif  // AUTODRIVER_CONFIG_AUTODRIVER_CONFIG_HPP_
```

`autodriver_config.cpp`:

```cpp
#include "autodriver/autodriver_config.hpp"

#include <unordered_set>

namespace autodriver {

bool HasDuplicateSensorId(const AutodriverConfig& config) {
    std::unordered_set<SensorId> seen;
    for (const SensorConfig& sensor : config.sensors) {
        if (!seen.insert(sensor.sensor_id).second) {
            return true;
        }
    }
    return false;
}

}  // namespace autodriver
```

把 `autodriver_config.cpp` 加入 `AUTODRIVER_SOURCES`。`autodriver_add_test(test_autodriver_config test/test_autodriver_config.cpp)`。

- [ ] **Step 4: 跑测试确认通过**

```bash
cmake --build <build> -j --target test_autodriver_config
ctest --test-dir <build> -R test_autodriver_config --output-on-failure
```

Expected: PASS

- [ ] **Step 5: Commit**

```bash
git add autodriver/autodriver/autodriver_config.hpp \
        autodriver/autodriver/config/autodriver_config.cpp \
        autodriver/test/test_autodriver_config.cpp \
        autodriver/autodriver/CMakeLists.txt autodriver/CMakeLists.txt
git commit -m "feat(autodriver): add AutodriverConfig types"
```

---

### Task 2: 默认 channel 与 Writer QoS

**Files:**
- Create: `autodriver/autodriver/publish/channel.hpp`
- Create: `autodriver/autodriver/publish/channel.cpp`
- Create: `autodriver/test/test_default_channel.cpp`

- [ ] **Step 1: 写失败测试**

```cpp
#include <gtest/gtest.h>
#include "autodriver/publish/channel.hpp"
#include "autodriver/types/sensor_type.hpp"

TEST(DefaultChannel, UsesSpecDefaults) {
    using autodriver::SensorType;
    EXPECT_EQ(autodriver::DefaultChannel(SensorType::kImu), "/imu");
    EXPECT_EQ(autodriver::DefaultChannel(SensorType::kGps), "/gps/fix");
    EXPECT_EQ(autodriver::DefaultChannel(SensorType::kRangeFinder), "/range");
    EXPECT_EQ(autodriver::DefaultChannel(SensorType::kLidar), "/scan");
    EXPECT_EQ(autodriver::DefaultChannel(SensorType::kCamera, "color"),
              "/camera/image_raw");
    EXPECT_EQ(autodriver::DefaultChannel(SensorType::kCamera, "depth"),
              "/camera/depth/image_raw");
}

TEST(DefaultChannel, PrefersOverride) {
    EXPECT_EQ(autodriver::ResolveChannel("/imu/serial",
                                        autodriver::SensorType::kImu),
              "/imu/serial");
    EXPECT_EQ(autodriver::ResolveChannel("", autodriver::SensorType::kImu),
              "/imu");
}
```

- [ ] **Step 2: 确认编译失败**

Expected: `channel.hpp` not found.

- [ ] **Step 3: 实现**

`channel.hpp`:

```cpp
#ifndef AUTODRIVER_PUBLISH_CHANNEL_HPP_
#define AUTODRIVER_PUBLISH_CHANNEL_HPP_

#include <string>

#include "autodriver/types/sensor_type.hpp"

namespace autodriver {

std::string DefaultChannel(SensorType type, const std::string& stream = {});

std::string ResolveChannel(const std::string& configured, SensorType type,
                           const std::string& stream = {});

}  // namespace autodriver

#endif  // AUTODRIVER_PUBLISH_CHANNEL_HPP_
```

`channel.cpp`：`kCamera` 在 `stream=="depth"` 时返回 `/camera/depth/image_raw`，否则 `/camera/image_raw`；`kWheelOdometry` 返回空串（本轮不用）。`MakeWriterAttr` 放到 Task 6（链接 autolink 之后），本任务不要 include protobuf。

- [ ] **Step 4: 测试通过**

Expected: PASS

- [ ] **Step 5: Commit**

```bash
git commit -m "feat(autodriver): add default autolink channel names"
```

---

### Task 3: udev 匹配纯函数

**Files:**
- Create: `autodriver/autodriver/hotplug/udev_match.hpp`
- Create: `autodriver/autodriver/hotplug/udev_match.cpp`
- Create: `autodriver/test/test_udev_match.cpp`

- [ ] **Step 1: 写失败测试**

```cpp
#include <gtest/gtest.h>
#include "autodriver/hotplug/udev_match.hpp"

TEST(UdevMatch, EmptyMatchNeverHits) {
    autodriver::DeviceMatch match;
    autodriver::UdevAttrs attrs;
    attrs.subsystem = "tty";
    attrs.devnode = "/dev/ttyUSB0";
    EXPECT_FALSE(autodriver::MatchUdevDevice(attrs, match));
}

TEST(UdevMatch, AllSpecifiedFieldsMustHit) {
    autodriver::DeviceMatch match;
    match.subsystem = "tty";
    match.devnode = "/dev/ttyUSB0";
    autodriver::UdevAttrs attrs;
    attrs.subsystem = "tty";
    attrs.devnode = "/dev/ttyUSB0";
    EXPECT_TRUE(autodriver::MatchUdevDevice(attrs, match));
    attrs.devnode = "/dev/ttyUSB1";
    EXPECT_FALSE(autodriver::MatchUdevDevice(attrs, match));
}

TEST(UdevMatch, FindsSensorId) {
    autodriver::SensorConfig imu;
    imu.sensor_id = "imu/serial";
    imu.match.subsystem = "tty";
    imu.match.devnode = "/dev/ttyUSB0";
    autodriver::AutodriverConfig config;
    config.sensors = {imu};
    autodriver::UdevAttrs attrs;
    attrs.subsystem = "tty";
    attrs.devnode = "/dev/ttyUSB0";
    EXPECT_EQ(autodriver::FindMatchingSensorId(attrs, config), "imu/serial");
}
```

- [ ] **Step 2: 确认失败**

Expected: missing header.

- [ ] **Step 3: 实现**

规则：`DeviceMatch` 五个字段全空 → 不参与 udev（返回 false）。非空字段必须与 `UdevAttrs` 相等（`vendor`/`product` 比较前去掉 `0x` 并转小写）。`FindMatchingSensorId` 返回第一个命中的 `sensor_id`，否则空串。

- [ ] **Step 4: 测试通过**

Expected: PASS

- [ ] **Step 5: Commit**

```bash
git commit -m "feat(autodriver): add udev match helper"
```

---

### Task 4: SensorModule 与 FakeModule

**Files:**
- Create: `autodriver/autodriver/sensor_module.hpp`
- Create: `autodriver/test/fake_module.hpp`
- Create: `autodriver/test/fake_module.cpp`
- Create: `autodriver/test/test_fake_module.cpp`

- [ ] **Step 1: 写失败测试**

```cpp
#include <gtest/gtest.h>
#include "autodriver/sensor_module.hpp"
#include "autolink/class_loader/class_loader_manager.hpp"
#include "fake_module.hpp"

TEST(FakeModule, InProcessCreate) {
    autolink::class_loader::ClassLoaderManager manager;
    auto module = manager.CreateClassObj<autodriver::SensorModule>("FakeModule");
    ASSERT_NE(module, nullptr);
    EXPECT_EQ(module->GetType(), autodriver::SensorType::kImu);
}
```

测试目标必须编译 `fake_module.cpp`（宏注册在 cpp，不在头文件）。

- [ ] **Step 2: 确认失败**

Expected: `sensor_module.hpp` not found.

- [ ] **Step 3: 实现接口与 FakeModule**

`sensor_module.hpp`:

```cpp
#ifndef AUTODRIVER_MODULE_SENSOR_MODULE_HPP_
#define AUTODRIVER_MODULE_SENSOR_MODULE_HPP_

#include <functional>
#include <memory>

#include "autodriver/sensor_id.hpp"
#include "autodriver/autodriver_config.hpp"
#include "autodriver/types/sensor_sample.hpp"
#include "autodriver/types/sensor_type.hpp"
#include "autolink/node/node.hpp"

namespace autodriver {

using SampleTap = std::function<void(const SensorSample&)>;

struct SensorModuleContext {
    std::shared_ptr<autolink::Node> node;
    SensorConfig config;
    SampleTap tap;
};

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

}  // namespace autodriver

#endif  // AUTODRIVER_MODULE_SENSOR_MODULE_HPP_
```

`test/fake_module.hpp`：全局命名空间 `class FakeModule : public autodriver::SensorModule`；`Init` 保存 `config.sensor_id`，不创建 Writer。

`test/fake_module.cpp` **只在这一处**注册（头文件禁止放宏，避免多 TU 重复注册）：

```cpp
#include "fake_module.hpp"
#include "autolink/class_loader/class_loader_register_macro.hpp"
CLASS_LOADER_REGISTER_CLASS(FakeModule, autodriver::SensorModule)
```

测试目标：`test_fake_module` 与 `test_sensor_manager` 都编译 `fake_module.cpp`，**不要**把该 cpp 链进 `libautodriver`。CreateClassObj 使用 `"FakeModule"`。

- [ ] **Step 4: 测试通过**

Expected: PASS（进程内 unmanaged 注册，`CreateUnmanagedClassObj` 路径）

- [ ] **Step 5: Commit**

```bash
git commit -m "feat(autodriver): add SensorModule plugin interface"
```

---

### Task 5: DeviceMonitor 空实现

**Files:**
- Create: `autodriver/autodriver/hotplug/device_monitor.hpp`
- Create: `autodriver/autodriver/hotplug/device_monitor.cpp`
- Create: `autodriver/test/test_device_monitor.cpp`

- [ ] **Step 1: 写失败测试**

```cpp
#include <gtest/gtest.h>
#include "autodriver/hotplug/device_monitor.hpp"

TEST(DeviceMonitor, NoopStartStop) {
    autodriver::NoopDeviceMonitor monitor;
    EXPECT_TRUE(monitor.Start([](autodriver::HotplugAction, const autodriver::UdevAttrs&) {}));
    monitor.Stop();
}
```

- [ ] **Step 2: 确认失败**

Expected: missing type.

- [ ] **Step 3: 实现**

```cpp
enum class HotplugAction { kAdd, kRemove };

class DeviceMonitor {
public:
    using Callback = std::function<void(HotplugAction, const UdevAttrs&)>;
    virtual ~DeviceMonitor() = default;
    virtual bool Start(Callback callback) = 0;
    virtual void Stop() = 0;
};

class NoopDeviceMonitor : public DeviceMonitor {
public:
    bool Start(Callback) override { return true; }
    void Stop() override {}
};

std::unique_ptr<DeviceMonitor> CreateDeviceMonitor(bool enable_udev);
```

`CreateDeviceMonitor`：非 Linux 或 `enable_udev==false` 返回 `NoopDeviceMonitor`。Linux 实现留到 Task 17，本任务在 Linux 上也先返回 Noop（`#if 0` 或 TODO 路径用 Noop），避免本任务引入 libudev。

- [ ] **Step 4: 测试通过**

Expected: PASS

- [ ] **Step 5: Commit**

```bash
git commit -m "feat(autodriver): add no-op device monitor"
```

---

### Task 6: CMake 接入 autolink 并打开测试

**Files:**
- Modify: `autodriver/autodriver/CMakeLists.txt`
- Modify: `autodriver/CMakeLists.txt`
- Modify: `autodriver/cmake/autodriver_embed.cmake`
- Modify: `autodriver/cmake/autodriver_install.cmake`

- [ ] **Step 1: 改 embed，不再 FORCE 关测试**

`autodriver_embed.cmake` 删除：

```cmake
set(AUTODRIVER_BUILD_TEST OFF CACHE BOOL "" FORCE)
set(AUTODRIVER_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
```

改为：

```cmake
# 沿用调用方 option；根工程打开 BUILD_TESTING 时编 autodriver 测试。
if(NOT DEFINED AUTODRIVER_BUILD_TEST)
  set(AUTODRIVER_BUILD_TEST ${BUILD_TESTING})
endif()
```

- [ ] **Step 2: 核心库链接 autolink / automsgs**

```cmake
target_link_libraries(autodriver PUBLIC Threads::Threads autolink automsgs)
```

去掉 `libautodriver` 对 `realsense2` 的 PUBLIC 链接（RealSense 延后到 imu/camera 插件）。`io/realsense_device.cpp` 暂时留在源列表会在无 SDK 时编不过：本任务把它从 `AUTODRIVER_SOURCES` **移除**（文件先留着，Task 13 再进 OBJECT 库）。`realsense_*_driver.cpp` 同样移出核心源列表（Task 12/13 再进插件）。本任务若移出后核心仍能编过（serial/can 仍在）。

在 `publish/channel.hpp` 增加 `MakeWriterAttr`（`autolink/proto/role_attributes.pb.h`）：`HISTORY_KEEP_LAST`、`depth=10`。

- [ ] **Step 3: 配置测试 include**

`autodriver_add_test` 增加：

```cmake
target_link_libraries(${NAME} PRIVATE autodriver GTest::gtest_main autolink)
```

- [ ] **Step 4: 从根目录编译**

```bash
cmake -S . -B build -DBUILD_AUTODRIVER=ON
cmake --build build -j --target autodriver test_autodriver_config test_default_channel test_udev_match
ctest --test-dir build -R 'test_(autodriver_config|default_channel|udev_match|fake_module|device_monitor)' --output-on-failure
```

Expected: PASS。`nm -C build/lib/libautodriver.so | grep mock` 应无输出（mock 仍在源列表则下一步删）。

- [ ] **Step 5: Commit**

```bash
git commit -m "build(autodriver): link autolink and enable tests when embedded"
```

---

### Task 7: SensorManager Attach / Detach

**Files:**
- Modify: `autodriver/autodriver/sensor_manager.hpp`
- Modify: `autodriver/autodriver/app/sensor_manager.cpp`
- Modify: `autodriver/test/test_sensor_manager.cpp`（整文件重写）

- [ ] **Step 1: 重写失败测试（替换 mock 用例）**

```cpp
#include <gtest/gtest.h>
#include "autodriver/sensor_manager.hpp"
#include "autolink/init.hpp"
#include "fake_module.hpp"

class SensorManagerTest : public ::testing::Test {
protected:
    static void SetUpTestSuite() { autolink::Init("test_sensor_manager"); }
};

TEST_F(SensorManagerTest, AttachIsIdempotent) {
    autodriver::AutodriverConfig config;
    autodriver::SensorConfig sensor;
    sensor.class_name = "FakeModule";
    sensor.sensor_id = "imu/test";
    sensor.attach_on_start = false;
    config.sensors = {sensor};
    autodriver::SensorManager manager(config);
    ASSERT_TRUE(manager.Initialize());
    EXPECT_TRUE(manager.Attach("imu/test"));
    EXPECT_TRUE(manager.Attach("imu/test"));
    EXPECT_EQ(manager.AttachedCount(), 1u);
    manager.Detach("imu/test");
    manager.Detach("imu/test");
    EXPECT_EQ(manager.AttachedCount(), 0u);
}

TEST_F(SensorManagerTest, UnknownIdFails) {
    autodriver::SensorManager manager(autodriver::AutodriverConfig{});
    ASSERT_TRUE(manager.Initialize());
    EXPECT_FALSE(manager.Attach("nope"));
}

TEST_F(SensorManagerTest, DuplicateConfigFailsInitialize) {
    autodriver::AutodriverConfig config;
    autodriver::SensorConfig a;
    a.sensor_id = "x";
    a.class_name = "FakeModule";
    config.sensors = {a, a};
    autodriver::SensorManager manager(config);
    EXPECT_FALSE(manager.Initialize());
}

TEST_F(SensorManagerTest, UnknownClassFailsAttach) {
    autodriver::AutodriverConfig config;
    autodriver::SensorConfig sensor;
    sensor.class_name = "DoesNotExist";
    sensor.sensor_id = "imu/x";
    config.sensors = {sensor};
    autodriver::SensorManager manager(config);
    ASSERT_TRUE(manager.Initialize());
    EXPECT_FALSE(manager.Attach("imu/x"));
}
```

- [ ] **Step 2: 跑测试确认失败**

Expected: `AttachedCount` / 新 `Initialize` 语义不存在，或旧 mock 测试先删掉导致链接 `mock` 失败——先把旧测试全部换成上面。

- [ ] **Step 3: 实现 SensorManager**

对外 API：

```cpp
class SensorManager {
public:
    explicit SensorManager(AutodriverConfig config);
    bool Initialize();
    bool Start();
    void Stop();
    bool Attach(const SensorId& sensor_id);
    void Detach(const SensorId& sensor_id);
    bool IsRunning() const;
    std::size_t AttachedCount() const;
    autolink::Node& node();
private:
    std::string ResolveLibraryPath(const SensorConfig& sensor) const;
    bool AttachLocked(const SensorId& sensor_id);
    void DetachLocked(const SensorId& sensor_id);
    void WorkerLoop();
    void Enqueue(HotplugAction action, const UdevAttrs& attrs);
    // ...
};
```

实现要点：

1. `Initialize`：`HasDuplicateSensorId` → false；`node_ = autolink::CreateNode(config_.node_name)`；`monitor_ = CreateDeviceMonitor(config_.hotplug.enable_udev)`；`monitor_->Start` 回调只 `Enqueue`；启动 `worker_` 从 `std::queue` 取事件，命中则 `AttachLocked`/`DetachLocked`。
2. `library` 为空：不 `LoadLibrary`，`ClassLoaderManager{}.CreateClassObj<SensorModule>(class_name)`（FakeModule 路径）。
3. `library` 非空：`ResolveLibraryPath` = `plugin_dir`（空则 `AUTODRIVER_PLUGIN_DIR` 编译宏）+ `/` + filename；`loaders_[path] = std::make_unique<ClassLoader>(path)`；`CreateClassObj<SensorModule>(class_name)`。
4. 先 `Init(context)` 再 `Start()`；任一步失败则 `Stop()`、丢弃 `shared_ptr`，不插入 `instances_`。
5. `Detach`：`Stop()`，reset 实例（必须先于销毁 `ClassLoader`），该 path 引用计数为 0 时 `loaders_.erase(path)`。
6. `Start()`：对 `attach_on_start` 的条目 `Attach`；全失败仍返回 true 若没有任何 `attach_on_start`（空配置进程可活）。
7. 日志：`AERROR` / `AINFO`。不抛出。
8. 可选 `alignment.enable`：本任务 tap 留空，Task 16 再接。

`context.node`、`context.config`、`context.tap` 空。

- [ ] **Step 4: 测试通过**

Expected: PASS

- [ ] **Step 5: Commit**

```bash
git commit -m "feat(autodriver): add Attach/Detach sensor manager"
```

---

### Task 8: 删除 mock 与旧工厂

**Files:**
- Delete: `autodriver/autodriver/drivers/mock/mock_drivers.hpp`
- Delete: `autodriver/autodriver/drivers/mock/mock_drivers.cpp`
- Delete: `autodriver/autodriver/hal/sensor_factory.hpp`
- Delete: `autodriver/autodriver/hal/sensor_factory.cpp`
- Delete: `autodriver/autodriver/drivers/driver_registry.hpp`
- Delete: `autodriver/autodriver/drivers/driver_registry.cpp`
- Modify: `autodriver/autodriver/config/hub_config.hpp`（删除 `DefaultSimulationHubConfig` / `register_builtin_mocks`，或整文件改为 `#include` 新 config 的兼容转发；**推荐直接删除** `hub_config.*`，所有引用改 `autodriver_config.hpp`）
- Delete: `autodriver/autodriver/config/hub_config.cpp`
- Modify: `autodriver/test/test_sensor_hub.cpp`（去掉 mock；本任务先删该测试文件，Task 16 用 FakeModule 重写）
- Modify: `autodriver/autodriver/CMakeLists.txt` 源列表
- Modify: `autodriver/examples/demo_main.cpp`（临时改成空配置 Manager Initialize+Start+Stop，避免链接 mock）

- [ ] **Step 1: 全局搜索 mock**

```bash
rg -n "mock_|DefaultSimulationHubConfig|SensorFactory|RegisterBuiltin" autodriver config
```

Expected: 多处命中。逐处删改到零命中（除本 plan/spec）。

- [ ] **Step 2: 从 CMake 去掉已删源文件并编译**

```bash
cmake --build build -j --target autodriver test_sensor_manager
```

Expected: 链接成功。

- [ ] **Step 3: 确认生产库无 mock 符号**

```bash
nm -C build/lib/libautodriver.so | grep -i mock || true
```

Expected: 无 `CreateMock` / `mock_imu`。

- [ ] **Step 4: 跑仍存在的测试**

```bash
ctest --test-dir build -R 'test_(sensor_manager|nmea0183|wit_motion)' --output-on-failure
```

Expected: PASS

- [ ] **Step 5: Commit**

```bash
git commit -m "refactor(autodriver): remove mock drivers and SensorFactory"
```

---

### Task 9: sample_converter 迁入核心

**Files:**
- Create: `autodriver/autodriver/convert/sample_converter.hpp`（内容来自 `bridge/autonomy/sample_converter.hpp`，命名空间改为 `autodriver::convert`，include 改为 automsgs）
- Create: `autodriver/autodriver/convert/sample_converter.cpp`
- Create: `autodriver/test/test_sample_converter.cpp`
- Modify: `autodriver/sample_converter.hpp` 改为 `#include "autodriver/convert/sample_converter.hpp"` 并 `namespace bridge { using convert::ToAutonomyImu; ... }` 以免立刻打碎 collator
- Modify: `autodriver/bridge/autonomy/CMakeLists.txt` 去掉重复编译的 `sample_converter.cpp`（改链接 autodriver）

- [ ] **Step 1: 转换单测（手造 sample）**

```cpp
#include <gtest/gtest.h>
#include "autodriver/convert/sample_converter.hpp"
#include "autodriver/types/imu_sample.hpp"

TEST(SampleConverter, ImuFrameIdAndAccel) {
    autodriver::ImuSample sample("imu/a", autodriver::Timestamp{},
                                 {0, 0, 1.2}, {0, 0, 9.8});
    const auto msg = autodriver::convert::ToAutonomyImu(sample);
    EXPECT_EQ(msg.header().frame_id(), "imu/a");
    EXPECT_DOUBLE_EQ(msg.linear_acceleration().z(), 9.8);
    EXPECT_DOUBLE_EQ(msg.angular_velocity().z(), 1.2);
}
```

再加 `Range` / `NavSatFix` 各一条（`GpsSample` / `RangeSample` 字段按现有类型头）。

- [ ] **Step 2: 确认失败**

Expected: `convert/sample_converter.hpp` missing.

- [ ] **Step 3: 移动实现**

`git mv` 后改 namespace。`MakeHeader` 继续用 `host_time()` 与 `sensor_id`。核心库 CMake 加入 `convert/sample_converter.cpp`，并 PUBLIC 链 `automsgs`（Task 6 已做）。

- [ ] **Step 4: 测试通过**

Expected: PASS

- [ ] **Step 5: Commit**

```bash
git commit -m "refactor(autodriver): move sample converter into core"
```

---

### Task 10: Lidar / Range 骨架插件

**Files:**
- Create: `autodriver/plugins/CMakeLists.txt`
- Create: `autodriver/plugins/lidar/lidar_module.cpp`
- Create: `autodriver/plugins/range/range_module.cpp`
- Modify: `autodriver/CMakeLists.txt` `add_subdirectory(plugins)`
- Modify: `autodriver/cmake/autodriver_install.cmake` 安装两个 `.so`

- [ ] **Step 1: 写插件加载测试（先会失败：没有 .so）**

`autodriver/test/test_plugin_lidar.cpp`：

```cpp
#include <cstdlib>
#include <gtest/gtest.h>
#include "autodriver/sensor_module.hpp"
#include "autolink/class_loader/class_loader.hpp"
#include "autolink/init.hpp"
#include "automsgs/msgs/sensor_msgs/laser_scan.pb.h"

TEST(LidarPlugin, StartsAndPublishesNothing) {
    autolink::Init("test_plugin_lidar");
    const char* dir = std::getenv("AUTODRIVER_PLUGIN_DIR");
    ASSERT_NE(dir, nullptr);
    const std::string path = std::string(dir) + "/libautodriver_lidar.so";
    autolink::class_loader::ClassLoader loader(path);
    ASSERT_TRUE(loader.IsLibraryLoaded());
    auto module = loader.CreateClassObj<autodriver::SensorModule>("LidarModule");
    ASSERT_NE(module, nullptr);
    autodriver::SensorModuleContext ctx;
    ctx.node = autolink::CreateNode("test_lidar_plugin");
    ctx.config.sensor_id = "lidar/front";
    ctx.config.channel = "/scan";
    ASSERT_TRUE(module->Init(ctx));
    ASSERT_TRUE(module->Start());

    std::atomic<int> n{0};
    auto reader = ctx.node->CreateReader<automsgs::msgs::sensor_msgs::LaserScan>(
        "/scan", [&](const auto&) { n.fetch_add(1); });
    ASSERT_NE(reader, nullptr);
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    module->Stop();
    EXPECT_EQ(n.load(), 0);
}
```

CMake 对该测试：

```cmake
target_compile_definitions(test_plugin_lidar PRIVATE
  AUTODRIVER_PLUGIN_DIR="${CMAKE_LIBRARY_OUTPUT_DIRECTORY}")
set_tests_properties(test_plugin_lidar PROPERTIES
  ENVIRONMENT "AUTODRIVER_PLUGIN_DIR=${CMAKE_LIBRARY_OUTPUT_DIRECTORY}")
```

- [ ] **Step 2: 跑测试确认失败**

Expected: `libautodriver_lidar.so` 无法加载。

- [ ] **Step 3: 实现骨架与 CMake**

`plugins/CMakeLists.txt`：

```cmake
function(autodriver_add_plugin target)
  add_library(${target} SHARED ${ARGN})
  target_link_libraries(${target} PRIVATE autodriver)
  target_include_directories(${target} PRIVATE
    ${AUTODRIVER_ROOT_DIR}/autodriver)
  set_target_properties(${target} PROPERTIES
    OUTPUT_NAME ${target}
    LIBRARY_OUTPUT_DIRECTORY ${CMAKE_LIBRARY_OUTPUT_DIRECTORY})
endfunction()

add_subdirectory(lidar)
add_subdirectory(range)
```

`lidar_module.cpp`：类必须在**具名**命名空间或全局，禁止匿名命名空间（否则 class_loader 静态注册在部分工具链上不可见）。

```cpp
#include "autodriver/sensor_module.hpp"
#include "autodriver/publish/channel.hpp"
#include "autodriver/types/sensor_type.hpp"
#include "autolink/class_loader/class_loader_register_macro.hpp"
#include "autolink/common/log.hpp"
#include "autolink/node/writer.hpp"
#include <automsgs/msgs/sensor_msgs/laser_scan.pb.h>

class LidarModule : public autodriver::SensorModule {
public:
    autodriver::SensorType GetType() const override {
        return autodriver::SensorType::kLidar;
    }
    const autodriver::SensorId& GetSensorId() const override {
        return sensor_id_;
    }
    bool Init(const autodriver::SensorModuleContext& context) override {
        if (!context.node) {
            AERROR << "LidarModule Init: null node";
            return false;
        }
        sensor_id_ = context.config.sensor_id;
        const std::string channel = autodriver::ResolveChannel(
            context.config.channel, autodriver::SensorType::kLidar);
        writer_ = context.node->CreateWriter<automsgs::msgs::sensor_msgs::LaserScan>(
            autodriver::MakeWriterAttr(channel));
        if (!writer_) {
            AERROR << "LidarModule failed to create writer on " << channel;
            return false;
        }
        return true;
    }
    bool Start() override {
        running_ = true;
        return true;
    }
    void Stop() override {
        running_ = false;
        writer_.reset();
    }
    bool IsRunning() const override { return running_; }

private:
    autodriver::SensorId sensor_id_;
    std::shared_ptr<autolink::Writer<automsgs::msgs::sensor_msgs::LaserScan>>
        writer_;
    bool running_ = false;
};

CLASS_LOADER_REGISTER_CLASS(LidarModule, autodriver::SensorModule)
```

Range 同样，Writer 类型 `Range`，channel `/range`。类名 `RangeModule`。**禁止**写任何假 ranges。

`core` 编译定义：

```cmake
target_compile_definitions(autodriver PUBLIC
  AUTODRIVER_PLUGIN_DIR="${CMAKE_LIBRARY_OUTPUT_DIRECTORY}")
```

- [ ] **Step 4: 测试通过**

Expected: PASS，0 条 LaserScan。

- [ ] **Step 5: Commit**

```bash
git commit -m "feat(autodriver): add lidar and range skeleton plugins"
```

---

### Task 11: GPS 插件

**Files:**
- Move: `serial_gps_driver.*`、`can_gps_driver.*` → `autodriver/plugins/gps/`
- Create: `autodriver/plugins/gps/gps_module.cpp`
- Create: `autodriver/plugins/gps/CMakeLists.txt`
- Modify: 核心 CMake 去掉这两个驱动源文件

- [ ] **Step 1: 未知 backend 单测（进程内无法加载 .so 时，用一个最小 GpsModule 测试文件？）**

不要把 GpsModule 链进核心。本任务用插件加载测 `backend` 失败：

```cpp
TEST(GpsPlugin, UnknownBackendFailsInit) {
    autolink::Init("test_plugin_gps");
    autolink::class_loader::ClassLoader loader(Plugin("libautodriver_gps.so"));
    auto module = loader.CreateClassObj<autodriver::SensorModule>("GpsModule");
    autodriver::SensorModuleContext ctx;
    ctx.node = autolink::CreateNode("test_gps");
    ctx.config.sensor_id = "gps/x";
    ctx.config.backend = "not-a-bus";
    EXPECT_FALSE(module->Init(ctx));
}
```

`Plugin()` 读 `AUTODRIVER_PLUGIN_DIR`。先写测试再实现。

- [ ] **Step 2: 确认失败**

Expected: 无 `libautodriver_gps.so`。

- [ ] **Step 3: GpsModule**

`Init`：

- `backend=="serial"` → `driver_ = std::make_shared<hardware::SerialGpsDriver>(id, params)`
- `backend=="can"` → `CanGpsDriver`
- 其它 → `AERROR` 返回 false
- `CreateWriter<NavSatFix>(MakeWriterAttr(ResolveChannel(...)))`
- `driver_->SetSampleCallback`：`ToAutonomyNavSatFix` 后 `writer_->Write`；`Write` 失败只计数；若 `tap` 非空则 `tap(*sample)`（在 convert 之前对 HAL 对象）

`Start`/`Stop` 转发给 driver，`Stop` 时 `writer_.reset()`。

硬件驱动 include 路径若因 mv 变化，改相对 include 为 `autodriver/plugins/gps/serial_gps_driver.hpp` 或保持 `autodriver/drivers/...` 并 **不要 mv 头文件安装路径**——更简单：**驱动文件留在 `autodriver/drivers/`，仅不编进 libautodriver，而编进 `libautodriver_gps`。** 这样少改 include。本任务采用此策略（不 git mv 源文件，只改 CMake 归属）。

`plugins/gps/CMakeLists.txt`：

```cmake
autodriver_add_plugin(autodriver_gps
  gps_module.cpp
  ${AUTODRIVER_ROOT_DIR}/autodriver/drivers/serial_gps_driver.cpp
  ${AUTODRIVER_ROOT_DIR}/autodriver/drivers/can_gps_driver.cpp
)
```

从 `AUTODRIVER_SOURCES` 删除这两份 cpp。

- [ ] **Step 4: 测试通过**（未知 backend；有串口设备的环境不作为 CI 必过）

Expected: PASS

- [ ] **Step 5: Commit**

```bash
git commit -m "feat(autodriver): add GPS class_loader plugin"
```

---

### Task 12: IMU 插件

**Files:**
- Create: `autodriver/plugins/imu/imu_module.cpp`
- Create: `autodriver/plugins/imu/CMakeLists.txt`
- Modify: 核心源列表去掉 `serial_imu_driver.cpp`、`can_imu_driver.cpp`（`realsense_imu_driver.cpp` 若仍在核心也去掉）

- [ ] **Step 1: 未知 backend 测试**（同 GPS，类名 `ImuModule`，库 `libautodriver_imu.so`）

- [ ] **Step 2: 确认失败**

Expected: 库不存在。

- [ ] **Step 3: ImuModule**

backend：`serial` → `SerialImuDriver`；`can` → `CanImuDriver`；`realsense` → 若 `AUTODRIVER_HAVE_REALSENSE` 则 `RealSenseImuDriver`，否则 `AERROR` 且 `Init` false。

发布 `ToAutonomyImu` → `Writer<Imu>`，默认 channel `/imu`。

`plugins/imu/CMakeLists.txt` 先不链 realsense（Task 13 再加 `realsense_imu_driver.cpp` + OBJECT）。本任务 serial+can 即可。

- [ ] **Step 4: 测试通过**

Expected: PASS

- [ ] **Step 5: Commit**

```bash
git commit -m "feat(autodriver): add IMU class_loader plugin"
```

---

### Task 13: Camera 插件与 RealSense OBJECT 库

**Files:**
- Create: `autodriver/plugins/camera/camera_module.cpp`
- Create: `autodriver/plugins/camera/CMakeLists.txt`
- Modify: `autodriver/plugins/imu/CMakeLists.txt`（realsense backend）
- Modify: `autodriver/autodriver/CMakeLists.txt` 定义 `autodriver_realsense_io` OBJECT
- Create: `autodriver/test/test_plugin_camera.cpp`（未知/未编译 realsense 时 Init false）

- [ ] **Step 1: 测试 Camera 非 realsense backend 失败；无 SDK 时 realsense 也失败**

```cpp
TEST(CameraPlugin, RejectsUnknownBackend) {
    autolink::Init("test_plugin_camera");
    autolink::class_loader::ClassLoader loader(Plugin("libautodriver_camera.so"));
    auto module = loader.CreateClassObj<autodriver::SensorModule>("CameraModule");
    autodriver::SensorModuleContext ctx;
    ctx.node = autolink::CreateNode("test_camera");
    ctx.config.sensor_id = "camera/x";
    ctx.config.backend = "v4l2";
    EXPECT_FALSE(module->Init(ctx));
}
```

- [ ] **Step 2: 确认失败**

Expected: 无 camera .so。

- [ ] **Step 3: OBJECT 库与 CameraModule**

```cmake
if(AUTODRIVER_WITH_REALSENSE)
  find_package(realsense2 QUIET)
endif()

if(AUTODRIVER_WITH_REALSENSE AND realsense2_FOUND)
  add_library(autodriver_realsense_io OBJECT
    io/realsense_device.cpp
    drivers/hardware/realsense_types.cpp)
  target_link_libraries(autodriver_realsense_io PUBLIC realsense2::realsense2 autodriver)
  target_compile_definitions(autodriver_realsense_io PUBLIC AUTODRIVER_HAVE_REALSENSE)
endif()
```

`CameraModule`：仅 `backend=="realsense"`；`params["stream"]=="depth"` 用 depth channel 与 `CreateRealSenseCameraDriver`；否则 color。`publish_camera_info` 从 params 读 `"true"` 时再 `CreateWriter<CameraInfo>`（本轮若现有 `CameraFrame` 无完整 CameraInfo，可只建 Writer 不发，直到 frame 带标定；有数据再 `Write`）。图像走 `ToAutonomyImage`。

IMU 插件在 `realsense2_FOUND` 时额外编译 `realsense_imu_driver.cpp` 并链 OBJECT。

- [ ] **Step 4: 测试通过**

Expected: PASS

- [ ] **Step 5: Commit**

```bash
git commit -m "feat(autodriver): add camera plugin and RealSense object lib"
```

---

### Task 14: Lua 配置与 loader

**Files:**
- Modify: `autodriver/autodriver/bridge/autonomy/hub_config_loader.hpp` 改为 `LoadAutodriverConfigFromDirectory` 返回 `AutodriverConfig`
- Modify: `autodriver/autodriver/bridge/autonomy/hub_config_loader.cpp`
- Modify: `config/driver/autodriver.lua`
- Modify: `config/driver/autodriver_hardware.lua`
- Modify: `config/driver/autodriver_realsense.lua`

- [ ] **Step 1: 空表加载测试**（若 loader 依赖 autonomy Lua，测试放在 `autodriver_autonomy_bridge` 能编时；否则用手写 `AutodriverConfig` 填字段的单元测试覆盖解析函数抽出的纯逻辑）

把字典解析抽成：

```cpp
AutodriverConfig LoadAutodriverConfigFromDictionary(
    autonomy::common::LuaParameterDictionary* dict);
```

用最小 Lua 字符串测（需要 LuaParameterDictionary）。若桥目标在 CI 默认不开，本任务至少改 lua 文件，解析在 `hub_config_loader.cpp` 按 spec 第 7 节键名实现，并在有 `autonomy` target 时编译。

解析：`sensors` 数组；每项 `class_name`/`library`/`sensor_id`/`channel`/`backend`/`attach_on_start`；`match` 子表；`params` 沿用现有 string/int/double 分支。删除 `register_builtin_mocks`、`factory_name`、`drivers`。

- [ ] **Step 2: 默认 lua**

```lua
AUTODRIVER = {
  node_name = "autodriver",
  plugin_dir = "",
  hotplug = { enable_udev = true },
  alignment = { enable = false },
  sensors = {},
}
return AUTODRIVER
```

`autodriver_hardware.lua` / `autodriver_realsense.lua` 改成 spec 示例字段（`ImuModule` + `libautodriver_imu.so` 等）。

- [ ] **Step 3: 编译桥与 hub**

根 `CMakeLists.txt` 里 `autodriver_hub` 仍链 `autodriver_autonomy_bridge`。改 `hub_main.cpp` 用 `AutodriverConfig` + `SensorManager`，去掉 `SetAlignedCallback` 对 mock IMU 的打印（无传感器时循环等待信号）。

- [ ] **Step 4: 空配置可启动**（手动或短测：Initialize+Start+Stop 返回成功）

- [ ] **Step 5: Commit**

```bash
git commit -m "feat(autodriver): switch Lua config to plugin sensors"
```

---

### Task 15: 示例

**Files:**
- Modify: `autodriver/examples/demo_main.cpp`
- Modify: `autodriver/examples/hub_main.cpp`
- Modify: `autodriver/CMakeLists.txt`（`autodriver_demo` 链 autolink；`AUTODRIVER_BUILD_EXAMPLES`）

- [ ] **Step 1: demo 加载 Lidar 骨架**

```cpp
int main(int argc, char** argv) {
    autolink::Init(argv[0]);
    autodriver::AutodriverConfig config;
    autodriver::SensorConfig lidar;
    lidar.class_name = "LidarModule";
    lidar.library = "libautodriver_lidar.so";
    lidar.sensor_id = "lidar/front";
    lidar.attach_on_start = true;
    config.sensors = {lidar};
    autodriver::SensorManager manager(std::move(config));
    if (!manager.Initialize() || !manager.Start()) {
        return 1;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    manager.Stop();
    autolink::Clear();
    return 0;
}
```

- [ ] **Step 2: 运行 demo**

```bash
cmake --build build -j --target autodriver_demo
./build/bin/autodriver_demo
```

Expected: 退出码 0，无 mock 日志。

- [ ] **Step 3: hub_main 用 loader**（`LoadAutodriverConfigFromDirectory`）+ SIGINT 循环，与现结构相同但不读 aligned IMU。

- [ ] **Step 4: Commit**

```bash
git commit -m "feat(autodriver): wire demo and hub to plugin manager"
```

---

### Task 16: 可选对齐旁路

**Files:**
- Modify: `autodriver/autodriver/sync/sensor_hub.hpp` 增加 `void PushSample(std::unique_ptr<SensorSample> sample);`
- Modify: `autodriver/autodriver/sync/sensor_hub.cpp`：`PushSample` 调用现有 `OnSample`；`Start()` 在 `drivers_` 为空时只起 alignment 线程
- Modify: `autodriver/autodriver/app/sensor_manager.cpp`：`alignment.enable` 时构造 `SensorHub`，`tap` = `hub.PushSample(sample.Clone())`
- Create: `autodriver/test/test_sensor_hub.cpp`（旧文件已删则新建）

- [ ] **Step 1: FakeModule 推 sample 的测试**

给 `FakeModule` 增加测试专用 `PublishOnce()`（仅测试头文件，生产插件没有）。或测试直接 `hub.PushSample(make_unique<ImuSample>(...))`，不经过 FakeModule：

```cpp
TEST(SensorHub, AlignsPushedImu) {
    autodriver::SensorHubOptions options;
    options.publish_period = std::chrono::milliseconds(10);
    options.alignment_window = std::chrono::milliseconds(200);
    autodriver::SensorHub hub(options);
    std::atomic<int> n{0};
    hub.SetAlignedCallback([&](const autodriver::AlignedSnapshot& snap) {
        ++n;
        EXPECT_NE(snap.Get<autodriver::ImuSample>(autodriver::SensorType::kImu),
                  nullptr);
    });
    ASSERT_TRUE(hub.Start());
    hub.PushSample(std::make_unique<autodriver::ImuSample>(
        "imu/t", autodriver::Now(), std::array<double, 3>{}, std::array<double, 3>{}));
    std::this_thread::sleep_for(std::chrono::milliseconds(80));
    hub.Stop();
    EXPECT_GT(n.load(), 0);
}
```

需要 `Now()`：`autodriver/time.hpp` 已有则用，否则 `Timestamp{}` 加上 buffer 逻辑仍可能对齐。

- [ ] **Step 2: 确认失败**

Expected: `PushSample` 未声明。

- [ ] **Step 3: 实现 PushSample；Manager 在 enable 时 `hub_.Start()` 并把 tap 传入 context**

发布主路径：模块仍先 `Write` 再 tap（或先 tap 再 Write，但 **不得**因 tap 异常中断 Write——tap 包 try/catch 打 `AERROR`）。

- [ ] **Step 4: 测试通过**

Expected: PASS

- [ ] **Step 5: Commit**

```bash
git commit -m "feat(autodriver): optional alignment tap off publish path"
```

---

### Task 17: Linux udev DeviceMonitor

**Files:**
- Modify: `autodriver/autodriver/hotplug/device_monitor.cpp` 或拆 `udev_device_monitor.cpp`
- Modify: `autodriver/autodriver/CMakeLists.txt`：`if(UNIX AND NOT APPLE) find_library(UDEV udev)` 并 `target_link_libraries(autodriver PRIVATE ${UDEV})` + `AUTODRIVER_HAVE_UDEV`
- Modify: `autodriver/test/test_device_monitor.cpp` 增加注入：给 `UdevDeviceMonitor` 加测试可见的 `InjectForTest` **不要**。改为 Manager 已有 Enqueue：新增 `test_hotplug_queue.cpp` 测「attrs → Attach FakeModule」通过 **public 测试缝**。

最小缝：`SensorManager` 增加

```cpp
void HandleDeviceEvent(HotplugAction action, const UdevAttrs& attrs);
```

生产里 udev 回调只调用它（已入队则 Handle 在 worker 上执行）。测试直接 `HandleDeviceEvent`（若只在测试编译暴露，用 `#ifdef AUTODRIVER_TEST` 或直接 public——文档写明仅测试/监视器使用）。

- [ ] **Step 1: 测试匹配后 Attach**

```cpp
TEST_F(SensorManagerTest, DeviceAddAttachesMatchedSensor) {
    autodriver::AutodriverConfig config;
    autodriver::SensorConfig sensor;
    sensor.class_name = "FakeModule";
    sensor.sensor_id = "imu/serial";
    sensor.match.subsystem = "tty";
    sensor.match.devnode = "/dev/ttyUSB0";
    config.sensors = {sensor};
    autodriver::SensorManager manager(config);
    ASSERT_TRUE(manager.Initialize());
    autodriver::UdevAttrs attrs;
    attrs.subsystem = "tty";
    attrs.devnode = "/dev/ttyUSB0";
    manager.HandleDeviceEvent(autodriver::HotplugAction::kAdd, attrs);
    EXPECT_EQ(manager.AttachedCount(), 1u);
    manager.HandleDeviceEvent(autodriver::HotplugAction::kRemove, attrs);
    EXPECT_EQ(manager.AttachedCount(), 0u);
}
```

- [ ] **Step 2: 确认失败**

Expected: `HandleDeviceEvent` missing.

- [ ] **Step 3: 实现入队 + Linux udev**

Worker 已有则 `HandleDeviceEvent` push 队列。Linux：`udev_monitor_new_from_netlink`，线程里 `udev_monitor_receive_device`，填 `UdevAttrs`（`udev_device_get_subsystem` / `get_devnode` / `idVendor` / `idProduct` / `serial`）。`CreateDeviceMonitor(true)` 在 `__linux__ && AUTODRIVER_HAVE_UDEV` 返回该实现。找不到 libudev：编译 Noop 并 `AWARN`。

macOS：不链 udev，行为仍 Noop。

- [ ] **Step 4: 测试通过**（HandleDeviceEvent 在全平台；真实拔插不作为 CI）

Expected: PASS

- [ ] **Step 5: Commit**

```bash
git commit -m "feat(autodriver): add Linux udev hotplug monitor"
```

---

### Task 18: 桥接、安装、验收

**Files:**
- Modify: `autodriver/bridge/autonomy/collator_sink.*`（include 新 converter；类型名 `DriverConfig` 若仍引用则改掉）
- Modify: `autodriver/cmake/autodriver_install.cmake` 安装 `autodriver_imu` 等 SHARED
- Modify: `autodriver/examples/demo_main.cpp` 已完成则跳过
- `python3 scripts/format.py autodriver`

- [ ] **Step 1: 编译全目标**

```bash
cmake --build build -j --target autodriver autodriver_imu autodriver_gps \
  autodriver_camera autodriver_lidar autodriver_range autodriver_demo autodriver_hub
```

Expected: 成功（camera 在无 realsense 时仍能编出插件，Init(realsense) 运行期失败）。

- [ ] **Step 2: 全量相关 ctest**

```bash
ctest --test-dir build -R 'test_(autodriver_config|default_channel|udev_match|fake_module|device_monitor|sensor_manager|plugin_lidar|plugin_gps|plugin_imu|plugin_camera|sample_converter|sensor_hub|nmea0183|wit_motion|realsense_types|time_sync)' --output-on-failure
```

Expected: PASS

- [ ] **Step 3: 符号与风格**

```bash
nm -C build/lib/libautodriver.so | grep -Ei 'mock_|DefaultSimulation' && exit 1 || true
python3 scripts/format.py autodriver
```

Expected: 无 mock 符号；format 无额外改动或再提交 format。

- [ ] **Step 4: 对照 spec 第 11 节打勾**

1. 生产无 `mock_*` / `DefaultSimulationHubConfig`
2. 五个插件可 LoadLibrary；Lidar/Range 零样本
3. IMU/GPS/Camera 代码路径存在（真机不在 CI）
4. Attach/Detach 幂等；HandleDeviceEvent 覆盖 udev 语义
5. macOS 不链 libudev
6. 协议单测 + 新测试通过

- [ ] **Step 5: Commit**

```bash
git commit -m "chore(autodriver): format and install sensor plugins"
```

---

## Self-review（对照 spec）

| Spec | 任务 |
|------|------|
| §1 五模态、autolink 发布、删 mock | 8, 10–13, 15 |
| §1.2 无轮速插件、无真 Lidar/Range、无 macOS IOKit、无 Component | 未列入 |
| §2 SensorModule + Manager + Monitor + 可选 Hub | 4, 5, 7, 16, 17 |
| §3 目录、class_loader 类名、realsense OBJECT、collator 仍可选 | 10–14, 18 |
| §4 Init/Start/Stop、幂等 Attach、空 library 测试路径 | 4, 7 |
| §5 channel / proto | 2, 9–13 |
| §6 match 字段、无 match 不参与 udev | 3, 17 |
| §7 Lua 键名 | 14 |
| §8 错误不 abort、Write 丢帧 | 7, 10–13 |
| §9 FakeModule、骨架 0 消息、Match 单测 | 3, 4, 10 |
| §10 Google format | 18 |
| §11 成功标准 | 18 |

`ClassLoaderManager::UnloadLibrary` private：Task 7 用自持 `ClassLoader`，与 spec「最后实例 unload」一致。
