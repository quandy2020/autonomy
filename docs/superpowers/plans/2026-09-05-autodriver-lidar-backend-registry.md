# autodriver Lidar Backend Registry Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 为 lidar_3d 增加进程内静态后端注册表，Velodyne 自注册；`Lidar3dModule` 只查表；proto 增加 `VelodyneLidarConf` 并摊平进 `params`。

**Architecture:** `LidarBackendRegistry` 单例 map + `REGISTER_LIDAR_BACKEND` 静态注册；厂商 TU 链入 `libautodriver`；`ConfigFromProto` 将 `lidar_vendor.velodyne` 写入 `DriverParams`（已有 params 键优先）。

**Tech Stack:** C++17, GTest, protobuf, yaml-cpp, existing SensorDriver / SensorPlugin

**Spec:** `docs/superpowers/specs/2026-09-05-autodriver-lidar-backend-registry-design.md`

## Global Constraints

- 中文回复用户；不主动 `git commit`（除非用户明确要求）
- 不引入 Cyber DAG；不拆 Driver/Convert/Compensator
- 不实现 Hesai Convert；仅注册表与 Velodyne typed conf
- YAML `backend` + `params` 行为不变
- 新头文件路径与目录一致：`autodriver/lidar/...`

## File map

| 文件 | 职责 |
|------|------|
| Create `autodriver/lidar/backend_registry.hpp` | Registry API |
| Create `autodriver/lidar/backend_registry.cpp` | 单例 map + Create/Register |
| Create `autodriver/lidar/backend_register.hpp` | `REGISTER_LIDAR_BACKEND` 宏 |
| Modify `autodriver/lidar/velodyne/udp_driver.cpp` | 文件末尾注册 velodyne + udp |
| Modify `autodriver/modules.cpp` | `MakeDriver` → `Registry::Create` |
| Modify `proto/autodriver_conf.proto` | `VelodyneLidarConf` + oneof |
| Modify `autodriver/config_proto.cpp` | FromProto 摊平；ToProto 尽力回填 |
| Modify `autodriver/CMakeLists.txt` | 加入 `backend_registry.cpp` |
| Create `test/test_lidar_backend_registry.cpp` | 注册表单测 |
| Modify `test/test_config_proto.cpp` | Velodyne conf 摊平 |
| Modify `docs/source/guide/backends.md` 等 | 扩展步骤 |
| Modify `docs/superpowers/plans/2026-09-05-autodriver-apollo-drivers-align.md` | 勾选/备注 |

---

### Task 1: LidarBackendRegistry + 单测

**Files:**
- Create: `autodriver/autodriver/lidar/backend_registry.hpp`
- Create: `autodriver/autodriver/lidar/backend_registry.cpp`
- Create: `autodriver/autodriver/lidar/backend_register.hpp`
- Create: `autodriver/test/test_lidar_backend_registry.cpp`
- Modify: `autodriver/autodriver/CMakeLists.txt`（加入 `lidar/backend_registry.cpp`）
- Modify: `autodriver/CMakeLists.txt`（`autodriver_add_test(test_lidar_backend_registry …)`）

**Interfaces:**
- Produces:
  - `using LidarDriverFactory = std::function<std::shared_ptr<SensorDriver>(const SensorId&, const hardware::DriverParams&)>`
  - `class LidarBackendRegistry` with `static LidarBackendRegistry& Instance()`
  - `void Register(const std::string& name, LidarDriverFactory factory)`
  - `void RegisterAlias(const std::string& alias, const std::string& canonical)`
  - `std::shared_ptr<SensorDriver> Create(const std::string& backend, const SensorId& id, const hardware::DriverParams& params) const`
  - `bool Has(const std::string& backend) const`
  - Macro `REGISTER_LIDAR_BACKEND(tag, name, factory_fn, ...)` — `tag` 为唯一 C++ 标识符；`name` 字符串主名；其余为可选 alias 字符串

- [ ] **Step 1: 写失败单测**

```cpp
#include <gtest/gtest.h>
#include "autodriver/lidar/backend_registry.hpp"
#include "autodriver/sensor_driver.hpp"

namespace {
struct DummyDriver : autodriver::SensorDriver {
  autodriver::SensorType GetType() const override {
    return autodriver::SensorType::kLidar3d;
  }
  const autodriver::SensorId& GetSensorId() const override { return id_; }
  bool Start() override { return true; }
  void Stop() override {}
  bool IsRunning() const override { return false; }
  void SetSampleCallback(autodriver::SampleCallback) override {}
  autodriver::SensorId id_{"lidar/dummy"};
};
}  // namespace

TEST(LidarBackendRegistry, RegisterAndCreate) {
  auto& reg = autodriver::lidar::LidarBackendRegistry::Instance();
  reg.Register("fake_lidar_ut", [](const autodriver::SensorId& id,
                                   const autodriver::hardware::DriverParams&) {
    auto d = std::make_shared<DummyDriver>();
    d->id_ = id;
    return d;
  });
  reg.RegisterAlias("fake_alias_ut", "fake_lidar_ut");
  EXPECT_TRUE(reg.Has("fake_lidar_ut"));
  EXPECT_TRUE(reg.Has("fake_alias_ut"));
  autodriver::hardware::DriverParams params;
  auto driver = reg.Create("fake_alias_ut", "lidar/x", params);
  ASSERT_NE(driver, nullptr);
  EXPECT_EQ(driver->GetSensorId(), "lidar/x");
  EXPECT_EQ(reg.Create("no_such_backend_ut", "lidar/y", params), nullptr);
}
```

- [ ] **Step 2: 实现 registry 头/源与宏**

`backend_registry.hpp` 核心：

```cpp
class LidarBackendRegistry {
 public:
  static LidarBackendRegistry& Instance();
  void Register(const std::string& name, LidarDriverFactory factory);
  void RegisterAlias(const std::string& alias, const std::string& canonical);
  std::shared_ptr<SensorDriver> Create(
      const std::string& backend, const SensorId& id,
      const hardware::DriverParams& params) const;
  bool Has(const std::string& backend) const;
 private:
  std::string Resolve(const std::string& backend) const;
  mutable std::mutex mutex_;
  std::unordered_map<std::string, LidarDriverFactory> factories_;
  std::unordered_map<std::string, std::string> aliases_;
};
```

`backend_register.hpp`：

```cpp
#define REGISTER_LIDAR_BACKEND(tag, name, factory, ...) \
  namespace { \
  struct LidarBackendRegistrar_##tag { \
    LidarBackendRegistrar_##tag() { \
      auto& reg = ::autodriver::lidar::LidarBackendRegistry::Instance(); \
      reg.Register(name, factory); \
      const char* aliases[] = {__VA_ARGS__}; \
      for (const char* a : aliases) { \
        if (a != nullptr && a[0] != '\0') { \
          reg.RegisterAlias(a, name); \
        } \
      } \
    } \
  }; \
  static LidarBackendRegistrar_##tag g_lidar_backend_registrar_##tag; \
  }
```

注意：无 alias 时写成 `REGISTER_LIDAR_BACKEND(velodyne, "velodyne", CreateFn)` — `__VA_ARGS__` 空则 `aliases[]` 为空数组，循环不执行。若编译器对空 `__VA_ARGS__` 挑剔，提供两套宏或尾部哑元 `""`。

实现 `Create`：加锁 → `Resolve`（若在 aliases_ 则取 canonical）→ 查 `factories_` → 调用；失败打 `AERROR`。

- [ ] **Step 3: CMake 加入源与测试 target**

- [ ] **Step 4: 编译并跑 `test_lidar_backend_registry`**

Expected: PASS（若全工程 cmake 不可用，至少保证源文件自洽；有 build 则跑该测）

- [ ] **Step 5: 不 commit**（除非用户要求）

---

### Task 2: Velodyne 自注册 + Lidar3dModule 查表

**Files:**
- Modify: `autodriver/autodriver/lidar/velodyne/udp_driver.cpp`（末尾）
- Modify: `autodriver/autodriver/modules.cpp`（Lidar3dModule）
- Modify: `autodriver/test/test_stream_lidar_base.cpp`（可选：断言 `Has("velodyne")`）

**Interfaces:**
- Consumes: `REGISTER_LIDAR_BACKEND`, `LidarBackendRegistry::Create`
- Produces: 进程加载后 `Has("velodyne")` 与 `Has("udp")` 为 true

- [ ] **Step 1: 在 `udp_driver.cpp` 末尾注册**

```cpp
#include "autodriver/lidar/backend_register.hpp"

REGISTER_LIDAR_BACKEND(velodyne, "velodyne",
                       autodriver::hardware::CreateVelodyneUdpDriver, "udp");
```

（若宏对空 VA 有问题，显式只传 `"udp"`。）

- [ ] **Step 2: 改 `Lidar3dModule::MakeDriver`**

```cpp
#include "autodriver/lidar/backend_registry.hpp"

std::shared_ptr<autodriver::SensorDriver> MakeDriver(
    const autodriver::Config::Sensor& sensor) override {
  return autodriver::lidar::LidarBackendRegistry::Instance().Create(
      sensor.backend, sensor.id, sensor.params);
}
```

删除 `#include "autodriver/lidar/velodyne/udp_driver.hpp"`（注册靠链接 udp_driver.cpp）。

- [ ] **Step 3: 在 `test_stream_lidar_base.cpp` 增加**

```cpp
TEST(LidarBackendRegistry, VelodyneRegistered) {
  auto& reg = autodriver::lidar::LidarBackendRegistry::Instance();
  EXPECT_TRUE(reg.Has("velodyne"));
  EXPECT_TRUE(reg.Has("udp"));
}
```

（该测试链接 `autodriver` so，静态注册应已执行。）

- [ ] **Step 4: 跑 `test_stream_lidar_base` + `test_lidar_backend_registry`**

Expected: Velodyne raw_packet 出云仍绿；Has 断言通过。

---

### Task 3: Proto `VelodyneLidarConf` + ConfigFromProto 摊平

**Files:**
- Modify: `autodriver/proto/autodriver_conf.proto`
- Modify: `autodriver/autodriver/config_proto.cpp`（`SensorFromProto` / 可选 `SensorToProto`）
- Modify: `autodriver/test/test_config_proto.cpp`

**Interfaces:**
- Produces: `proto::VelodyneLidarConf`；`SensorConfig.lidar_vendor` oneof field 24 = `velodyne`
- Consumes: 现有 `SetParamIfNonEmpty` / `SetParamIfPositive`；新增 `SetParamIfAbsent` 助手：仅当 `params` 无该 key 时写入

- [ ] **Step 1: 扩展 proto**

在 `LidarConfigBase` 后增加：

```protobuf
message VelodyneLidarConf {
  int32 data_port = 1;
  int32 packets_per_scan = 2;
  string model = 3;
  string frame_id = 4;
  string bind_host = 5;
  int32 reconnect_attempts = 6;
  bool enable_compensator = 7;
  string world_frame_id = 8;
  string extrinsic_path = 9;
}
```

在 `SensorConfig` 中 `LidarConfigBase lidar = 23;` 之后：

```protobuf
  oneof lidar_vendor {
    VelodyneLidarConf velodyne = 24;
  }
```

- [ ] **Step 2: `SensorFromProto` 摊平**

在 `has_lidar()` 处理之后：

```cpp
if (message.has_velodyne()) {
  const auto& v = message.velodyne();
  if (sensor.backend.empty()) {
    sensor.backend = "velodyne";
  }
  auto set_str = [&](const char* key, const std::string& val) {
    if (!val.empty() && sensor.params.find(key) == sensor.params.end()) {
      sensor.params.emplace(key, val);
    }
  };
  auto set_int = [&](const char* key, int val) {
    if (val != 0 && sensor.params.find(key) == sensor.params.end()) {
      sensor.params.emplace(key, std::to_string(val));
    }
  };
  set_int("data_port", v.data_port());
  set_int("packets_per_scan", v.packets_per_scan());
  set_str("model", v.model());
  set_str("frame_id", v.frame_id());
  set_str("bind_host", v.bind_host());
  set_int("reconnect_attempts", v.reconnect_attempts());
  if (sensor.params.find("enable_compensator") == sensor.params.end()) {
    // proto3 bool 默认 false：仅当 true 时写入，避免覆盖驱动默认
    if (v.enable_compensator()) {
      sensor.params.emplace("enable_compensator", "true");
    }
  }
  set_str("world_frame_id", v.world_frame_id());
  set_str("extrinsic_path", v.extrinsic_path());
}
```

- [ ] **Step 3: 单测**

```cpp
TEST(ConfigProto, VelodyneConfFlattensToParams) {
  autodriver::proto::AutodriverConfig message;
  auto* s = message.add_sensors();
  s->set_module("Lidar3dModule");
  s->set_sensor_id("lidar/vlp16");
  s->set_attach_on_start(true);
  s->mutable_velodyne()->set_data_port(2369);
  s->mutable_velodyne()->set_model("VLP-16");
  s->mutable_params()->insert({"data_port", "9999"});  // params 优先
  const autodriver::Config config = autodriver::ConfigFromProto(message);
  ASSERT_EQ(config.sensors.size(), 1u);
  EXPECT_EQ(config.sensors[0].backend, "velodyne");
  EXPECT_EQ(config.sensors[0].params.at("data_port"), "9999");
  EXPECT_EQ(config.sensors[0].params.at("model"), "VLP-16");
}
```

- [ ] **Step 4: 跑 `test_config_proto`**

Expected: 新旧用例均 PASS。

---

### Task 4: 文档与对齐计划

**Files:**
- Modify: `autodriver/docs/source/guide/backends.md`
- Modify: `autodriver/docs/source/api/overview.md`
- Modify: `autodriver/docs/source/guide/configuration.md`（pb.txt 示例一小段）
- Modify: `docs/superpowers/plans/2026-09-05-autodriver-apollo-drivers-align.md`

- [ ] **Step 1: backends.md「扩展真 Lidar」改为**

1. `lidar/<vendor>/` 实现 packet + convert + driver  
2. `REGISTER_LIDAR_BACKEND(tag, "name", CreateFn, "alias"...)`  
3. CMake 加入源文件  
4. **不必**改 `Lidar3dModule`

- [ ] **Step 2: api/overview 表增加 `lidar/backend_registry.hpp`**

- [ ] **Step 3: configuration.md lidar_3d 节增加 pb.txt 示例**

```text
sensors {
  module: "Lidar3dModule"
  sensor_id: "lidar/vlp16"
  attach_on_start: true
  channel: "/lidar/vlp16/points"
  velodyne {
    data_port: 2368
    packets_per_scan: 75
    model: "VLP-16"
  }
}
```

- [ ] **Step 4: 对齐计划后续项增加并勾选**

`- [x] Lidar 厂商静态注册表 + VelodyneLidarConf`

保留 `- [ ] Hesai / 其他厂商 Convert`。

---

## Spec coverage check

| Spec 要求 | Task |
|-----------|------|
| Registry API + 宏 | Task 1 |
| Velodyne 自注册 + MakeDriver 查表 | Task 2 |
| VelodyneLidarConf + 摊平 / params 优先 | Task 3 |
| 文档与扩展步骤 | Task 4 |
| 不做流水线拆分 / Hesai Convert | 全计划未包含 |

## 执行说明

测试命令（在已配置的 autonomy/autodriver build 目录）：

```bash
ctest -R 'test_lidar_backend_registry|test_stream_lidar_base|test_config_proto' --output-on-failure
```
