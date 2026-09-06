# Autolink FastDDS Multimachine (A→B→C) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 按 M1→M2→M3 让 Autolink 文档/配置与实现一致，拓扑后端可插拔，并在可选 Fast DDS **2.14** 下恢复 Hybrid `diff_host→RTPS` Channel 数据面。

**Architecture:** 默认仍 INTRA+SHM + `LocalTopologyBackend`。`TopologyBackendFactory` 读后端名；`AUTOLINK_ENABLE_FASTDDS` 打开时编译 `transport/rtps/`，Transport/Hybrid 创建真正的 RTPS transceiver。P0 跨机靠 DDS SIMPLE 多播匹配同名 topic；拓扑图仍可本机文件。MessageInfo 本轮锁定为 `UnderlayMessage.data` 前缀 **24 字节**（sender_id/spare_id/seq 各 8B LE），避免依赖 WriteParams 跨版本差异。

**Tech Stack:** C++17、CMake、GTest、protobuf、Fast DDS 2.14 + FastCDR（可选）、现有 Hybrid/Transport/TopologyManager

**Spec:** `docs/superpowers/specs/2026-09-06-autolink-fastdds-multimachine-design.md`

**Build / test（后续步骤默认）：**

```bash
# 默认 OFF
cmake -S autolink -B autolink/build -DBUILD_TESTS=ON
cmake --build autolink/build -j
ctest --test-dir autolink/build -R 'transport|topology|shm|intra|hybrid' --output-on-failure

# 可选 ON（需能拉到 Fast DDS 2.14）
cmake -S autolink -B autolink/build-fastdds -DBUILD_TESTS=ON -DAUTOLINK_ENABLE_FASTDDS=ON
cmake --build autolink/build-fastdds -j --target autolink
ctest --test-dir autolink/build-fastdds -R 'rtps_|topology_backend' --output-on-failure
```

若仓库以 `autoviz/build` 拉 `_deps/autolink`，在对应 build 目录对 `autolink` 目标执行同等开关。

---

## File Map

| 路径 | 职责 |
|------|------|
| `docs/source/03_Communication/{00_guide,01_architecture,03_channel,13_survey}.md` | 纠正跨机/RTPS/发现表述 |
| `autolink/README.md`、`autolink/CHANGELOG.rst` | 默认本地 + 可选 FastDDS |
| `autolink/examples/python/README.md` | 去掉误导 DOMAIN 说明 |
| `autolink/autolink/conf/autolink.pb.conf` | conf 注释 |
| `autolink/autolink/setup.bash`、`.release.bash` | env 注释 |
| `autolink/autolink/proto/transport_conf.proto` | RTPS 注释改为「optional when enabled」 |
| `autolink/.../hybrid_{transmitter,receiver}.hpp` | 去掉静默 RTPS→SHM |
| `autolink/.../transport/transport.hpp` | RTPS 分支条件编译 |
| `autolink/.../service_discovery/topology_backend_factory.{hpp,cpp}` | 工厂 |
| `autolink/.../service_discovery/topology_manager.cpp` | 使用工厂 + env |
| `autolink/.../service_discovery/topology_backend_factory_test.cpp` | 工厂单测 |
| `autolink/CMakeLists.txt` 或 `autolink/autolink/CMakeLists.txt` | `AUTOLINK_ENABLE_FASTDDS` |
| `autolink/autolink/transport/rtps/*` | Participant / Underlay / Filler / Tx/Rx/Dispatcher |
| `autolink/autolink/common/global_data.{hpp,cpp}` | `DomainId()` 读 `AUTOLINK_DOMAIN_ID` |
| `autolink/.../integration_test/rtps_transceiver_test.cpp` | ENABLE=ON 集成测 |
| `autolink/docs/source/...` 或 `docs/source/03_Communication/` | FastDDS 启用与双机步骤 + 3.x 升级清单 |

---

### Task 1: M1 — 通信文档真相

**Files:**
- Modify: `docs/source/03_Communication/00_guide.md`
- Modify: `docs/source/03_Communication/01_architecture.md`
- Modify: `docs/source/03_Communication/03_channel.md`
- Modify: `docs/source/03_Communication/13_survey.md`
- Modify: `autolink/examples/python/README.md`
- Modify: `autolink/README.md`

- [ ] **Step 1: 统一替换误导表述**

将「跨主机 = RTPS（默认）」「发现经 UDP 多播拓扑」「`AUTOLINK_DOMAIN_ID` = 跨主机发现域（暗示已生效）」改为：

```markdown
默认传输：同进程 INTRA、同机多进程 SHM；拓扑发现为本机文件总线（`/tmp/autolink_topology_events.log`）。
跨机 Channel 为**可选**能力：编译打开 `AUTOLINK_ENABLE_FASTDDS`，配置 `diff_host: RTPS`，
双方设置可达 `AUTOLINK_IP` 与相同 `AUTOLINK_DOMAIN_ID`（默认 80），依赖 Fast DDS SIMPLE 多播做端点匹配。
完整跨机拓扑图（ChangeMsg over RTPS）不在本阶段。
```

`01_architecture.md` / `03_channel.md` 中芯片与表格：RTPS 标注为「可选」；发现层改为「本机文件总线（默认）/ DDS 端点发现（可选 RTPS 数据面）」。

`13_survey.md`：Autolink 一行改为「发现：本机文件；跨机：可选 FastDDS」。

`examples/python/README.md`：删除或改写「跨机/容器 discovery 需对齐 `AUTOLINK_DOMAIN_ID`」为上述可选说明。

- [ ] **Step 2: 目视检查无残留「默认跨机 RTPS」**

Run: `rg -n '跨主机.*RTPS|发现经 \*\*UDP|AUTOLINK_DOMAIN_ID.*跨主机发现域' docs/source/03_Communication autolink/examples/python/README.md`

Expected: 无未加「可选」限定的旧句；或仅出现在「历史/对比」且标明已移除默认路径。

- [ ] **Step 3: Commit**

```bash
git add docs/source/03_Communication autolink/examples/python/README.md autolink/README.md
git commit -m "$(cat <<'EOF'
docs(autolink): align multimachine docs with INTRA/SHM defaults

Clarify that FastDDS RTPS is optional and topology discovery is local-file by default.
EOF
)"
```

---

### Task 2: M1 — conf / env / 禁止静默 RTPS→SHM

**Files:**
- Modify: `autolink/autolink/conf/autolink.pb.conf`
- Modify: `autolink/autolink/setup.bash`
- Modify: `autolink/autolink/.release.bash`
- Modify: `autolink/autolink/proto/transport_conf.proto`
- Modify: `autolink/autolink/transport/transmitter/hybrid_transmitter.hpp`
- Modify: `autolink/autolink/transport/receiver/hybrid_receiver.hpp`
- Modify: `autolink/autolink/transport/transport.hpp`

- [ ] **Step 1: 更新 conf 与 proto 注释**

`autolink.pb.conf` 文件头改为：

```text
# Default: INTRA (same proc) + SHM (diff proc / diff host).
# Cross-host Channel requires AUTOLINK_ENABLE_FASTDDS=ON and diff_host: RTPS.
# participant_attr / AUTOLINK_DOMAIN_ID apply only when FastDDS RTPS is enabled.
```

`transport_conf.proto`：

```protobuf
  RTPS = 3;  // Optional; requires AUTOLINK_ENABLE_FASTDDS
...
  OptionalMode diff_host = 3;  // Default SHM; set RTPS when FastDDS enabled
```

`setup.bash` / `.release.bash` 在 export 旁：

```bash
# Used by RTPS when AUTOLINK_ENABLE_FASTDDS=ON. Topology backend default is local file.
export AUTOLINK_DOMAIN_ID=80
export AUTOLINK_IP=127.0.0.1
# export AUTOLINK_TOPOLOGY_BACKEND=local
```

- [ ] **Step 2: Hybrid / Transport 去掉静默 fallback**

`hybrid_transmitter.hpp` 中 `case OptionalMode::RTPS:` 改为（`hybrid_receiver.hpp` 对称）：

```cpp
case OptionalMode::RTPS:
#if AUTOLINK_ENABLE_FASTDDS
    transmitters_[mode] =
        std::make_shared<RtpsTransmitter<M>>(this->attr_,
                                             Transport::Instance()->participant());
#else
    AERROR << "HybridTransmitter: RTPS requested but AUTOLINK_ENABLE_FASTDDS=OFF";
#endif
    break;
```

若 OFF 时未插入 transmitter，后续 `Enable`/`Transmit` 对缺失 mode 打 AERROR 并 return false（保持与「明确失败」一致）。在文件顶增加：

```cpp
#ifndef AUTOLINK_ENABLE_FASTDDS
#define AUTOLINK_ENABLE_FASTDDS 0
#endif
#if AUTOLINK_ENABLE_FASTDDS
#include "autolink/transport/transmitter/rtps_transmitter.hpp"  // Task 6 落地后再解开；本 Task 可先只改错误路径
#endif
```

**本 Task 可先实现 OFF 路径**：RTPS 分支只 AERROR、不创建 SHM；ON 路径等 Task 6/7 include 真正类型。临时：

```cpp
case OptionalMode::RTPS:
    AERROR << "HybridTransmitter: RTPS not available "
              "(build with -DAUTOLINK_ENABLE_FASTDDS=ON).";
    break;
```

`transport.hpp` 的 `CreateTransmitter/Receiver` RTPS case 同样改为 AERROR + `nullptr`（删除「use HYBRID/INTRA/SHM」之外的静默成功路径）。

- [ ] **Step 3: 编译默认 OFF 目标**

Run: `cmake --build <build> -j --target autolink`  
Expected: 成功。

- [ ] **Step 4: Commit**

```bash
git add autolink/autolink/conf autolink/autolink/setup.bash autolink/autolink/.release.bash \
  autolink/autolink/proto/transport_conf.proto \
  autolink/autolink/transport/transmitter/hybrid_transmitter.hpp \
  autolink/autolink/transport/receiver/hybrid_receiver.hpp \
  autolink/autolink/transport/transport.hpp
git commit -m "$(cat <<'EOF'
fix(autolink): fail loud when RTPS requested without FastDDS

Stop silent Hybrid RTPS→SHM fallback; document conf/env for optional RTPS.
EOF
)"
```

---

### Task 3: M2 — TopologyBackendFactory

**Files:**
- Create: `autolink/autolink/service_discovery/topology_backend_factory.hpp`
- Create: `autolink/autolink/service_discovery/topology_backend_factory.cpp`
- Modify: `autolink/autolink/service_discovery/topology_manager.cpp`
- Create: `autolink/autolink/service_discovery/topology_backend_factory_test.cpp`
- Modify: 测试注册 CMake（与现有 `*_test.cpp` 同样 GLOB/显式列表）

- [ ] **Step 1: 写失败单测**

```cpp
#include "gtest/gtest.h"
#include "autolink/service_discovery/topology_backend_factory.hpp"

namespace autolink {
namespace service_discovery {
namespace {

TEST(TopologyBackendFactoryTest, DefaultLocal) {
  auto b = TopologyBackendFactory::Create("local");
  ASSERT_NE(b, nullptr);
  EXPECT_TRUE(dynamic_cast<LocalTopologyBackend*>(b.get()) != nullptr);
}

TEST(TopologyBackendFactoryTest, UnknownFallsBackToLocal) {
  auto b = TopologyBackendFactory::Create("rtps");
  ASSERT_NE(b, nullptr);
  EXPECT_TRUE(dynamic_cast<LocalTopologyBackend*>(b.get()) != nullptr);
}

TEST(TopologyBackendFactoryTest, EmptyFallsBackToLocal) {
  auto b = TopologyBackendFactory::Create("");
  ASSERT_NE(b, nullptr);
}

}  // namespace
}  // namespace service_discovery
}  // namespace autolink
```

- [ ] **Step 2: 实现工厂**

`topology_backend_factory.hpp`:

```cpp
#pragma once
#include <memory>
#include <string>
#include "autolink/service_discovery/topology_backend.hpp"

namespace autolink {
namespace service_discovery {

class TopologyBackendFactory {
 public:
  // "local" → LocalTopologyBackend.
  // Unknown / "rtps" (not implemented) → warn + LocalTopologyBackend.
  static std::shared_ptr<ITopologyBackend> Create(const std::string& name);
};

}  // namespace service_discovery
}  // namespace autolink
```

`topology_backend_factory.cpp`:

```cpp
#include "autolink/service_discovery/topology_backend_factory.hpp"
#include "autolink/common/log.hpp"

namespace autolink {
namespace service_discovery {

std::shared_ptr<ITopologyBackend> TopologyBackendFactory::Create(
    const std::string& name) {
  std::string key = name.empty() ? "local" : name;
  if (key == "local") {
    return std::make_shared<LocalTopologyBackend>();
  }
  AWARN << "Topology backend '" << key
        << "' unavailable; falling back to local file backend.";
  return std::make_shared<LocalTopologyBackend>();
}

}  // namespace service_discovery
}  // namespace autolink
```

- [ ] **Step 3: TopologyManager 接线**

将 `backend_ = std::make_unique<LocalTopologyBackend>();` 改为：

```cpp
#include <cstdlib>
#include "autolink/service_discovery/topology_backend_factory.hpp"

const char* backend_env = std::getenv("AUTOLINK_TOPOLOGY_BACKEND");
const std::string backend_name =
    backend_env == nullptr ? "local" : std::string(backend_env);
backend_ = TopologyBackendFactory::Create(backend_name);
```

注意：`backend_` 若现类型为 `unique_ptr`，改为 `shared_ptr<ITopologyBackend>`，或工厂返回 `unique_ptr`。**选定：`shared_ptr<ITopologyBackend> backend_`**，同步改 `topology_manager.hpp` 成员类型与 `Manager::SetTopologyBackend(backend_.get())`。

日志改为：`TopologyManager using backend=` << backend_name << ` host_ip=` << host_ip。

- [ ] **Step 4: 跑单测**

Run: `ctest --test-dir <build> -R topology_backend_factory -V`  
Expected: PASS。

- [ ] **Step 5: Commit**

```bash
git add autolink/autolink/service_discovery/topology_backend_factory.* \
  autolink/autolink/service_discovery/topology_manager.* \
  autolink/autolink/service_discovery/topology_backend_factory_test.cpp
git commit -m "$(cat <<'EOF'
feat(autolink): add TopologyBackendFactory with local default

Honor AUTOLINK_TOPOLOGY_BACKEND; unknown/rtps names fall back to local.
EOF
)"
```

---

### Task 4: M3 — CMake `AUTOLINK_ENABLE_FASTDDS`

**Files:**
- Modify: `autolink/CMakeLists.txt`（根，若 option 在此）及/或 `autolink/autolink/CMakeLists.txt`
- Create: `autolink/cmake/FastDDS.cmake`（可选，保持根 CMake 清晰）

- [ ] **Step 1: 增加 option 与探测**

```cmake
option(AUTOLINK_ENABLE_FASTDDS "Build RTPS transport with Fast DDS 2.14" OFF)

set(AUTOLINK_HAS_FASTDDS OFF)
if(AUTOLINK_ENABLE_FASTDDS)
  find_package(fastdds 2.14 QUIET)
  if(NOT fastdds_FOUND)
    include(FetchContent)
    FetchContent_Declare(
      fastdds
      GIT_REPOSITORY https://github.com/eProsima/Fast-DDS.git
      GIT_TAG 2.14.4
      GIT_SHALLOW TRUE)
    set(BUILD_SHARED_LIBS ON CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(fastdds)
  endif()
  set(AUTOLINK_HAS_FASTDDS ON)
endif()

if(AUTOLINK_HAS_FASTDDS)
  target_compile_definitions(autolink PUBLIC AUTOLINK_ENABLE_FASTDDS=1)
  target_link_libraries(autolink PUBLIC fastdds fastcdr)
else()
  target_compile_definitions(autolink PUBLIC AUTOLINK_ENABLE_FASTDDS=0)
endif()
```

GLOB 已包含 `transport/*.cpp`；确保 `transport/rtps/*.cpp` 被编入，或 ON 时显式 `target_sources(autolink PRIVATE ...)`。OFF 时 **不要** 编译依赖 FastDDS 头的 `.cpp`（用 `if(AUTOLINK_HAS_FASTDDS)` 追加 sources）。

- [ ] **Step 2: 配置 OFF / ON 各一次**

```bash
cmake -S autolink -B /tmp/al-off -DAUTOLINK_ENABLE_FASTDDS=OFF
cmake -S autolink -B /tmp/al-on  -DAUTOLINK_ENABLE_FASTDDS=ON   # 可先只验证 option 解析
```

Expected: OFF 立刻成功；ON 开始找包或 FetchContent（网络允许）。

- [ ] **Step 3: Commit**

```bash
git add autolink/CMakeLists.txt autolink/autolink/CMakeLists.txt autolink/cmake/FastDDS.cmake
git commit -m "$(cat <<'EOF'
build(autolink): add optional AUTOLINK_ENABLE_FASTDDS for Fast DDS 2.14
EOF
)"
```

---

### Task 5: M3 — DomainId + UnderlayMessage + Participant

**Files:**
- Modify: `autolink/autolink/common/global_data.hpp`
- Modify: `autolink/autolink/common/global_data.cpp`
- Create: `autolink/autolink/transport/rtps/underlay_message.hpp`
- Create: `autolink/autolink/transport/rtps/underlay_message.cpp`
- Create: `autolink/autolink/transport/rtps/underlay_message_type.hpp`
- Create: `autolink/autolink/transport/rtps/underlay_message_type.cpp`
- Create: `autolink/autolink/transport/rtps/participant.hpp`
- Create: `autolink/autolink/transport/rtps/participant.cpp`

- [ ] **Step 1: GlobalData::DomainId()**

```cpp
// global_data.hpp
uint32_t DomainId() const;

// global_data.cpp Init 中
domain_id_ = 80;
const char* d = getenv("AUTOLINK_DOMAIN_ID");
if (d != nullptr) {
  domain_id_ = static_cast<uint32_t>(std::stoul(d));
}
```

- [ ] **Step 2: UnderlayMessage + 24B MessageInfo 前缀约定**

在 `underlay_message.hpp` 定义字段 `timestamp`/`seq`/`data`/`datatype`，并提供：

```cpp
// message_info_prefix.hpp（可同目录）
inline constexpr size_t kMsgInfoPrefixSize = 24;
bool PackMessageInfoPrefix(const MessageInfo& info, std::string* out_prefixed_payload);
bool UnpackMessageInfoPrefix(const std::string& prefixed, MessageInfo* info, std::string* payload);
```

`data` = prefix(24) + 业务序列化字节。

- [ ] **Step 3: Participant**

封装 Fast DDS 2.14：

```cpp
class Participant {
 public:
  explicit Participant(const proto::RtpsParticipantAttr& attr);
  bool Init();
  eprosima::fastdds::dds::DomainParticipant* get();
 private:
  uint32_t domain_id_;
  std::string host_ip_;
  eprosima::fastdds::dds::DomainParticipant* participant_ = nullptr;
};
```

`Init()`：`DomainParticipantQos` 设置 name=`hostname+pid`；builtin SIMPLE discovery；unicast locator 用 `GlobalData::HostIp()`；multicast `239.255.0.1`；注册 `UnderlayMessage` TypeSupport。失败打 AERROR 含 domain/ip。

- [ ] **Step 4: 仅 ON 配置下编译这些源文件**

Expected: `autolink` with `-DAUTOLINK_ENABLE_FASTDDS=ON` 能编过 Participant（可暂无 Tx）。

- [ ] **Step 5: Commit**

```bash
git commit -m "$(cat <<'EOF'
feat(autolink): add FastDDS Participant and UnderlayMessage (2.14)
EOF
)"
```

---

### Task 6: M3 — AttributesFiller + RtpsTransmitter/Dispatcher/Receiver

**Files:**
- Create: `autolink/autolink/transport/rtps/attributes_filler.{hpp,cpp}`
- Create: `autolink/autolink/transport/transmitter/rtps_transmitter.hpp`
- Create: `autolink/autolink/transport/receiver/rtps_receiver.hpp`
- Create: `autolink/autolink/transport/dispatcher/rtps_dispatcher.{hpp,cpp}`

参考 Apollo Cyber 同名文件结构，但 API 用 Fast DDS 2.14：

- Topic 名 = `attr.channel_name()`
- Writer：业务 `SerializeToString` → `PackMessageInfoPrefix` → `UnderlayMessage.data` → `DataWriter::write`
- Dispatcher：按 `channel_id` 建 DataReader；Listener 里 `UnpackMessageInfoPrefix` 后回调
- Receiver：向 Dispatcher 注册 listener（与 `ShmReceiver` 模式一致）

QoS 映射最小值：

```cpp
void FillInPubQos(const QosProfile& qos, DataWriterQos* wqos);
void FillInSubQos(const QosProfile& qos, DataReaderQos* rqos);
// History depth、RELIABLE/BEST_EFFORT、VOLATILE/TRANSIENT_LOCAL
```

- [ ] **Step 1: 实现 filler + dispatcher + templates**
- [ ] **Step 2: ON 下编译**
- [ ] **Step 3: Commit**

```bash
git commit -m "$(cat <<'EOF'
feat(autolink): add RtpsTransmitter/Receiver/Dispatcher for FastDDS 2.14
EOF
)"
```

---

### Task 7: M3 — Transport / Hybrid 接线 + History

**Files:**
- Modify: `autolink/autolink/transport/transport.hpp` / `transport.cpp`
- Modify: `hybrid_transmitter.hpp` / `hybrid_receiver.hpp`
- Modify: Hybrid `ThreadFunc` history 补发：DIFF_HOST+RTPS 时用 `RtpsTransmitter`，禁止 `ShmTransmitter`

- [ ] **Step 1: Transport 持有 Participant**

```cpp
#if AUTOLINK_ENABLE_FASTDDS
  participant_ = std::make_shared<rtps::Participant>(
      common::GlobalData::Instance()->Config().transport_conf().participant_attr());
  if (!participant_->Init()) {
    AERROR << "RTPS Participant init failed";
    // 保持 participant_ 为空；Create RTPS 返回 nullptr
  }
#endif
```

`CreateTransmitter(RTPS)`：

```cpp
#if AUTOLINK_ENABLE_FASTDDS
  return std::make_shared<RtpsTransmitter<M>>(attr, participant_);
#else
  AERROR << "RTPS transport is not available";
  return nullptr;
#endif
```

- [ ] **Step 2: Hybrid RTPS case 创建真正 RtpsTransmitter/Receiver**（替换 Task 2 占位）
- [ ] **Step 3: History 补发**

`ThreadFunc` 内创建临时 transmitter 时：

```cpp
#if AUTOLINK_ENABLE_FASTDDS
  auto transmitter = std::make_shared<RtpsTransmitter<M>>(new_attr, Transport::Instance()->participant());
#else
  auto transmitter = std::make_shared<ShmTransmitter<M>>(new_attr);
#endif
```

仅当 relation 对应 RTPS 时走 RTPS 分支（按 `mapping_table_[DIFF_HOST]` 判断）。

- [ ] **Step 4: Commit**

```bash
git commit -m "$(cat <<'EOF'
feat(autolink): wire Hybrid/Transport to optional RTPS data plane
EOF
)"
```

---

### Task 8: M3 — 测试、CHANGELOG、启用文档

**Files:**
- Create: `autolink/autolink/transport/integration_test/rtps_transceiver_test.cpp`
- Modify: `autolink/CHANGELOG.rst`
- Create or Modify: `docs/source/03_Communication/` 增加小节「可选 FastDDS」或 `autolink/docs/source/guide/fastdds.md`
- 内容含：启用步骤、双机清单、`diff_host: RTPS`、`AUTOLINK_IP`、`AUTOLINK_DOMAIN_ID`、**升 3.x 检查清单**（包名/`DomainParticipant` QoS/CMake）

- [ ] **Step 1: 集成测（同机两角色，强制不同 host_ip 字符串触发 DIFF_HOST 或显式 RTPS mode）**

```cpp
#if !AUTOLINK_ENABLE_FASTDDS
TEST(RtpsTransceiverTest, DisabledSkip) { GTEST_SKIP(); }
#else
TEST(RtpsTransceiverTest, PubSub) {
  // Create transmitter/receiver OptionalMode::RTPS on same channel
  // Send one RawMessage / proto; expect callback within 2s
}
#endif
```

- [ ] **Step 2: OFF 回归**

Run 默认 build 的 hybrid/shm/intra tests — Expected: PASS。

- [ ] **Step 3: ON 跑 `rtps_transceiver_test`** — Expected: PASS。

- [ ] **Step 4: CHANGELOG**

```rst
2026-09-06
----------
- 文档/配置与本机 INTRA+SHM 实现对齐；RTPS 不再静默 fallback SHM。
- 新增 TopologyBackendFactory（``AUTOLINK_TOPOLOGY_BACKEND``）。
- 可选 Fast DDS 2.14 RTPS 数据面（``AUTOLINK_ENABLE_FASTDDS``）。
```

- [ ] **Step 5: Commit**

```bash
git commit -m "$(cat <<'EOF'
test(autolink): add RTPS transceiver coverage and FastDDS enablement docs
EOF
)"
```

---

### Task 9: Spec 状态回写

**Files:**
- Modify: `docs/superpowers/specs/2026-09-06-autolink-fastdds-multimachine-design.md`

- [ ] **Step 1:** 将状态改为「已批准（实现中）」或全部 Task 完成后「已实现（M1–M3）」
- [ ] **Step 2: Commit**

```bash
git commit -m "docs(autolink): mark FastDDS multimachine spec approved for implementation"
```

---

## Spec coverage checklist

| Spec 项 | Task |
|---------|------|
| A 文档 | 1 |
| A conf/env/静默 fallback | 2 |
| B 工厂 + env | 3 |
| C CMake 2.14 可选 | 4 |
| C Participant / Underlay / DomainId | 5 |
| C Tx/Rx/Dispatcher / QoS | 6 |
| C Transport/Hybrid/History | 7 |
| 测试 + CHANGELOG + 双机/3.x 文档 | 8 |
| 不做 D/E | 未列入（正确） |

## Placeholder / 一致性自检

- MessageInfo：**统一 24B LE 前缀**（与 plan Architecture 一致）。
- `AUTOLINK_ENABLE_FASTDDS` 宏：0/1，Hybrid/Transport/测试同一套。
- 无 TBD 任务；FetchContent tag 钉 `2.14.4`（若上游无该 tag，改为该时刻最新 2.14.x patch 并在 Task 4 注释）。
