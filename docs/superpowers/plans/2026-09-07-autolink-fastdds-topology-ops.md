# Autolink FastDDS Topology & Ops (D+E) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 实现跨机 `RtpsTopologyBackend`（双 Participant）与运维增强（可选 Discovery Server、payload 软限、RtpsStats、3.x 文档），在无 DDS Security 的前提下补齐多机拓扑闭环。

**Architecture:** `RtpsParticipantHub` 持有 topology/transport 两个 DomainParticipant；`TopologyBackendFactory` 在 FastDDS ON 时创建 `RtpsTopologyBackend`；Transport 从 Hub 取 transport Participant。E 阶段给 Participant 加 Discovery Server CLIENT、给 Transmitter 加软限、加进程内 `RtpsStats`。

**Tech Stack:** C++17、Fast DDS 2.14、既有 `ITopologyBackend` / `ChangeMsg` / `transport/rtps/*`、GTest

**Spec:** `docs/superpowers/specs/2026-09-07-autolink-fastdds-topology-ops-design.md`

**Build / test:**

```bash
# OFF regression
cmake -S autolink -B autolink/build -DAUTOLINK_ENABLE_FASTDDS=OFF -DAUTOLINK_BUILD_TEST=ON
cmake --build autolink/build -j --target autolink
./autolink/build/bin/autolink.service_discovery.topology_backend_factory_test

# ON
cmake -S autolink -B autolink/build-fastdds -DAUTOLINK_ENABLE_FASTDDS=ON -DAUTOLINK_BUILD_TEST=ON
cmake --build autolink/build-fastdds -j --target autolink
./autolink/build-fastdds/bin/autolink.transport.integration_test.rtps_transceiver_test
# + new topology/rtps stats tests as added
```

`autolink/` is a **git submodule** — commit inside submodule then bump parent pointer.

---

## File Map

| 路径 | 职责 |
|------|------|
| `autolink/.../transport/rtps/participant_hub.{hpp,cpp}` | 双 Participant 单例 Hub |
| `autolink/.../transport/rtps/participant.{hpp,cpp}` | 扩展 role / discovery-server / name 后缀 |
| `autolink/.../transport/transport.cpp` | 从 Hub 取 transport Participant |
| `autolink/.../service_discovery/rtps_topology_backend.{hpp,cpp}` | ChangeMsg over RTPS |
| `autolink/.../service_discovery/topology_backend_factory.*` | `rtps` → 真后端或 fallback |
| `autolink/.../transport/rtps/rtps_stats.{hpp,cpp}` | 原子计数 + Dump |
| `autolink/.../transport/rtps/payload_limit.{hpp,cpp}` | 读 env 软限 |
| `autolink/.../transport/transmitter/rtps_transmitter.hpp` | 软限 + stats |
| `docs/source/03_Communication/14_fastdds.md` | BACKEND=rtps、Server、软限、3.x、无 Security |
| `autolink/cmake/FastDDS.cmake` | 3.x WARNING |
| `*_test.cpp` | factory / oversize / topology smoke |

---

### Task 1: M4 — Extend Participant + RtpsParticipantHub

**Files:**
- Modify: `autolink/autolink/transport/rtps/participant.hpp`
- Modify: `autolink/autolink/transport/rtps/participant.cpp`
- Create: `autolink/autolink/transport/rtps/participant_hub.hpp`
- Create: `autolink/autolink/transport/rtps/participant_hub.cpp`
- Modify: CMake fastdds-gated sources
- Modify: `autolink/autolink/transport/transport.cpp`

- [ ] **Step 1: Extend Participant constructor**

```cpp
enum class ParticipantRole { kTopology, kTransport };

explicit Participant(const proto::RtpsParticipantAttr& attr,
                     ParticipantRole role,
                     const std::vector<std::string>& discovery_servers = {});
// name_ = hostname + pid + (role==kTopology ? ":topology" : ":transport");
// If discovery_servers non-empty: configure CLIENT discovery (Fast DDS 2.14 API).
```

Keep SIMPLE when `discovery_servers` empty (current behavior). Parse servers later in Task 5; for Task 1 accept empty vector only if needed — or parse env in Hub now:

```cpp
// participant_hub.cpp
std::vector<std::string> ParseDiscoveryServers() {
  const char* e = std::getenv("AUTOLINK_DISCOVERY_SERVER");
  // split by comma; trim; return list (may be empty)
}
```

- [ ] **Step 2: Implement Hub**

```cpp
class RtpsParticipantHub {
 public:
  static RtpsParticipantHub& Instance();
  bool Init(const proto::RtpsParticipantAttr& attr);  // creates both participants
  ParticipantPtr TopologyParticipant() const;
  ParticipantPtr TransportParticipant() const;
  void Shutdown();
 private:
  ParticipantPtr topology_;
  ParticipantPtr transport_;
};
```

`Init`：若已 init 返回 true；创建 topology + transport；任一侧失败 → AERROR、reset 两者、return false。

- [ ] **Step 3: Transport uses Hub**

Replace Transport ctor Participant creation with:

```cpp
#if AUTOLINK_ENABLE_FASTDDS
  auto& hub = RtpsParticipantHub::Instance();
  if (!hub.Init(config.transport_conf().participant_attr())) {
    AERROR << "RtpsParticipantHub Init failed";
  } else {
    participant_ = hub.TransportParticipant();
    RtpsDispatcher::Instance()->set_participant(participant_);
  }
#endif
```

`Shutdown`：先 Dispatcher，再 `hub.Shutdown()`（勿对 participant_ 双重 Shutdown 弄乱所有权——Hub 拥有对象，Transport 只持 shared_ptr）。

- [ ] **Step 4: Build ON + OFF**

Expected: both `--target autolink` PASS；existing `rtps_transceiver_test` PASS on ON.

- [ ] **Step 5: Commit**

```bash
# in autolink submodule
git commit -m "$(cat <<'EOF'
feat(autolink): add RtpsParticipantHub with topology/transport participants
EOF
)"
# parent bump
```

---

### Task 2: M4 — RtpsTopologyBackend

**Files:**
- Create: `autolink/autolink/service_discovery/rtps_topology_backend.hpp`
- Create: `autolink/autolink/service_discovery/rtps_topology_backend.cpp`
- Modify: `topology_backend_factory.cpp` / test
- Gate sources with `AUTOLINK_ENABLE_FASTDDS`

- [ ] **Step 1: Failing factory test (ON build)**

```cpp
#if AUTOLINK_ENABLE_FASTDDS
TEST(TopologyBackendFactoryTest, RtpsCreatesRtpsBackend) {
  auto b = TopologyBackendFactory::Create("rtps");
  ASSERT_NE(b, nullptr);
  EXPECT_TRUE(dynamic_cast<RtpsTopologyBackend*>(b.get()) != nullptr);
}
#else
TEST(TopologyBackendFactoryTest, RtpsFallsBackWhenDisabled) {
  auto b = TopologyBackendFactory::Create("rtps");
  ASSERT_NE(b, nullptr);
  EXPECT_TRUE(dynamic_cast<LocalTopologyBackend*>(b.get()) != nullptr);
}
#endif
```

- [ ] **Step 2: Implement RtpsTopologyBackend**

```cpp
class RtpsTopologyBackend final : public ITopologyBackend {
 public:
  bool Start() override;
  int64_t Subscribe(proto::ChangeType type, const ChangeCallback& cb) override;
  void Unsubscribe(int64_t id) override;
  bool Publish(const proto::ChangeMsg& msg) override;
  void Shutdown() override;
};
```

Implementation notes:
- `Start`: `RtpsParticipantHub::Instance().Init(...)` using GlobalData config; create 3 topics/writers/readers with RELIABLE + TRANSIENT_LOCAL via AttributesFiller or inline QoS.
- Topic names: `node_change_broadcast`, `channel_change_broadcast`, `service_change_broadcast` (match Manager).
- `Publish`: serialize ChangeMsg → UnderlayMessage `{datatype="ChangeMsg", data=bytes}` → write on writer for `msg.change_type()` (map ChangeType → topic). Apply same payload limit helper as Task 4 when available; until then skip limit.
- Reader listener: parse ChangeMsg; skip if `host_name+pid` == self; dispatch to subscribers filtered by type (or all).
- Do **not** use 24B MessageInfo prefix for topology (datatype discriminates).

- [ ] **Step 3: Factory**

```cpp
if (key == "rtps") {
#if AUTOLINK_ENABLE_FASTDDS
  return std::make_shared<RtpsTopologyBackend>();
#else
  AERROR << "AUTOLINK_TOPOLOGY_BACKEND=rtps requires AUTOLINK_ENABLE_FASTDDS";
  return std::make_shared<LocalTopologyBackend>();
#endif
}
```

- [ ] **Step 4: Run factory tests OFF + ON**

Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git commit -m "$(cat <<'EOF'
feat(autolink): implement RtpsTopologyBackend for ChangeMsg over RTPS
EOF
)"
```

---

### Task 3: M4 — Topology integration smoke + docs (D)

**Files:**
- Create: `autolink/.../service_discovery/rtps_topology_backend_test.cpp` (or integration_test)
- Modify: `docs/source/03_Communication/14_fastdds.md`
- Modify: `autolink/CHANGELOG.rst`

- [ ] **Step 1: Smoke test (ON)**

Two `Node` instances (or two backends) with `AUTOLINK_TOPOLOGY_BACKEND=rtps` in process: join a channel writer on A, assert B’s ChannelManager sees writer within timeout. If same-process discovery is flaky, use two threads after Hub Init + short sleep for match.

Minimal acceptable test:
```cpp
TEST(RtpsTopologyBackendTest, PublishSubscribeChangeMsg) {
  // Start two backends or one backend + direct Publish/Subscribe
  // Publish a synthetic ChangeMsg JOIN; expect callback
}
```

- [ ] **Step 2: Update 14_fastdds.md**

Add sections:
- `AUTOLINK_TOPOLOGY_BACKEND=rtps` dual-host topology
- Dual Participant note
- Security: **not implemented**

- [ ] **Step 3: CHANGELOG M4 bullets**

- [ ] **Step 4: Commit**

```bash
git commit -m "$(cat <<'EOF'
test(autolink): add RtpsTopologyBackend coverage and multimachine topology docs
EOF
)"
```

---

### Task 4: M5 — Payload soft limit

**Files:**
- Create: `autolink/.../transport/rtps/payload_limit.hpp` (header-only OK)
- Modify: `rtps_transmitter.hpp`
- Create: `payload_limit_test.cpp` or add cases to existing test
- Optionally call from `RtpsTopologyBackend::Publish`

- [ ] **Step 1: Helper**

```cpp
struct PayloadLimit {
  size_t max_bytes = 4 * 1024 * 1024;  // 0 = disabled
  bool reject_oversize = false;
  static PayloadLimit FromEnv();
  // AUTOLINK_RTPS_MAX_PAYLOAD_BYTES, AUTOLINK_RTPS_REJECT_OVERSIZE
};

enum class PayloadCheck { kOk, kWarn, kReject };
PayloadCheck CheckPayloadSize(size_t size, const PayloadLimit& lim);
```

- [ ] **Step 2: RtpsTransmitter::Transmit**

After building underlay `data` size, `CheckPayloadSize`; on `kWarn` AWARN + continue; on `kReject` AERROR + return false; increment stats when Task 5 lands (or call RtpsStats stub).

- [ ] **Step 3: Unit test** FromEnv parsing + CheckPayloadSize branches.

- [ ] **Step 4: Commit**

```bash
git commit -m "$(cat <<'EOF'
feat(autolink): add configurable RTPS payload soft limit
EOF
)"
```

---

### Task 5: M5 — Discovery Server wiring + RtpsStats + 3.x docs

**Files:**
- Modify: `participant.cpp` (CLIENT discovery when servers non-empty) — if not fully done in Task 1
- Create: `autolink/.../transport/rtps/rtps_stats.hpp` (+ `.cpp` if needed)
- Modify: transmitter/dispatcher to bump counters on write/recv/match/fail/oversize
- Modify: `cmake/FastDDS.cmake` warn on major≥3
- Modify: `14_fastdds.md` Discovery Server steps + 3.x checklist + soft env table
- Modify: `CHANGELOG.rst`

- [ ] **Step 1: RtpsStats**

```cpp
class RtpsStats {
 public:
  static RtpsStats& Instance();
  void AddSent(uint64_t n = 1);
  void AddRecv(uint64_t n = 1);
  void AddWriteFail(uint64_t n = 1);
  void AddOversize(uint64_t n = 1);
  void SetMatchedReaders(int64_t v);
  void SetMatchedWriters(int64_t v);
  std::string Dump() const;
};
```

Wire: Transmitter success/fail/oversize；Dispatcher on_data；PublicationMatched/SubscriptionMatched if available.

- [ ] **Step 2: Discovery Server**

In `Participant::Init`, if `discovery_servers_` non-empty, set discovery protocol to CLIENT and add server locators (`ip:port` parse). Document running:

```bash
fast-discovery-server -i 0.0.0.0 -p 11811
export AUTOLINK_DISCOVERY_SERVER=192.168.1.10:11811
```

- [ ] **Step 3: CMake 3.x warning**

When find_package returns version ≥3.0, `message(WARNING "Autolink RTPS validated on Fast DDS 2.14; 3.x is untested")`.

- [ ] **Step 4: Docs + CHANGELOG M5**

- [ ] **Step 5: Run OFF factory test + ON transceiver + new unit tests**

Expected: PASS.

- [ ] **Step 6: Commit**

```bash
git commit -m "$(cat <<'EOF'
feat(autolink): add Discovery Server client, RtpsStats, and 3.x notes
EOF
)"
```

---

### Task 6: Spec status

**Files:**
- Modify: `docs/superpowers/specs/2026-09-07-autolink-fastdds-topology-ops-design.md`

- [ ] Set status to `已实现（M4–M5）` after all tasks green.
- [ ] Commit: `docs(autolink): mark topology-ops spec M4–M5 implemented`

---

## Spec coverage

| Spec | Task |
|------|------|
| Hub + dual Participant | 1 |
| RtpsTopologyBackend + factory | 2 |
| Topology test + D docs | 3 |
| Soft limit | 4 |
| Discovery Server + Stats + 3.x | 5 |
| Spec status | 6 |
| No Security | documented in Task 3/5 |

## Consistency

- Topology Underlay: **no** 24B MessageInfo prefix；business Channel keeps 24B prefix.
- Hub owns Participants；Transport/Backend hold `shared_ptr` only.
- `BACKEND=rtps` + FastDDS OFF → AERROR + local fallback.
