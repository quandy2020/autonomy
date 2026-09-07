# Autolink FastDDS 3.x Hard-Cut (M7) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 硬切 Fast DDS 3.x（CMake + API），跟迁 M6 Security，保持 `AUTOLINK_ENABLE_FASTDDS` 默认 OFF 与业务载荷语义不变。

**Architecture:** `FastDDS.cmake` 只认 `fastdds` major≥3（find_package 或 FetchContent 钉最新稳定 `v3.x.y`，设计参考 `v3.6.2`）；代码按 UPGRADING.md 去掉 `eprosima::fastrtps::*`；Security PropertyPolicy 语义保留。无 2.14 双轨、无 compat shim。

**Tech Stack:** C++17、Fast DDS 3.x、Fast CDR v2、OpenSSL（SECURITY=ON）、GTest

**Spec:** `docs/superpowers/specs/2026-09-07-autolink-fastdds-3x-design.md`

## Global Constraints

- 硬切 **3.x only**；系统仅 2.x → CMake **FATAL_ERROR**
- FetchContent pin：实现时核对 GitHub releases 最新稳定 3.x（参考 **`v3.6.2`**）
- `AUTOLINK_ENABLE_FASTDDS` **默认 OFF**；不改为默认 ON
- 载荷语义不变（Underlay、24B MessageInfo、拓扑无前缀、Hub 双 Participant、env 名）
- Security 必须跟迁回归（缺证 fail-loud + SECURITY=ON）
- `autolink/` 是 **git submodule** — 先 submodule commit，再 bump 父仓
- 首次 ON：建议 `rm -rf autolink/build-fastdds` 再配置
- 权威迁移：https://github.com/eProsima/Fast-DDS/blob/master/UPGRADING.md

**Build / test:**

```bash
# OFF
cmake -S autolink -B autolink/build -DAUTOLINK_ENABLE_FASTDDS=OFF -DAUTOLINK_BUILD_TEST=ON
cmake --build autolink/build -j --target autolink
./autolink/build/bin/autolink.service_discovery.topology_backend_factory_test
./autolink/build/bin/autolink.transport.rtps.security_config_test

# ON（wipe 推荐）
rm -rf autolink/build-fastdds
cmake -S autolink -B autolink/build-fastdds -DAUTOLINK_ENABLE_FASTDDS=ON -DAUTOLINK_BUILD_TEST=ON \
  -DOPENSSL_ROOT_DIR="$(brew --prefix openssl 2>/dev/null || true)"
cmake --build autolink/build-fastdds -j --target autolink
# then run ON suite listed in Task 4
```

---

## File Map

| 路径 | 职责 |
|------|------|
| `autolink/cmake/FastDDS.cmake` | 3.x only find/Fetch；FATAL on 2.x；SECURITY+OpenSSL |
| `autolink/.../transport/rtps/participant.cpp` | Factory library_settings；Discovery；Security；namespaces |
| `autolink/.../transport/rtps/underlay_message_type.{hpp,cpp}` | SerializedPayload / InstanceHandle / MD5 |
| `autolink/.../transport/dispatcher/rtps_dispatcher.hpp` | 3.x includes / ReturnCode |
| `autolink/.../transport/transmitter/rtps_transmitter.hpp` | 同上 |
| `autolink/.../service_discovery/rtps_topology_backend.cpp` | 同上 |
| `docs/source/03_Communication/14_fastdds.md` | 基线 3.x |
| `autolink/CHANGELOG.rst` | M7 |
| spec `...-fastdds-3x-design.md` | 完成后状态 → 已实现（M7） |

---

### Task 1: CMake hard-cut to Fast DDS 3.x

**Files:**
- Modify: `autolink/cmake/FastDDS.cmake`

**Interfaces:**
- Produces: `AUTOLINK_FASTDDS_GIT_TAG` = latest 3.x tag string；`AUTOLINK_FASTDDS_LINK_LIBS` = `fastdds` (+ `fastcdr`)；无 `fastrtps` 作为主依赖

- [ ] **Step 1: Confirm latest stable 3.x tag**

```bash
# Check https://github.com/eProsima/Fast-DDS/releases — pick newest v3.*.* stable
# Document chosen tag in commit message (e.g. v3.6.2)
```

- [ ] **Step 2: Rewrite FastDDS.cmake (sketch)**

```cmake
set(AUTOLINK_FASTDDS_GIT_TAG "v3.6.2")  # replace with Step 1 result

# Helper: if ConfigVersion reports < 3.0 → FATAL_ERROR with upgrade message

if(AUTOLINK_ENABLE_FASTDDS)
  find_package(fastdds 3 QUIET)
  if(fastdds_FOUND)
    if(fastdds_VERSION VERSION_LESS "3.0")
      message(FATAL_ERROR "autolink requires Fast DDS >= 3.0 (found ${fastdds_VERSION})")
    endif()
    set(AUTOLINK_HAS_FASTDDS ON)
    set(AUTOLINK_FASTDDS_LINK_LIBS fastdds fastcdr)
  else()
    # Optional: detect fastrtps/fastdds 2.x on prefix → FATAL (do not Fetch silently over 2.x confusion)
    # OpenSSL REQUIRED for SECURITY
    find_package(OpenSSL REQUIRED)  # or QUIET + FATAL with brew hint (keep M6 text)
    set(SECURITY ON CACHE BOOL "Enable Fast DDS Security" FORCE)
    # THIRDPARTY / COMPILE_* flags as today
    FetchContent_Declare(fastdds
      GIT_REPOSITORY https://github.com/eProsima/Fast-DDS.git
      GIT_TAG ${AUTOLINK_FASTDDS_GIT_TAG}
      GIT_SHALLOW TRUE)
    FetchContent_MakeAvailable(fastdds)
    set(AUTOLINK_HAS_FASTDDS ON)
    # 3.x target name is typically fastdds — verify after MakeAvailable:
    if(TARGET fastdds)
      set(AUTOLINK_FASTDDS_LINK_LIBS fastdds fastcdr)
    elseif(TARGET fastrtps)
      # only if 3.x still exports alias; prefer fastdds
      set(AUTOLINK_FASTDDS_LINK_LIBS fastrtps fastcdr)
      message(WARNING "autolink: linking fastrtps alias; prefer fastdds target")
    else()
      message(FATAL_ERROR "autolink: Fast DDS FetchContent produced no fastdds/fastrtps target")
    endif()
  endif()
endif()
```

Remove: `find_package(fastrtps 2.14)`、`v2.14.6`、`autolink_warn_if_fastdds_3x_installed` WARNING-only soft path（改为 FATAL on 2.x）。

- [ ] **Step 3: Wipe + configure ON（预期：库可编，autolink 可能因 API 编不过 — 可接受至 Task 2/3）**

```bash
rm -rf autolink/build-fastdds
cmake -S autolink -B autolink/build-fastdds -DAUTOLINK_ENABLE_FASTDDS=ON -DAUTOLINK_BUILD_TEST=ON \
  ${OPENSSL_ROOT_DIR:+-DOPENSSL_ROOT_DIR="$OPENSSL_ROOT_DIR"}
```

Expected: CMake 显示 pin 3.x；若立即 `cmake --build ... --target fastdds` 或 full autolink 失败在 `participant.cpp` 等，进入 Task 2（不要回退 2.14）。

- [ ] **Step 4: OFF configure still works（不拉 Fast DDS）**

```bash
cmake -S autolink -B autolink/build -DAUTOLINK_ENABLE_FASTDDS=OFF -DAUTOLINK_BUILD_TEST=ON
cmake --build autolink/build -j --target autolink
```

Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git commit -m "$(cat <<'EOF'
build(autolink): hard-cut Fast DDS FetchContent and find_package to 3.x
EOF
)"
```

---

### Task 2: Migrate Participant (+ Security properties namespaces)

**Files:**
- Modify: `autolink/autolink/transport/rtps/participant.cpp`
- Modify: `autolink/autolink/transport/rtps/participant.hpp` if includes need update

**Interfaces:**
- Consumes: Fast DDS 3.x headers after Task 1
- Produces: Participant Init compiles and runs on 3.x（Security 属性表语义同 M6）

- [ ] **Step 1: Replace intraprocess settings (UPGRADING)**

Remove:

```cpp
eprosima::fastrtps::LibrarySettingsAttributes ls =
    eprosima::fastrtps::xmlparser::XMLProfileManager::library_settings();
ls.intraprocess_delivery = eprosima::fastrtps::INTRAPROCESS_FULL;
eprosima::fastrtps::xmlparser::XMLProfileManager::library_settings(ls);
```

Add (adjust exact type names to 3.x headers):

```cpp
eprosima::fastdds::LibrarySettings ls;
ls.intraprocess_delivery = eprosima::fastdds::INTRAPROCESS_FULL;  // verify enum path
DomainParticipantFactory::get_instance()->set_library_settings(ls);
```

- [ ] **Step 2: Namespace sweep in participant.cpp**

- `eprosima::fastrtps::rtps::DiscoveryProtocol_t` → `eprosima::fastdds::rtps::...`（以 3.x 头为准）
- `IPLocator` / `Locator_t` / `ReturnCode_t` → `fastdds` 路径
- Drop `#include "fastrtps/..."` where replaced
- Discovery Server：`load_environment_server_info` — 若仍存在于 `fastdds::rtps` 则改命名空间；若 API 变更，按 3.x Discovery Server CLIENT 示例改写，保持 env `ip:port` 列表语义
- Security：`dds.sec.*` PropertyPolicy 字符串优先不变；若 create_participant 因插件属性失败，查 3.x Security 文档微调键名

- [ ] **Step 3: Build Participant objects**

```bash
cmake --build autolink/build-fastdds -j --target autolink 2>&1 | tee /tmp/m7-build.log
```

Expected: 错误从 participant 移开或整库通过；剩余错误交给 Task 3。

- [ ] **Step 4: Commit（可与 Task 3 合并若一次编过；否则先提交 participant）**

```bash
git commit -m "$(cat <<'EOF'
feat(autolink): migrate RTPS Participant to Fast DDS 3.x APIs
EOF
)"
```

---

### Task 3: Migrate Underlay, Dispatcher, Transmitter, Topology

**Files:**
- Modify: `autolink/autolink/transport/rtps/underlay_message_type.hpp`
- Modify: `autolink/autolink/transport/rtps/underlay_message_type.cpp`
- Modify: `autolink/autolink/transport/dispatcher/rtps_dispatcher.hpp`
- Modify: `autolink/autolink/transport/transmitter/rtps_transmitter.hpp`
- Modify: `autolink/autolink/service_discovery/rtps_topology_backend.cpp`
- Grep-fix any remaining `fastrtps` under `autolink/autolink/`

**Interfaces:**
- TypeSupport serialize/deserialize signatures must match 3.x `TopicDataType`
- `write` / `take_next_sample` 继续用 `ReturnCode_t::RETCODE_OK`（正确命名空间）

- [ ] **Step 1: Grep inventory**

```bash
rg -n 'fastrtps|XMLProfileManager|LibrarySettingsAttributes|SerializedPayload|md5\.h' \
  autolink/autolink --glob '*.{hpp,cpp}'
```

- [ ] **Step 2: underlay_message_type**

Replace includes:

```cpp
// Was: fastrtps/rtps/common/SerializedPayload.h, InstanceHandle.h, utils/md5.h
// Use fastdds equivalents per 3.x tree (check include paths after FetchContent)
```

Update method signatures / `using` aliases so overrides still match base `TopicDataType`.

- [ ] **Step 3: dispatcher / transmitter / topology**

- Fix includes and `ReturnCode_t` / QoS types
- Keep MessageInfo prefix / topology no-prefix behavior unchanged

- [ ] **Step 4: Full ON link of `autolink`**

```bash
cmake --build autolink/build-fastdds -j --target autolink
```

Expected: PASS（零 `fastrtps` 编译错误）。

- [ ] **Step 5: Smoke tests**

```bash
unset AUTOLINK_RTPS_SECURITY AUTOLINK_RTPS_SECURITY_DIR
./autolink/build-fastdds/bin/autolink.transport.integration_test.rtps_transceiver_test
./autolink/build-fastdds/bin/autolink.transport.rtps.rtps_security_hub_test
```

Expected: PASS.

- [ ] **Step 6: Commit**

```bash
git commit -m "$(cat <<'EOF'
feat(autolink): migrate RTPS underlay and endpoints to Fast DDS 3.x
EOF
)"
```

---

### Task 4: Full regression + docs + spec status

**Files:**
- Modify: `docs/source/03_Communication/14_fastdds.md`
- Modify: `autolink/CHANGELOG.rst`
- Modify: `docs/superpowers/specs/2026-09-07-autolink-fastdds-3x-design.md` → 状态 `已实现（M7）`

- [ ] **Step 1: Update §14**

- 基线：Fast DDS **3.x** pin（写明实际 GIT_TAG）
- CMake：`find_package(fastdds 3)` / 目标 `fastdds`；**不支持 2.14**
- 删除「3.x 未测 WARNING / 默认 2.14」主叙事
- 保留 Security / Discovery Server / payload env；注明 API 已跟 3.x
- 构建仍需 OpenSSL when FetchContent

- [ ] **Step 2: CHANGELOG M7 bullets**

- [ ] **Step 3: Regression matrix**

| Build | Tests |
|-------|--------|
| OFF | `topology_backend_factory_test`, `security_config_test` |
| ON | factory, `rtps_topology_backend_test`, `rtps_transceiver_test`, `payload_limit_test`, `rtps_stats_test`, `security_config_test`, `rtps_security_hub_test` |

All PASS.

- [ ] **Step 4: Confirm no stray 2.14 pins**

```bash
rg -n '2\.14|v2\.14|fastrtps 2' autolink/cmake docs/source/03_Communication/14_fastdds.md
```

Expected: 无「默认 2.14」；历史提及仅作迁移说明可接受。

- [ ] **Step 5: Mark spec implemented; commit submodule + parent**

```bash
git commit -m "$(cat <<'EOF'
docs(autolink): baseline Fast DDS 3.x and mark M7 implemented
EOF
)"
```

---

## Spec coverage

| Spec | Task |
|------|------|
| CMake 3.x only + FATAL on 2.x | 1 |
| FetchContent pin + SECURITY | 1 |
| Participant / Discovery / Security API | 2 |
| Underlay / dispatcher / tx / topology | 3 |
| ON/OFF 回归 + §14 + CHANGELOG + 状态 | 4 |
| 不做默认开 FastDDS / 不做 2.14 双轨 | Global |

## Consistency

- 链接目标优先 `fastdds`；仅当 3.x 仍导出 `fastrtps` 别名时才回退并 WARNING
- Security 键名优先保持 M6；仅在 3.x 官方变更时改
- Task 1 后库可能暂不可链 — 以 Task 3 编过为硬门；不要为绿而 pin 回 2.14
