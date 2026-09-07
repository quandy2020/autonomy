# Autolink FastDDS Security (M6) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 在 Fast DDS 2.14 上为 topology/transport Participant 提供纯 opt-in 的 DDS Security（Auth + Crypto；Access 仅 allow-all），缺证 fail-loud，默认明文行为不变。

**Architecture:** `security_config` 解析 env 并校验证书目录；`RtpsParticipantHub::Init` 在启用时先校验再创建 Participant；`Participant::Init` 注入 Fast DDS 2.14 `PropertyPolicy`（PKI-DH / Access-Permissions / AES-GCM-GMAC）。FetchContent 构建开启 `SECURITY=ON` + OpenSSL。

**Tech Stack:** C++17、Fast DDS 2.14、OpenSSL、GTest、既有 `RtpsParticipantHub` / `Participant`

**Spec:** `docs/superpowers/specs/2026-09-07-autolink-fastdds-security-design.md`

## Global Constraints

- Fast DDS 钉 **2.14.x**（现 `AUTOLINK_FASTDDS_GIT_TAG=v2.14.6`）；不做 3.x 默认化（M7）
- Security **默认关**；仅 `AUTOLINK_RTPS_SECURITY=1` 启用
- 启用时必须 `AUTOLINK_RTPS_SECURITY_DIR` + 六文件齐全；失败 **不回退明文**
- Access 仅为 allow-all（满足插件链）；不做 topic ACL
- `autolink/` 是 **git submodule** — 先 submodule commit，再 bump 父仓
- OFF 构建：`AUTOLINK_ENABLE_FASTDDS=0`，Security 源可编译测试辅助但不链接 DDS 插件

**Build / test:**

```bash
# OFF
cmake -S autolink -B autolink/build -DAUTOLINK_ENABLE_FASTDDS=OFF -DAUTOLINK_BUILD_TEST=ON
cmake --build autolink/build -j --target autolink
./autolink/build/bin/autolink.transport.rtps.security_config_test

# ON（首次开 SECURITY 后建议清 build-fastdds 再配）
cmake -S autolink -B autolink/build-fastdds -DAUTOLINK_ENABLE_FASTDDS=ON -DAUTOLINK_BUILD_TEST=ON
cmake --build autolink/build-fastdds -j --target autolink
./autolink/build-fastdds/bin/autolink.transport.rtps.security_config_test
./autolink/build-fastdds/bin/autolink.transport.integration_test.rtps_transceiver_test
# + security hub / handshake tests as added
```

---

## File Map

| 路径 | 职责 |
|------|------|
| `autolink/.../transport/rtps/security_config.hpp` | `SecurityConfig`：FromEnv、ValidateDir、必备文件名 |
| `autolink/.../transport/rtps/security_config.cpp` | 实现（可 OFF/ON 均编译） |
| `autolink/.../transport/rtps/security_config_test.cpp` | env / 缺文件单测 |
| `autolink/.../transport/rtps/participant.hpp/.cpp` | 启用时注入 Security PropertyPolicy |
| `autolink/.../transport/rtps/participant_hub.cpp` | Init 前 Validate；失败不建 Participant |
| `autolink/cmake/FastDDS.cmake` | FetchContent `SECURITY=ON`；找 OpenSSL |
| `autolink/.../transport/rtps/rtps_security_hub_test.cpp` | Hub 缺证 fail-loud（ON） |
| `docs/source/03_Communication/14_fastdds.md` | Security 节 |
| `autolink/CHANGELOG.rst` | M6 |
| 可选 `autolink/scripts/gen_rtps_security_dev.sh` | 开发证书生成（非硬性） |

---

### Task 1: SecurityConfig helper + unit tests

**Files:**
- Create: `autolink/autolink/transport/rtps/security_config.hpp`
- Create: `autolink/autolink/transport/rtps/security_config.cpp`
- Create: `autolink/autolink/transport/rtps/security_config_test.cpp`
- Modify: `autolink/CMakeLists.txt`（加入源与测试目标；**不**要求 FastDDS）

**Interfaces:**
- Produces:
```cpp
namespace autolink::transport {
struct SecurityConfig {
  bool enabled = false;
  std::string dir;  // absolute or as given
  static SecurityConfig FromEnv();
  // Returns false + sets err if enabled but dir/files invalid.
  bool Validate(std::string* err) const;
  static const char* kIdentityCa;       // "identity_ca.crt"
  static const char* kPermissionsCa;    // "permissions_ca.crt"
  static const char* kCert;             // "cert.pem"
  static const char* kKey;              // "key.pem"
  static const char* kGovernance;       // "governance.smime"
  static const char* kPermissions;      // "permissions.smime"
  std::string Path(const char* filename) const;  // dir + "/" + filename
};
}
```
- `FromEnv`: `AUTOLINK_RTPS_SECURITY=="1"` → enabled；`AUTOLINK_RTPS_SECURITY_DIR` → dir（trim）。其它值 = disabled。
- `Validate`: disabled → true；enabled 且 dir 空/非目录/缺任一文件 → false + err 说明缺失项。

- [ ] **Step 1: Write failing tests**

```cpp
TEST(SecurityConfigTest, FromEnvDisabledByDefault) {
  unsetenv("AUTOLINK_RTPS_SECURITY");
  unsetenv("AUTOLINK_RTPS_SECURITY_DIR");
  auto c = SecurityConfig::FromEnv();
  EXPECT_FALSE(c.enabled);
  std::string err;
  EXPECT_TRUE(c.Validate(&err));
}

TEST(SecurityConfigTest, EnabledMissingDirFailsValidate) {
  setenv("AUTOLINK_RTPS_SECURITY", "1", 1);
  unsetenv("AUTOLINK_RTPS_SECURITY_DIR");
  auto c = SecurityConfig::FromEnv();
  ASSERT_TRUE(c.enabled);
  std::string err;
  EXPECT_FALSE(c.Validate(&err));
  EXPECT_FALSE(err.empty());
  unsetenv("AUTOLINK_RTPS_SECURITY");
}

TEST(SecurityConfigTest, EnabledEmptyDirFails) {
  // create temp dir without files; SECURITY=1 + DIR=temp → Validate false
}
```

- [ ] **Step 2: Run tests — expect FAIL (missing symbols)**

```bash
# after adding target to CMake
cmake --build autolink/build -j --target autolink.transport.rtps.security_config_test
./autolink/build/bin/autolink.transport.rtps.security_config_test
```

Expected: link/compile fail or FAIL until Step 3.

- [ ] **Step 3: Implement SecurityConfig**

Header-only 可，但 `.cpp` 便于测试链接。用 `std::filesystem`（C++17）检查 `is_directory` / `is_regular_file`。

- [ ] **Step 4: Run tests — expect PASS**

- [ ] **Step 5: Commit submodule + parent bump**

```bash
git commit -m "$(cat <<'EOF'
feat(autolink): add RTPS SecurityConfig env and directory validation
EOF
)"
```

---

### Task 2: CMake — FetchContent SECURITY=ON + OpenSSL

**Files:**
- Modify: `autolink/cmake/FastDDS.cmake`

**Interfaces:**
- Consumes: existing FetchContent block for `fastdds`
- Produces: Fast DDS built with security plugins when FetchContent path used

- [ ] **Step 1: Before FetchContent_Declare, set**

```cmake
find_package(OpenSSL REQUIRED)
set(SECURITY ON CACHE BOOL "Enable Fast DDS Security" FORCE)
# Keep other existing CACHE FORCE flags
```

若 `find_package(OpenSSL)` 失败：在 `AUTOLINK_ENABLE_FASTDDS=ON` 时 `message(FATAL_ERROR ...)` 提示安装 openssl（macOS: `brew install openssl` 并设 `OPENSSL_ROOT_DIR`）。

- [ ] **Step 2: Reconfigure ON build (clean recommended)**

```bash
rm -rf autolink/build-fastdds
cmake -S autolink -B autolink/build-fastdds -DAUTOLINK_ENABLE_FASTDDS=ON -DAUTOLINK_BUILD_TEST=ON
cmake --build autolink/build-fastdds -j --target autolink
```

Expected: 配置成功；编过 fastrtps（耗时可能较长）。

- [ ] **Step 3: Smoke OFF still configures**

```bash
cmake -S autolink -B autolink/build -DAUTOLINK_ENABLE_FASTDDS=OFF -DAUTOLINK_BUILD_TEST=ON
cmake --build autolink/build -j --target autolink
```

Expected: PASS（不要求 OpenSSL）。

- [ ] **Step 4: Commit**

```bash
git commit -m "$(cat <<'EOF'
build(autolink): enable Fast DDS SECURITY and OpenSSL for FetchContent
EOF
)"
```

---

### Task 3: Wire Hub validate + Participant PropertyPolicy

**Files:**
- Modify: `autolink/autolink/transport/rtps/participant.hpp`
- Modify: `autolink/autolink/transport/rtps/participant.cpp`
- Modify: `autolink/autolink/transport/rtps/participant_hub.cpp`
- Create: `autolink/autolink/transport/rtps/rtps_security_hub_test.cpp`（`#if AUTOLINK_ENABLE_FASTDDS`）

**Interfaces:**
- Consumes: `SecurityConfig::FromEnv()` / `Validate()` / `Path()`
- Participant：可选 `explicit` 增加 `const SecurityConfig*` 或在 Init 内自行 `FromEnv()`（推荐 **Init 内 FromEnv**，Hub 与 Participant 一致，避免双源）。
- Hub：`Init` 开头：

```cpp
auto sec = SecurityConfig::FromEnv();
std::string err;
if (!sec.Validate(&err)) {
  AERROR << "RTPS Security config invalid: " << err;
  return false;
}
```

- Participant：在 `create_participant` 前，若 `sec.enabled`：

```cpp
auto& props = qos.properties().properties();
const std::string file_prefix = "file://";
props.emplace_back("dds.sec.auth.plugin", "builtin.PKI-DH");
props.emplace_back("dds.sec.auth.builtin.PKI-DH.identity_ca",
                   file_prefix + sec.Path(SecurityConfig::kIdentityCa));
props.emplace_back("dds.sec.auth.builtin.PKI-DH.identity_certificate",
                   file_prefix + sec.Path(SecurityConfig::kCert));
props.emplace_back("dds.sec.auth.builtin.PKI-DH.private_key",
                   file_prefix + sec.Path(SecurityConfig::kKey));
props.emplace_back("dds.sec.access.plugin", "builtin.Access-Permissions");
props.emplace_back("dds.sec.access.builtin.Access-Permissions.permissions_ca",
                   file_prefix + sec.Path(SecurityConfig::kPermissionsCa));
props.emplace_back("dds.sec.access.builtin.Access-Permissions.governance",
                   file_prefix + sec.Path(SecurityConfig::kGovernance));
props.emplace_back("dds.sec.access.builtin.Access-Permissions.permissions",
                   file_prefix + sec.Path(SecurityConfig::kPermissions));
props.emplace_back("dds.sec.crypto.plugin", "builtin.AES-GCM-GMAC");
```

若官方 2.14.6 属性名有差异，以 eProsima 2.14 Security 示例为准微调，但保持上表语义。

`create_participant` 返回 nullptr → 现有 AERROR + return false（插件缺失时 fail-loud）。

- [ ] **Step 1: Failing hub test (ON)**

```cpp
#if AUTOLINK_ENABLE_FASTDDS
TEST(RtpsSecurityHubTest, InitFailsWhenSecurityEnabledWithoutCerts) {
  setenv("AUTOLINK_RTPS_SECURITY", "1", 1);
  unsetenv("AUTOLINK_RTPS_SECURITY_DIR");
  RtpsParticipantHub::Instance().Shutdown();
  proto::RtpsParticipantAttr attr;  // minimal / from GlobalData defaults
  EXPECT_FALSE(RtpsParticipantHub::Instance().Init(attr));
  unsetenv("AUTOLINK_RTPS_SECURITY");
}
#endif
```

- [ ] **Step 2: Run — expect FAIL (Init still succeeds without validate)**

- [ ] **Step 3: Implement Hub Validate + Participant properties**

- [ ] **Step 4: Run hub test PASS；Security OFF 下 `rtps_transceiver_test` PASS**

```bash
unset AUTOLINK_RTPS_SECURITY AUTOLINK_RTPS_SECURITY_DIR
./autolink/build-fastdds/bin/autolink.transport.rtps.rtps_security_hub_test
./autolink/build-fastdds/bin/autolink.transport.integration_test.rtps_transceiver_test
```

- [ ] **Step 5: Commit**

```bash
git commit -m "$(cat <<'EOF'
feat(autolink): wire DDS Security PropertyPolicy on RTPS participants
EOF
)"
```

---

### Task 4: Docs + CHANGELOG + optional gen script + spec status

**Files:**
- Modify: `docs/source/03_Communication/14_fastdds.md`
- Modify: `autolink/CHANGELOG.rst`
- Modify: `docs/superpowers/specs/2026-09-07-autolink-fastdds-security-design.md`（状态 → 已实现（M6）在全部绿灯后）
- Optional Create: `autolink/scripts/gen_rtps_security_dev.sh`

- [ ] **Step 1: Update §14**

新增小节（或改写原 “Security 未实现”）：

- env 表增加 `AUTOLINK_RTPS_SECURITY` / `AUTOLINK_RTPS_SECURITY_DIR`
- 目录六文件约定
- openssl 生成 + `openssl smime -sign` 签 governance/permissions（可引用 Fast DDS `test/certs` 思路）
- 双机清单：两端同 CA、各自身份证、均 `SECURITY=1`
- 明确 Permissions **allow-all**；默认仍明文
- Discovery Server 与 Security 正交

- [ ] **Step 2: CHANGELOG M6 bullets**

- [ ] **Step 3: Optional script** — 在临时目录生成一对 identity + allow-all 已签名 XML；失败则文档步骤即可，脚本非阻塞。

- [ ] **Step 4: Regression**

OFF: `security_config_test`  
ON: `security_config_test` + `rtps_security_hub_test` + `rtps_transceiver_test` + factory/topology/payload/stats

- [ ] **Step 5: Mark spec `已实现（M6）`；commit**

```bash
git commit -m "$(cat <<'EOF'
docs(autolink): document RTPS Security M6 enablement
EOF
)"
# parent: docs + submodule bump + spec status
```

**Handshake 互通测（可选增强，不阻塞 M6）：** 若本机有 openssl 且脚本能生成 `$DIR_A`/`$DIR_B`，可加 `AUTOLINK_RTPS_SECURITY_TEST=1` 门控的双 Participant 冒烟；否则 GTEST_SKIP。一侧 ON 一侧 OFF 断言无业务回调。优先手工双机验收。

---

## Spec coverage

| Spec 要求 | Task |
|-----------|------|
| SecurityConfig env + 目录校验 | 1 |
| FetchContent SECURITY + OpenSSL | 2 |
| Hub fail-loud + PropertyPolicy | 3 |
| 文档 / CHANGELOG / 状态 | 4 |
| Auth+Crypto / allow-all Access | 3（属性）+ 4（文档） |
| OFF 回归 | 1、3、4 |
| 不做细粒度 ACL / 3.x | 全局约束 |
| 双 identity 互通 | Task 4 可选；正式靠双机 |

## Consistency

- Property 前缀 `file://` + `SecurityConfig::Path`
- Hub 与 Participant 均以 `FromEnv()` 为准（进程生命周期内勿中途改 env）
- Security Init 失败 ≠ 拓扑 local fallback 的「明文 RTPS」；无 Participant 时 Transport 保持既有错误路径
