# Autolink FastDDS 3.x 硬切设计（M7）

- **日期**：2026-09-07
- **状态**：已批准（实现计划已就绪）
- **计划**：`docs/superpowers/plans/2026-09-07-autolink-fastdds-3x.md`
- **前置**：
  - M1–M5：多机 RTPS 数据面 + 拓扑/运维（2.14）
  - M6：`docs/superpowers/specs/2026-09-07-autolink-fastdds-security-design.md`（Security opt-in，已实现）
- **范围**：`AUTOLINK_ENABLE_FASTDDS=ON` 时硬切 Fast DDS **3.x**；API/CMake 迁移；M6 Security 跟迁回归
- **不做**：双轨支持 2.14；默认打开 `AUTOLINK_ENABLE_FASTDDS`；细粒度 Permissions；改 Underlay/拓扑载荷语义

## 1. 背景与目标

M1–M6 钉 Fast DDS **2.14**（FetchContent `v2.14.6`，CMake 目标常为 `fastrtps`）。3.x 为上游主线：命名空间与包名统一为 `fastdds`、依赖 Fast CDR v2、安全与工具链优先演进。本里程碑**不为新业务能力**，而为降低技术债并跟上游。

**决策摘要**

| 项 | 选择 |
|----|------|
| 兼容 | **硬切 3.x**，不保证 2.14 可编 |
| Pin | 实现时最新稳定 3.x tag（设计参考 **`v3.6.2`**，落地前核对 releases） |
| 开关 | `AUTOLINK_ENABLE_FASTDDS` **默认仍 OFF**；ON 后只认 3.x |
| 实现路线 | CMake 硬切 + 一次 API 迁完（无 shim、无双轨宏） |
| Security | **必须跟迁**；缺证 fail-loud 与明文回归均 PASS |

**成功标准**

1. ON 构建主链接目标为 **`fastdds`**（+ `fastcdr`）；FetchContent pin 为 3.x。
2. 环境仅有 Fast DDS 2.x 时 CMake **FATAL_ERROR**，不静默使用 2.14。
3. OFF 回归不受影响。
4. ON 明文：`rtps_transceiver`、`rtps_topology_backend`、`topology_backend_factory`、`payload_limit`、`rtps_stats` PASS。
5. ON Security：`security_config`、`rtps_security_hub`（缺证 fail-loud）PASS；有效证书时 Participant Init 不因 API 迁移失败。
6. §14 以 3.x 为唯一基线；去掉「默认 2.14 / 3.x 未测」叙事。

## 2. 架构与约束

```text
AUTOLINK_ENABLE_FASTDDS=ON
        │
        ▼
 FastDDS.cmake
   ├─ find_package(fastdds 3)  → link fastdds
   └─ else FetchContent v3.x.y + SECURITY=ON + OpenSSL
        │
        ▼
 transport/rtps/* + topology  （仅 fastdds::* API）
```

- 业务语义不变：Underlay 二进制、Channel 24B MessageInfo、拓扑无 MessageInfo、双 Participant Hub、既有 env 名（含 Security / Discovery Server / payload）。
- 参考：[Fast DDS UPGRADING.md（v2→v3）](https://github.com/eProsima/Fast-DDS/blob/master/UPGRADING.md)。

## 3. CMake

修改 `autolink/cmake/FastDDS.cmake`：

1. 删除 `find_package(fastrtps 2.14)` 与 `AUTOLINK_FASTDDS_GIT_TAG=v2.14.6`。
2. `find_package(fastdds 3 QUIET)`；成功 → `AUTOLINK_FASTDDS_LINK_LIBS = fastdds fastcdr`（以包实际导出为准）。
3. 未找到 → FetchContent：`GIT_TAG` = 最新稳定 3.x；`SECURITY ON`；`find_package(OpenSSL)` 失败则 FATAL（同 M6 提示）。
4. Soft-probe / `find_package` 若命中 major&lt;3 → **FATAL**，提示卸载 2.x 或改 `CMAKE_PREFIX_PATH`。
5. 首次 ON 建议 wipe `autolink/build-fastdds`，避免旧 `fastrtps` 缓存。

## 4. API 迁移面

按 UPGRADING 硬切；优先文件顺序：

1. `transport/rtps/participant.cpp` — `XMLProfileManager` → Factory `set_library_settings`；Discovery CLIENT；Security PropertyPolicy；`ReturnCode_t` 命名空间
2. `transport/rtps/underlay_message_type.{hpp,cpp}` — `SerializedPayload` / `InstanceHandle` / MD5 头与命名空间
3. `transport/dispatcher/rtps_dispatcher.hpp`、`transmitter/rtps_transmitter.hpp`
4. `service_discovery/rtps_topology_backend.cpp`
5. 相关测例 include / 断言

| 类别 | 2.14 现状 | 3.x 方向 |
|------|-----------|----------|
| 命名空间 | `eprosima::fastrtps::*` | `eprosima::fastdds::*` |
| Intraprocess | `XMLProfileManager::library_settings` | `DomainParticipantFactory::set_library_settings` |
| Discovery Server | `load_environment_server_info` / `m_DiscoveryServers` | 按 3.x CLIENT API |
| TypeSupport | `fastrtps::rtps::SerializedPayload_t` 等 | 新路径/类型 |
| CMake 目标 | `fastrtps` | `fastdds` |
| Security | M6 PropertyPolicy 字符串 | 核对同名；若变更则改注入表，语义不变 |

## 5. Security 跟迁

- FetchContent 保持 **`SECURITY=ON`**。
- Hub `SecurityConfig::Validate` fail-loud **不变**。
- Participant 注入属性：优先保持 M6 键名；若 3.x 文档变更则同步，仍为 Auth PKI-DH + Access allow-all + Crypto AES-GCM-GMAC。
- 不扩展细粒度 Permissions。

## 6. 测试计划

| 用例 | 条件 |
|------|------|
| OFF 回归 | factory、security_config 等 |
| ON 明文套件 | transceiver / topology / factory / payload / stats |
| ON Security | security_hub 缺证；security_config |
| CMake 拒绝 2.x | 文档或手工：prefix 仅 2.14 时 FATAL |
| 双机 / 带证握手 | 正式验收仍靠清单；CI 不强制 |

## 7. 文档

更新 `docs/source/03_Communication/14_fastdds.md`：

- 基线改为 3.x pin；删除 2.14 默认与「3.x 未测 WARNING」为主叙事
- CMake 目标 `fastdds`；OpenSSL / SECURITY 说明保留
- Security / Discovery Server 节按 3.x 修订（若 API/路径有差）
- `autolink/CHANGELOG.rst` 增加 M7 条目

## 8. 里程碑与风险

| 里程碑 | 交付 |
|--------|------|
| **M7** | 本 spec：硬切 3.x + API + Security 跟迁 + 文档 |

| 风险 | 缓解 |
|------|------|
| FetchContent 全量重编耗时 | wipe 一次；CI 缓存可选 |
| OpenSSL / SECURITY | 沿用 M6 FATAL 提示 |
| UPGRADING 个别符号差异 | 以 3.x.y 官方文档为准微调 |
| 误混 2.14 系统包 | FATAL 而非 WARNING |

## 9. 参考

- Fast DDS UPGRADING.md（v2→v3）
- 现码：`cmake/FastDDS.cmake`、`transport/rtps/*`、`rtps_topology_backend.cpp`
- 前置 spec：multimachine、topology-ops、security（M6）
