# Autolink FastDDS Security 设计（M6）

- **日期**：2026-09-07
- **状态**：已实现（M6）
- **计划**：`docs/superpowers/plans/2026-09-07-autolink-fastdds-security.md`
- **前置**：
  - `docs/superpowers/specs/2026-09-06-autolink-fastdds-multimachine-design.md`（M1–M3）
  - `docs/superpowers/specs/2026-09-07-autolink-fastdds-topology-ops-design.md`（M4–M5 已实现）
- **范围**：Fast DDS **2.14** 上可选 DDS Security（Auth + Crypto；Access 仅 allow-all）
- **不做（本轮）**：细粒度 Permissions/ACL；FastDDS 3.x 默认化（**M7** 另开 spec）；证书轮换/PKI 服务/HSM；强制默认开启 Security；明文/密文双栈混跑

## 1. 背景与目标

M1–M5 已提供跨机 RTPS 数据面与拓扑面，但链路为明文，仅适合可信局域网。本设计在 **不改变默认行为** 的前提下，提供 opt-in 加密通道。

排期约定：先 Security（钉 2.14），再另开 **M7** 做 FastDDS 3.x 默认化（届时跟迁 Security 属性与 CMake）。

**成功标准**

1. `AUTOLINK_RTPS_SECURITY=1` 且证书目录有效：topology 与 transport 两个 Participant 均启用 Auth + Crypto；同 CA、不同 identity 的两进程可互通 ChangeMsg 与 Channel。
2. 一方 Security ON、一方 OFF：不能明文互通（握手失败或无业务回调），无静默旁路。
3. Security 未开启：与现网一致；既有 OFF/ON 回归（factory、topology、transceiver、payload、stats）PASS。
4. 开启但证书缺失/无效/插件失败：`RtpsParticipantHub::Init` 失败并 AERROR（fail-loud）。
5. §14 文档含目录约定、生成步骤、双机加密清单；明确 Permissions 本轮为 allow-all。

## 2. 决策摘要

| 项 | 选择 |
|----|------|
| 排期 | 先 M6 Security（2.14），后 M7 3.x 默认化 |
| 能力 | Authentication + Cryptography；Access 仅 allow-all 模板 |
| 启用 | 纯 opt-in（`AUTOLINK_RTPS_SECURITY=1`） |
| 证书 | `AUTOLINK_RTPS_SECURITY_DIR` 目录约定 |
| 失败 | Fail-loud：不回退明文 |
| 接线 | Participant `PropertyPolicyQos`（方案 A） |
| 身份 | 同进程 topology/transport **共用**同一 identity |
| 与 Discovery Server | 正交；两端 Security 配置须一致 |

## 3. 架构

```text
Env: AUTOLINK_RTPS_SECURITY=1
     AUTOLINK_RTPS_SECURITY_DIR=$DIR
                │
                ▼
        RtpsParticipantHub::Init
          ├─ validate $DIR files
          └─ create Participants with Security properties
                ├─ …:topology
                └─ …:transport
                     │
                     ├─ DDS:Auth:PKI-DH
                     ├─ DDS:Access:Permissions  (allow-all)
                     └─ DDS:Crypto:AES-GCM-GMAC
```

未设 Security 时：Hub/Participant 路径与 M5 完全相同（无插件属性）。

## 4. 环境变量与证书布局

| 变量 | 默认 | 含义 |
|------|------|------|
| `AUTOLINK_RTPS_SECURITY` | 未设（关） | `1` = 启用 |
| `AUTOLINK_RTPS_SECURITY_DIR` | 无 | 启用时必填；证书与签名策略目录 |

**`$DIR` 必备文件**

```text
$DIR/
  identity_ca.crt
  permissions_ca.crt    # 可与 identity CA 相同材料的副本或同 CA
  cert.pem              # 本节点 identity 证书
  key.pem               # 本节点私钥
  governance.smime      # 已签名 Governance（要求加密；策略放宽）
  permissions.smime     # 已签名 Permissions（allow-all）
```

- 仓库不提交生产私钥。
- 文档给出 openssl + Fast DDS Security 工具的生成步骤；实现阶段可增加开发用生成脚本（例如 `scripts/gen_rtps_security_dev.sh`），不作为本设计硬性交付物。

## 5. 实现要点

### 5.1 Hub 校验

`RtpsParticipantHub::Init` 在 Security=1 时：

1. 检查 `AUTOLINK_RTPS_SECURITY_DIR` 非空且为目录。
2. 检查上表必备文件可读。
3. 任一失败 → AERROR、不创建 Participant、返回 false。

拓扑 `BACKEND=rtps` 时既有「Start 失败 → fallback local」逻辑仍适用；但 **Security 开启且 Hub Init 因证书失败** 属于配置错误，应 fail-loud（与「不静默明文」一致）。Transport 无 Participant 时既有 fail-loud 路径保持。

### 5.2 Participant PropertyPolicy

在现有 `Participant::Init` QoS 组装之后、`create_participant` 之前，若 Security 启用，注入 Fast DDS 2.14 标准属性，指向 `$DIR` 内文件，启用：

- `DDS:Auth:PKI-DH`
- `DDS:Access:Permissions`（allow-all，非业务 ACL）
- `DDS:Crypto:AES-GCM-GMAC`

具体 property 名与文件 URI 格式对齐 eProsima Fast DDS 2.14 Security 文档（实现时以官方示例为准，本 spec 不锁定逐字段字符串以免与小版本漂移冲突）。

### 5.3 CMake / 依赖

- FetchContent Fast DDS **2.14.x** 构建需 **`SECURITY=ON`** 并链接 OpenSSL。
- 系统 `find_package` 的 Fast DDS 若未编 Security：运行时插件加载失败 → Init fail-loud；CMake 尽可能 WARNING。
- `AUTOLINK_ENABLE_FASTDDS=OFF`：不引入 Security 代码路径。

### 5.4 与 Discovery Server

`AUTOLINK_DISCOVERY_SERVER` 与 Security 独立配置。加密域内两端均须 Security ON 且信任同一 Identity CA。

## 6. 测试计划

| 用例 | 条件 |
|------|------|
| Security OFF 回归 | 既有 factory / topology / transceiver / payload / stats |
| 缺证 fail-loud | `SECURITY=1` 缺必备文件 → Hub Init false |
| 同 CA 双 identity 互通 | 临时 `$DIR_A`/`$DIR_B`；有 OpenSSL 则跑，否则 SKIP |
| 一侧 ON 一侧 OFF | 无明文业务互通 |
| FastDDS OFF | Security 测 SKIP；库不链接插件 |

## 7. 文档

更新 `docs/source/03_Communication/14_fastdds.md`：

- Security 开关与目录约定
- 证书生成步骤
- 双机加密清单
- 与 Discovery Server 正交说明
- **Permissions 本轮仅为 allow-all**；细粒度 ACL 不在本里程碑

`autolink/CHANGELOG.rst` 增加 M6 条目。

## 8. 里程碑

| 里程碑 | 交付 |
|--------|------|
| **M6** | 本 spec：2.14 Security opt-in（Auth+Crypto + allow-all Access） |
| **M7** | 另开 spec：FastDDS 3.x 默认化（CMake `fastdds`、API 迁移、Security 跟迁） |

## 9. 风险与缓解

| 风险 | 缓解 |
|------|------|
| FetchContent + SECURITY 编时长 / OpenSSL 路径 | 文档钉 OpenSSL；CI 可选开关 |
| `.smime` 签名工具链不熟 | 文档步骤 + 可选 dev 脚本 |
| 误以为已加密 | fail-loud；文档强调默认明文 |
| 3.x 升版时属性/目标更名 | M7 单独验收；M6 钉 2.14 |

## 10. 参考

- Fast DDS 2.14 Security（eProsima）
- 现码：`transport/rtps/participant.{hpp,cpp}`、`participant_hub.*`、`cmake/FastDDS.cmake`
- 前置：M1–M5 multimachine + topology-ops specs
