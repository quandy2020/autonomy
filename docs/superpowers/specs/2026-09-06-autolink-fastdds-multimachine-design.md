# Autolink FastDDS 多机通信设计（A→B→C）

- **日期**：2026-09-06
- **状态**：待用户审阅
- **来源**：Apollo Cyber RT 孵化；Autolink 已移除 FastDDS，仅保留本机 INTRA + SHM
- **范围**：文档/配置真相（A）→ 可插拔拓扑后端工厂（B）→ Fast DDS **2.14** 可选数据面 + Hybrid `diff_host→RTPS`（C）
- **不做（本轮）**：Discovery Server、DDS Security、大消息外带、完整 RTPS 拓扑 SDP（`RtpsTopologyBackend` 实装）、Fast DDS 3.x 编译、双 Participant 拆分

## 1. 背景与目标

Autolink 从 Apollo Cyber 孵化后去掉了跨机路径：无 `transport/rtps/`，`OptionalMode::RTPS` 创建失败或静默 fallback SHM；拓扑发现改为本机 `/tmp` 文件总线。仓库内部分通信文档仍描述「跨机 RTPS / `AUTOLINK_DOMAIN_ID`」，与实现不符。

本设计按 **A→B→C** 恢复可选多机能力，并与最新可维护的 Fast DDS 版本对齐（默认钉 **2.14 LTS**，文档预留 3.x 升级路径）。

**成功标准**

1. `AUTOLINK_ENABLE_FASTDDS=OFF`：现有 INTRA/SHM/Hybrid 回归通过；项目无 FastDDS 也可编译。
2. `=ON`：同网段两进程/两机 Channel pub/sub 可通；`AUTOLINK_IP`、`AUTOLINK_DOMAIN_ID` 真正生效。
3. conf 写 `RTPS` 但未启用库时：**明确失败**，禁止再静默当 SHM。
4. 文档与 `autolink.pb.conf` 不再暗示「默认已支持跨机 RTPS」。

## 2. 决策摘要

| 项 | 选择 |
|----|------|
| 落地顺序 | A（文档/conf）→ B（Backend 工厂）→ C（RTPS 数据面） |
| 做法 | 薄适配 + 可插拔；不一次复刻 Cyber 全栈 |
| FastDDS 依赖 | CMake `AUTOLINK_ENABLE_FASTDDS`（默认 OFF）+ 钉官方 **2.14.x**（find_package 或 FetchContent） |
| 发现（P0） | DDS **SIMPLE 多播**；Discovery Server 归后续子项目 |
| 拓扑 | 默认仍 `LocalTopologyBackend`；`rtps` 后端仅预留，本轮不实装 |
| Participant | P0 每进程 **一个** DomainParticipant（不拆 topology/transport） |
| `diff_host` 默认 | 保持 **SHM**；跨机需显式 `diff_host: RTPS` 且 ENABLE=ON |

## 3. 架构

```text
Node / Writer / Reader
        │
        ▼
   Transport
   ├─ HYBRID → SAME_PROC: INTRA | DIFF_PROC: SHM | DIFF_HOST: conf（可 RTPS*）
   ├─ INTRA / SHM（始终可编）
   └─ RTPS* → 仅 AUTOLINK_ENABLE_FASTDDS=ON
                    │
                    ▼
              transport/rtps/
              Participant / UnderlayMessage / AttributesFiller
              RtpsTransmitter / RtpsDispatcher + RtpsReceiver

TopologyManager
        │
        ▼
 TopologyBackendFactory
   ├─ "local" → LocalTopologyBackend   （默认，/tmp 事件文件）
   └─ "rtps"  → 未实装 → 日志并回退 local
```

**边界**

- FastDDS 头文件与链接仅出现在 `transport/rtps/` 及 Transport 工厂的条件编译路径；不得泄漏到 `node/`、`shm/`、Python 绑定默认路径。
- Discovery（拓扑图）与数据面解耦：P0 跨机 Channel 依赖 DDS 端点匹配同名 topic；同机多进程拓扑仍走 Local 文件后端。

## 4. 阶段 A — 文档与配置真相

### 4.1 文档

更新至少下列材料，使表述与实现一致：

- `docs/source/03_Communication/` 中声称「跨主机 = RTPS」或「发现 = UDP 多播拓扑」的章节
- `autolink/examples/python/README.md` 等对 `AUTOLINK_DOMAIN_ID` 的跨机说明
- `autolink/README.md` / CHANGELOG：记录「可选 FastDDS 跨机」为新增能力，默认仍本地

表述模板：

- 默认：同进程 INTRA、同机 SHM；拓扑 = 本机文件总线。
- 跨机：编译打开 `AUTOLINK_ENABLE_FASTDDS`，conf `diff_host: RTPS`，双方设置可达的 `AUTOLINK_IP` 与相同 `AUTOLINK_DOMAIN_ID`。

### 4.2 配置与环境变量

| 项 | 行为 |
|----|------|
| `autolink.pb.conf` | 注释标明 `diff_host` 默认 SHM；`RtpsParticipantAttr`「仅 ENABLE=ON 生效」 |
| `AUTOLINK_IP` | 接线为 HostIp / RTPS unicast locator（核对 `GlobalData::InitHostInfo`） |
| `AUTOLINK_DOMAIN_ID` | 接线为 Domain id，默认 **80**；ENABLE=OFF 时可读但无 RTPS 副作用 |
| `AUTOLINK_TOPOLOGY_BACKEND` | `local`（默认）；`rtps` 本轮回退 local 并告警 |
| `setup.bash` | 注释与上表一致，删除「只改 DOMAIN 即可跨机」类误导 |

### 4.3 代码注释 / 静默 fallback

- Hybrid 中「DDS discovery…」过时注释删除或改写。
- `ENABLE=OFF` 或创建 RTPS 失败时：**AERROR + 返回 nullptr / Init 失败**，禁止再 fallback SHM 而不提示。

## 5. 阶段 B — TopologyBackendFactory

- 新增工厂：`Create(const std::string& name) → std::shared_ptr<ITopologyBackend>`。
- `TopologyManager` 只通过工厂获取后端；启动时读取 `AUTOLINK_TOPOLOGY_BACKEND`。
- `local`：现有 `LocalTopologyBackend`（协议不变：`/tmp/autolink_topology_events.log`）。
- 未知名或 `rtps`：告警并回退 `local`（本轮不实现 `RtpsTopologyBackend`）。
- 单测：工厂默认 local；非法名回退。

## 6. 阶段 C — Fast DDS 2.14 数据面

### 6.1 依赖与 CMake

- `option(AUTOLINK_ENABLE_FASTDDS "Build RTPS transport with Fast DDS 2.14" OFF)`
- ON 时：`find_package(fastdds 2.14)`；失败则 FetchContent 钉 **2.14.x**；配置阶段校验 major == 2。
- 链接 `fastdds`、`fastcdr`；定义编译宏 `AUTOLINK_ENABLE_FASTDDS=1`。
- 文档专节「升 3.x 检查清单」（Participant/QoS/CMake 包名）；本轮不编译 3.x。

### 6.2 模块（均在 `autolink/transport/rtps/`）

| 组件 | 职责 |
|------|------|
| `Participant` | 创建 `DomainParticipant`；domain / IP / lease 来自 env+conf；SIMPLE PDP/EDP；multicast `239.255.0.1` |
| `UnderlayMessage` + TypeSupport | FastCDR：`timestamp`、`seq`、`data`、`datatype`；topic 类型名 `UnderlayMessage` |
| `AttributesFiller` | `QosProfile` → DataWriter/DataReader QoS（History / Reliability / Durability） |
| `RtpsTransmitter` | 按 `channel_name` 建 Writer；序列化业务消息入 `data` |
| `RtpsDispatcher` + `RtpsReceiver` | 按 channel 建 Reader；回调还原 `MessageInfo` + payload |
| Transport 工厂 | `CreateTransmitter/Receiver(RTPS)` 在 ENABLE=ON 时返回上述类型 |

### 6.3 Topic 与 MessageInfo

- DDS topic 名 = Autolink `channel_name`。
- 业务载荷：protobuf/POD 序列化后写入 `UnderlayMessage.data`。
- **MessageInfo 编码（选定）**：优先将 `sender_id` / `spare_id` / `seq_num` 写入 DDS `WriteParams` / related sample identity（对齐 Cyber）。若 2.14 API 无法稳定承载，则在 `data` 前增加固定 **24 字节**二进制头（8+8+8），并在 Receiver 对称解析；实现以集成测试锁定一种，不得混用。

### 6.4 Hybrid

| Relation | 默认 conf | ENABLE=ON 且 `diff_host: RTPS` |
|----------|-----------|-------------------------------|
| SAME_PROC | INTRA | 不变 |
| DIFF_PROC | SHM | 不变 |
| DIFF_HOST | SHM | 使用 RtpsTransmitter/Receiver；History 补发走 RTPS |

### 6.5 错误处理

- Participant 创建失败 → Transport / Node 相关 Init 失败，日志含 IP 与 domain。
- Writer 暂无 matched reader 时不阻塞 send（与 Cyber 类似）；可选 debug 计数。

## 7. 测试计划

| 用例 | 条件 |
|------|------|
| 现有 intra / shm / hybrid 单测 | `ENABLE=OFF` 必过 |
| Backend 工厂 | 默认 local；非法/`rtps` 回退 |
| `rtps_transceiver_test` | `ENABLE=ON`；同机两进程显式 RTPS 或 Hybrid+`diff_host: RTPS` |
| OFF + conf RTPS | 明确错误，无静默 SHM |
| 手工双机 | 文档步骤：同网段、双方 `AUTOLINK_IP`、同 `DOMAIN_ID`、`diff_host: RTPS` |

## 8. 里程碑

| 里程碑 | 交付 |
|--------|------|
| **M1 = A** | 文档 + conf + env 注释 + 去掉静默 RTPS→SHM |
| **M2 = B** | `TopologyBackendFactory` + env；默认行为不变 |
| **M3 = C** | `transport/rtps/*` + CMake 开关 + Hybrid 接线 + 单测 + CHANGELOG |

## 9. 后续子项目（本 spec 外）

| 子项目 | 内容 |
|--------|------|
| **D** | `RtpsTopologyBackend`（ChangeMsg 走 RTPS）；可选双 Participant（topology / transport 分端口） |
| **E** | Discovery Server、大消息策略、QoS 产品化表、诊断指标、安全 |

## 10. 风险与缓解

| 风险 | 缓解 |
|------|------|
| Homebrew / 系统包与 2.14 ABI 漂移 | 优先 FetchContent 钉版本；CI 固定容器镜像 |
| MessageInfo 与 Cyber 不完全兼容 | 明确编码策略 + 单测；不承诺与旧 Cyber 1.5 互通 |
| P0 无 RTPS 拓扑导致「半残」误解 | 文档强调：跨机 Channel 靠 DDS 发现；完整图/Service 跨机待 D |
| Mac 多播受限 | 文档标明以 Linux 同网段为首要验证平台；Mac 作尽力编译 |

## 11. 参考

- Apollo Cyber：`cyber/transport/rtps/`、`hybrid_*`、`CYBER_IP` / `CYBER_DOMAIN_ID`（本机树仍偏 Fast-RTPS 1.5 API）
- Autolink 现状：`transport/transport.hpp`、`service_discovery/topology_backend.*`、`proto/transport_conf.proto`
- Fast DDS 2.14 LTS 官方文档（DomainParticipant / DataWriter / DataReader / QoS）
