# Autolink FastDDS 拓扑与运维设计（D+E）

- **日期**：2026-09-07
- **状态**：已批准（实现计划已就绪）
- **计划**：`docs/superpowers/plans/2026-09-07-autolink-fastdds-topology-ops.md`
- **前置**：`docs/superpowers/specs/2026-09-06-autolink-fastdds-multimachine-design.md`（M1–M3 已实现）
- **范围**：跨机拓扑后端（D）+ Discovery Server / 大消息软限 / 诊断 / 3.x 文档（E）
- **不做**：DDS Security；默认切 FastDDS 3.x；元数据+外带大对象通道

## 1. 背景与目标

M1–M3 已提供可选 RTPS **数据面**与本机文件拓扑。跨机仍看不到完整 node/channel/service 图，禁多播场景缺少 Discovery Server，大包与可观测性不足。

本设计在保持 `local` 默认的前提下，补齐 **RTPS 拓扑发现**与运维能力，使「两机同域」可同时完成拓扑互通与 Channel 通信。

**成功标准**

1. `AUTOLINK_ENABLE_FASTDDS=ON` 且 `AUTOLINK_TOPOLOGY_BACKEND=rtps`：两机（或两进程）拓扑 ChangeMsg 互通，远端 channel 可见。
2. 未设 Discovery Server 时行为与现网一致（SIMPLE）；设置 `AUTOLINK_DISCOVERY_SERVER` 后可作为 CLIENT 发现。
3. RTPS 发送超过可配阈值时默认 `AWARN` 仍发送；`AUTOLINK_RTPS_REJECT_OVERSIZE=1` 时拒绝。
4. 提供基础 RTPS 计数（matched / sent / recv / fail / oversize），至少可日志 Dump。
5. `BACKEND=local` + FastDDS OFF 回归不受影响。
6. 文档明确 Security 未实现；含 FastDDS 3.x 升级清单。

## 2. 决策摘要

| 项 | 选择 |
|----|------|
| 架构 | 双 Participant（topology / transport）+ `RtpsTopologyBackend` |
| 拓扑默认 | 仍为 `local`；显式 `rtps` 才启用 |
| Discovery Server | 可选；env 非空则 CLIENT |
| 大消息 | 软限 + 告警；可选硬拒绝 |
| Security | 本轮不做 |
| FastDDS 版本 | 默认仍 2.14；3.x 仅文档 + CMake 提示 |

## 3. 架构

```text
TopologyManager
  └─ TopologyBackendFactory
        ├─ local → LocalTopologyBackend
        └─ rtps  → RtpsTopologyBackend  （需 FastDDS）
                      │
                      ▼
              RtpsParticipantHub
                ├─ TopologyParticipant  (name …:topology)
                └─ TransportParticipant (name …:transport)
                     ▲
Transport / Hybrid ──┘
```

- 两 Participant 共用 `AUTOLINK_DOMAIN_ID`、`AUTOLINK_IP`、可选 Discovery Server 列表。
- 拓扑 Topics（沿用 Manager 已有名）：
  - `node_change_broadcast`
  - `channel_change_broadcast`
  - `service_change_broadcast`
- 拓扑 QoS：RELIABLE + TRANSIENT_LOCAL（对齐 Cyber TOPO_CHANGE）。
- 业务 Channel 仍走 Transport Participant + 既有 RtpsTransmitter/Receiver。

## 4. 阶段 D — RtpsTopologyBackend

### 4.1 RtpsParticipantHub

- 单例或由 Init 拥有：懒创建 / 显式 `InitFromConfig()`。
- API：`TopologyParticipant()`、`TransportParticipant()`、`Shutdown()`。
- Transport 改为从 Hub 取 transport Participant（替换「仅 Transport 内创建一个」）；拓扑后端从 Hub 取 topology Participant。
- Init 失败：对应路径 AERROR；`BACKEND=rtps` 时 factory 可 fallback `local` 并告警（或 Init 失败——**选定：fallback local + AERROR**，保证进程可起）。

### 4.2 RtpsTopologyBackend

实现 `ITopologyBackend`：

| 方法 | 行为 |
|------|------|
| `Start` | 确保 Hub topology Participant；为三类 topic 创建 Writer/Reader |
| `Publish` | `ChangeMsg` 序列化 → Underlay（`datatype=ChangeMsg`）；可无业务 24B MessageInfo 前缀 |
| `Subscribe` | Reader 回调 → 过滤本机回环（host_name+process_id）→ 用户 callback |
| `Shutdown` | 释放拓扑 Reader/Writer；不关闭 transport Participant |

### 4.3 Factory

- `Create("rtps")`：FastDDS ON → `RtpsTopologyBackend`；OFF → AERROR + `LocalTopologyBackend`。
- `Create("local")` / 未知：保持现行为。

## 5. 阶段 E — 运维与增强

### 5.1 Discovery Server

- Env：`AUTOLINK_DISCOVERY_SERVER=ip:port[,ip:port]`。
- 非空：两 Participant 的 discovery 使用 CLIENT，server 列表来自该 env。
- 空：保持 SIMPLE 多播（现状）。
- 文档说明如何用 Fast DDS `fast-discovery-server`（或等价）启动 Server；本仓库可不自研 server 进程。

### 5.2 大消息软限

- 默认阈值：`4 * 1024 * 1024` bytes（payload，含 Underlay data）。
- Env：`AUTOLINK_RTPS_MAX_PAYLOAD_BYTES`；`0` 表示不检查。
- Env：`AUTOLINK_RTPS_REJECT_OVERSIZE=1` → 超限不发送并计 `oversize` + return false；默认仅 `AWARN` 仍发送。
- 作用于 `RtpsTransmitter`；拓扑 Publish 建议共用同一检查（ChangeMsg 通常很小）。

### 5.3 诊断

- `RtpsStats`（原子计数）：`matched_readers`、`matched_writers`、`sent`、`recv`、`write_fail`、`oversize`。
- `Dump()` → 字符串；周期性 `AINFO`（可选，默认仅显式 Dump）或 monitor/cli 钩子。
- DataWriter/DataReader listener 的 match 回调更新 matched。

### 5.4 FastDDS 3.x

- 文档清单：CMake 目标名 `fastdds` vs `fastrtps`、QoS/API 差异、建议验证矩阵。
- CMake：若 `find_package` 到 major≥3，`WARNING` 提示「默认支持 2.14；3.x 未验证」。

## 6. 测试计划

| 用例 | 条件 |
|------|------|
| factory rtps ON → RtpsTopologyBackend | 单测 |
| factory rtps OFF → fallback local | 单测 |
| 双节点拓扑 Join 可见远端 channel | ON 集成（双线程/双进程） |
| Channel pub/sub + BACKEND=rtps | 冒烟 |
| oversize warn / reject | 单测 |
| Discovery Server | 文档手工步骤；CI 可选 |
| OFF + local 回归 | topology_backend_factory + 既有 transport 测 |

## 7. 里程碑

| 里程碑 | 交付 |
|--------|------|
| **M4 = D** | Hub、双 Participant、RtpsTopologyBackend、factory、拓扑测试、文档更新 |
| **M5 = E** | Discovery Server 配置、软限、RtpsStats、3.x 提示与文档 |

## 8. 风险与缓解

| 风险 | 缓解 |
|------|------|
| 双 Participant 资源/端口 | 独立 name；同 domain；文档说明 |
| 拓扑与数据面启动顺序 | Hub 统一 Init；Backend Start 依赖 Hub |
| 同进程 intraprocess | 沿用 M3 Underlay 二进制序列化修复 |
| Discovery Server 运维负担 | 默认 SIMPLE；Server 可选 |

## 9. 参考

- 前置 spec：2026-09-06 FastDDS multimachine（A→B→C）
- Apollo Cyber：topology 与 transport 分 Participant；`*_change_broadcast`
- 现码：`TopologyBackendFactory`、`LocalTopologyBackend`、`transport/rtps/*`
