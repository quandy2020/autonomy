# bridge/grpc 设计备忘

权威短文：读代码前先看本页。协议细节见 `docs/source/15_Bridge/`。

## 1. 四层

```text
Handler  →  Context / DomainBundle  →  Stub  →  Channel | Action | Cache
 薄            所有权 · Cancel · infra         域语义              机制
```

| 层 | 做什么 | 不做什么 |
|----|--------|----------|
| **Handler** | gRPC 进出、`BRIDGE_*` 宏、特例手写 | 写 topic、持有业务状态 |
| **Context / DomainBundle** | Muxer(Shared) · Hub/Idempotency(Unique) · DomainBundle(值) · Stub(Unique) · CancelAll | RPC 字段解析 |
| **Stub** | 域命令、互斥、session、组帧 | 直接碰 `async_grpc` 类型 |
| **机制** | GoalChannel / Action CRTP / SampleCache / ProtoPool | 某一 RPC 的特例分支 |

对外表面 **仅** `automsgs.rpcs.*`。对内 Task 通信 **仅** `automsgs/task/*.pb.h` + `bridge/constants.hpp` 通道名；**禁止** include / 链接 `autonomy/task`。

## 2. 复用栈内设施

| 设施 | 用法 |
|------|------|
| `autolink::base::ThreadPool` | `WorkScheduler` 后台 Action / Record |
| `autolink::base::ObjectPool` | `ProtoPool<MessageT>`（Sensor Record 帧） |
| `autonomy::common::Factory` | `CancelRegistry` 按 id 注册 CancelAction |
| `autonomy::common::function_traits` | `DispatchCommands` 判定 handler 返回类型 |

## 3. 不变量

1. **互斥**：同一时刻至多一个「命令任务」占 `TaskMuxer` 槽；estop 时 `TryAcquire` 失败。
2. **ACK 优先**：阻塞路径必须先经 Session / GoalChannel 发出非终态帧，再 `WorkScheduler::Schedule`。
3. **Cancel**：`DomainBundle::CancelAll` → Factory 产物 `CancelAction::Run`；不碰 gRPC writer。
4. **状态**：对外一律 `OkStatus` / `ErrorStatus`。
5. **线程**：Handler 在 event 线程；阻塞只进 worker。
6. **命令 GoalChannel**：一律 `BRIDGE_CHANNEL_TRAITS*` + `GoalChannelCommandStub`；写 `/autonomy/task/<domain>/{goal,feedback}`（常量在 `bridge/constants.hpp`）；编排由对端 Task 进程完成，Bridge 不聚合域 Stub、不依赖 `autonomy/task` 代码。
7. **宏**：命令域只用 `BRIDGE_CHANNEL_TRAITS*`（对齐 `follow_stub`）。门面（Sensor / SystemMonitor / MapService）为普通 C++，不造域内宏。

## 4. 失败路径

| 场景 | 做法 |
|------|------|
| 无 Context | `ErrorStatus(UNKNOWN,"no context")` 或 `Finish(INTERNAL)` |
| Estop | `RejectOnEstop` |
| 槽位占用 | `RejectOnBusy` |
| 池不可用 | Session reject：`"work scheduler unavailable"` |

## 5. 命名约定

- **文件名 = 主类型**：`ChargeStub` → `charge_stub.hpp`；机制同理（`GoalChannelStub` → `goal_channel_stub.hpp`）。
- **见名知意优先**：不强制 1–2 个单词；函数用完整动词短语亦可（如 `StartMonitorIfNeeded`、`RejectOnEstop`）。
- 谓词：`Is*` / `Has*`；宏：`BRIDGE_*`。

## 6. 最小测试清单

| 组件 | 文件 | 要点 |
|------|------|------|
| `TaskMuxer` / Idempotency | `test/unit/grpc/task_muxer_test.cpp` · `idempotency_test.cpp` | acquire / estop / release / duplicate cmd_id |
| `WorkScheduler` + Session | `test/unit/grpc/work_scheduler_test.cpp` · `session_gate_test.cpp` | Schedule；accept→execute→release |
| `DispatchCommands` | `test/unit/grpc/command_dispatch_test.cpp` | match / bool fail / RejectOn* |
| `CancelRegistry` | `test/unit/grpc/cancel_registry_test.cpp` | 命名 Factory hook / 重复 id |
| `OkStatus` / `ErrorStatus` | `test/unit/grpc/rpc_status_test.cpp` | 应用层 Status 构造 |

## 7. 平台能力（policy / tools）

机载 gRPC **横切能力**（拦截器、Health、Reflection、元数据、鉴权、限流、TLS、OTel）见：

- 文档索引：[`../docs/README.md`](../docs/README.md)
- 源码：`../policy/`、`../tools/`
- 测试：`../test/`（见 [`../docs/08_testing.md`](../docs/08_testing.md)）

业务四层不变量不变；平台在 `tools::ApplyPlatform` 于 Server Build 前装配。

