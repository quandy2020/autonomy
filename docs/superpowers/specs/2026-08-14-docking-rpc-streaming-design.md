# DockingService 流式 RPC 设计 — server streaming + unary

日期：2026-08-14  
状态：待审阅  
范围：`automsgs/proto/rpcs/docking.proto`、`automsgs/proto/rpcs/README.md`  
前置：`2026-08-14-charging-rpc-nav-align-design.md`（异步 goal / Cancel / GetStatus）  
相关：`2026-08-14-automsgs-rpcs-overview-design.md`（`DockingService` 归属）

## 1. 背景与目标

当前 `DockingService` 的 `Dock` / `Undock` 为 unary，进度依赖轮询 `GetStatus`，对第三方自动回充编排不够专业，且未在 API 面区分 unary 与 streaming。

**目标：**

- 明确标注并实现：**执行类 = server streaming（单端流）**，**取消/查询 = unary**。
- Feedback 带细粒度 `phase` + `progress`，粗状态仍用现有 `DockingState`。
- 流在**对接/离桩动作终态**关闭，不长期挂在充电过程上。

**非目标：**

- 双端流（bidi）
- ListDocks、目标 SoC、强制回充策略
- 将 Nav / Follow 一并改为 streaming（可后续同构）
- 改动 `CODE_CHARGING_*` 数值

## 2. 已确认决策

| 项 | 选择 |
|---|---|
| 流形态 | **混合**：Dock/Undock = server streaming；Cancel/GetStatus = unary |
| Feedback 内容 | **方案 B**：快照字段 + `phase` + `progress` |
| 关流时机 | **A**：对接/离桩动作完成即关流；充电驻留用 `GetStatus` |
| API 形态 | **方案 1**：共享 `DockingFeedback`；不拆独立 Result RPC |
| 取消 | 独立 unary `Cancel`；不在流上写客户端消息 |

## 3. 服务面

```text
service DockingService {
  rpc Dock(DockRequest) returns (stream DockingFeedback);
  rpc Undock(UndockRequest) returns (stream DockingFeedback);
  rpc Cancel(CancelRequest) returns (CancelResponse);
  rpc GetStatus(GetStatusRequest) returns (GetStatusResponse);
}
```

### 3.1 流式约定

1. **无双端流。** 中途取消走 `Cancel(goal_id)`（空 = 当前活动目标）。
2. **首条 Feedback** 尽快发出，含最终 `goal_id`。若拒绝（BUSY / NOT_READY / 参数错误等），首条即带非 OK `status` 并关流。
3. **关流条件：** 进入对接/离桩终态后发末条再关流：
   - Dock 成功侧：`DOCKED_NOT_CHARGING` / `CHARGING` / `FULL`
   - Undock 成功侧：`IDLE`
   - 失败/取消：`FAILED` / `CANCELLED`
4. 充电过程（电量爬升）**不**占用该 stream；客户端用 `GetStatus` 观察 `battery_pct` / `CHARGING` / `FULL`。
5. **并发：** 全局最多一个活动 Dock/Undock；忙时新请求首条 Feedback `CODE_CHARGING_BUSY`（504）并关流。
6. `GetStatus` 可与进行中的 stream 并行，用于断线重连后补快照。

### 3.2 Request / Cancel（不变）

**DockRequest：** `header`, `dock_id`（空 = 默认桩）, `goal_id`（可选）  
**UndockRequest：** `header`, `goal_id`（可选）  
**CancelRequest / CancelResponse：** 与现网一致  

删除 unary 时代的 `DockResponse` / `UndockResponse`（breaking）。

## 4. 状态与 Feedback

### 4.1 DockingState（保留 0–8）

| 枚举值 | 编号 | 含义 |
|---|---|---|
| `DOCKING_STATE_UNSPECIFIED` | 0 | 未设置 |
| `DOCKING_STATE_IDLE` | 1 | 空闲、未在桩 |
| `DOCKING_STATE_DOCKING` | 2 | 回桩进行中 |
| `DOCKING_STATE_UNDOCKING` | 3 | 离桩进行中 |
| `DOCKING_STATE_DOCKED_NOT_CHARGING` | 4 | 已对接未充电 |
| `DOCKING_STATE_CHARGING` | 5 | 充电中 |
| `DOCKING_STATE_FULL` | 6 | 已充满 |
| `DOCKING_STATE_FAILED` | 7 | 最近一次失败 |
| `DOCKING_STATE_CANCELLED` | 8 | 最近一次取消 |

`active == true` 当且仅当 `state ∈ {DOCKING, UNDOCKING}`（实现约定）。

### 4.2 DockingPhase（新增）

| 枚举值 | 含义 |
|---|---|
| `DOCKING_PHASE_UNSPECIFIED` | 未设置 |
| `DOCKING_PHASE_IDLE` | 无过程 |
| `DOCKING_PHASE_APPROACHING` | 接近桩位 |
| `DOCKING_PHASE_ALIGNING` | 精对齐 |
| `DOCKING_PHASE_CONTACTING` | 触点 / 机械对接 |
| `DOCKING_PHASE_CHARGING_SETUP` | 建立充电（Dock 末段） |
| `DOCKING_PHASE_UNDOCKING` | 离桩过程 |
| `DOCKING_PHASE_COMPLETED` | 本目标过程结束（末条常用） |
| `DOCKING_PHASE_FAILED` | 过程失败 |
| `DOCKING_PHASE_CANCELLED` | 过程取消 |

Undock 流主要使用 `UNDOCKING` → `COMPLETED`（或 FAILED/CANCELLED）；Dock 使用 APPROACHING → … → CHARGING_SETUP → COMPLETED。

### 4.3 DockingFeedback（Dock / Undock 共用）

| 字段 | 类型 | 说明 |
|---|---|---|
| `status` | `Status` | 接受/本条/终态是否成功 |
| `state` | `DockingState` | 粗状态 |
| `phase` | `DockingPhase` | 细阶段 |
| `progress` | `float` | 0..1；未知可为 0 |
| `goal_id` | `string` | |
| `dock_id` | `string` | |
| `battery_pct` | `float` | 0..100 |
| `active` | `bool` | 是否过程中 |
| `message` | `string` | 可选说明 |

### 4.4 GetStatusResponse

在现有字段上增加 `phase`、`progress`，与 Feedback 对齐。

## 5. 错误码

沿用 `common.proto` 中 `CODE_CHARGING_*`（500–507）。流上拒绝/失败写在 Feedback.`status`；`Cancel` / `GetStatus` 写在各自 Response.`status`。

约定：`Cancel` RPC 成功 → `CODE_OK`；对应 stream 末条 `state=CANCELLED`、`phase=CANCELLED`，且 Feedback.`status.code=CODE_CHARGING_CANCELLED`（505）。客户端判断取消以 `state`/`phase` 为准，`status.code` 作辅助。

## 6. 改动文件

| 文件 | 变更 |
|---|---|
| `automsgs/proto/rpcs/docking.proto` | Dock/Undock → stream；新增 Phase + Feedback；GetStatus 对齐 |
| `automsgs/proto/rpcs/README.md` | 标明 unary vs server-stream；关流语义 |
| `common.proto` | **不改** 错误码数值 |

## 7. 兼容性

Breaking：`Dock` / `Undock` 返回类型由 unary response 改为 `stream DockingFeedback`。第三方需按 streaming 客户端改写；轮询-only 客户端可继续只用 `GetStatus`（需另有触发 Dock 的路径，或改用 stream 并忽略中间帧）。

## 8. 验收

- [ ] `Dock` / `Undock` 为 `returns (stream DockingFeedback)`
- [ ] `Cancel` / `GetStatus` 为 unary
- [ ] Feedback 含 `phase` + `progress`；`GetStatus` 含同名字段
- [ ] README 写明：单端流 vs unary、关流=对接/离桩终态（非充至满）
- [ ] 无 bidi；忙时首条 BUSY 并关流
- [ ] 无 ListDocks / SoC 策略等非目标内容
