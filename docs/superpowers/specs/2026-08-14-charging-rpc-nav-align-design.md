# 自动回充 RPC（ChargeService）完备性设计 — 对齐 Nav

日期：2026-08-14  
状态：待审阅  
范围：`automsgs/proto/rpcs/charging.proto`、`status.proto`（500–599）、`rpcs/README.md`  
对齐：`navigation.proto`（`GoTo` / `Cancel` / `GetStatus` 异步模型）

## 1. 背景与目标

当前 `ChargeService` 仅有 `Dock` / `Undock` / `GetStatus`，状态枚举偏「驻留」，缺少过程态、`goal_id` 与取消，第三方难以做可靠的自动回充编排。

**目标（层 A）：** 与 `NavService` 同级完备——异步接受目标、单一活动 `goal_id`、`Cancel`、更细状态与错误码。

**非目标（层 B/C）：** `ListDocks`、目标 SoC、强制回充、暂停/恢复充电、streaming feedback。

## 2. 已确认决策

| 项 | 选择 |
|---|---|
| 完备层级 | **A**（对齐 Nav） |
| Dock/Undock 语义 | 异步：RPC 只表示接受目标，立刻返回 `goal_id`；进度靠 `GetStatus` |
| 取消与并发 | 单一活动目标；`Cancel(goal_id)`（空=取消当前）；忙时拒绝新请求（不抢占） |
| 状态模型 | **方案 1**：单一 `ChargingState`（过程 + 驻留 + 结果） |
| 枚举编号 | 重新排布 `0..8`（breaking；本目录 API 尚早） |

## 3. 服务面

```text
service ChargeService {
  rpc Dock(DockRequest) returns (DockResponse);
  rpc Undock(UndockRequest) returns (UndockResponse);
  rpc Cancel(CancelRequest) returns (CancelResponse);
  rpc GetStatus(GetStatusRequest) returns (GetStatusResponse);
}
```

### 3.1 消息

**DockRequest：** `header`, `dock_id`（空=默认桩）, `goal_id`（可选，客户端自带）  
**DockResponse：** `status`, `goal_id`（服务端最终 id）

**UndockRequest：** `header`, `goal_id`（可选）  
**UndockResponse：** `status`, `goal_id`

**CancelRequest：** `goal_id`（空=取消当前活动目标）  
**CancelResponse：** `status`

**GetStatusRequest：** 空

**GetStatusResponse：**

| 字段 | 类型 | 说明 |
|---|---|---|
| `status` | `Status` | 查询本身是否成功 |
| `state` | `ChargingState` | 见 §4 |
| `goal_id` | `string` | 当前或最近活动目标；无则空 |
| `dock_id` | `string` | 目标/当前桩；空=默认或未知 |
| `battery_pct` | `float` | 0..100 |
| `active` | `bool` | 是否有进行中的 Dock/Undock |
| `message` | `string` | 可选说明（失败原因等） |

### 3.2 并发规则

1. 任一时刻最多一个活动 Dock/Undock goal。  
2. 已有活动目标时再 `Dock`/`Undock` → `CODE_CHARGING_BUSY`（504）。  
3. `Cancel("")` 取消当前；未知 id → `CODE_NOT_FOUND`。  
4. `Cancel` 成功 → `CODE_OK`；状态随后可为 `CANCELLED`（再过渡到 `IDLE` 或仍 `DOCKED_*`）。

## 4. ChargingState

| 枚举值 | 编号 | 含义 |
|---|---|---|
| `CHARGING_STATE_UNSPECIFIED` | 0 | 未设置 |
| `CHARGING_STATE_IDLE` | 1 | 空闲、未在桩（替换原 `NOT_DOCKED`） |
| `CHARGING_STATE_DOCKING` | 2 | 回桩进行中（含接近/对接） |
| `CHARGING_STATE_UNDOCKING` | 3 | 离桩进行中 |
| `CHARGING_STATE_DOCKED_NOT_CHARGING` | 4 | 已对接未充电 |
| `CHARGING_STATE_CHARGING` | 5 | 充电中 |
| `CHARGING_STATE_FULL` | 6 | 已充满（可仍在桩） |
| `CHARGING_STATE_FAILED` | 7 | 最近一次 Dock/Undock 失败 |
| `CHARGING_STATE_CANCELLED` | 8 | 最近一次被取消 |

**典型流转：**

```text
IDLE → Dock → DOCKING → DOCKED_NOT_CHARGING | CHARGING | FULL
DOCKED_* / CHARGING / FULL → Undock → UNDOCKING → IDLE
活动中 → Cancel → CANCELLED → IDLE 或仍 DOCKED_*
DOCKING | UNDOCKING 失败 → FAILED
```

`active == true` 当且仅当 `state ∈ {DOCKING, UNDOCKING}`（实现约定；客户端可以 `active` 为准）。

## 5. 错误码（500–599）

保留：

- `CODE_CHARGING_DOCK_NOT_FOUND` = 500  
- `CODE_CHARGING_DOCK_FAILED` = 501  
- `CODE_CHARGING_ALREADY_DOCKED` = 502  
- `CODE_CHARGING_ALREADY_UNDOCKED` = 503  

新增：

- `CODE_CHARGING_BUSY` = 504 — 已有活动 goal  
- `CODE_CHARGING_CANCELLED` = 505 — 目标被取消（可选用于历史结果/查询旁路；`Cancel` RPC 成功仍用 `CODE_OK`）  
- `CODE_CHARGING_TIMEOUT` = 506 — 对接/离桩超时  
- `CODE_CHARGING_NOT_READY` = 507 — 定位/地图/底盘未就绪  

复用通用：`INVALID_ARGUMENT`、`NOT_FOUND`、`DEADLINE_EXCEEDED`、`ABORTED`（仅当语义更贴切时；忙时优先 504）。

## 6. 改动文件

| 文件 | 变更 |
|---|---|
| `automsgs/proto/rpcs/charging.proto` | 按 §3–§4 重写 |
| `automsgs/proto/rpcs/status.proto` | 追加 504–507 |
| `automsgs/proto/rpcs/README.md` | 更新 charging 行与错误码示例 |

## 7. 兼容性

- `ChargingState` 编号 breaking（删除 `NOT_DOCKED` 名，重排 1–8）。  
- 新增 RPC `Cancel`、新字段对 proto3 客户端为可忽略扩展；旧客户端若依赖旧枚举数值需同步升级。

## 8. 验收

- [ ] `charging.proto` 含 `Dock`/`Undock`/`Cancel`/`GetStatus` 与完整状态枚举  
- [ ] `status.proto` 含 504–507  
- [ ] `rpcs/README.md` 与枚举/错误码一致  
- [ ] 无 ListDocks / SoC 策略等 B/C 内容混入  
