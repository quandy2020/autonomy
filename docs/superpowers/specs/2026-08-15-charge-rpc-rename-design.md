# ChargeService 命名 — 自动回充对外 RPC

范围：`automsgs/proto/rpcs/docking.proto` → `charge.proto`；同步 `system.proto` 能力/目标枚举、rpcs README、cli README。

不改：`autonomy/bridge` / `task/apps/charging` 内部 `DockCommand`（由 bridge 映射）。

## 1. 动机

产品名「自动回充」；原 `DockingService` + `Dock`/`Undock` 偏机械对接，与 Navigation/Follow 的任务语义不一致。

## 2. 命名表

| 中文 | 英文 |
|------|------|
| 自动回充（域） | `ChargeService` / package `automsgs.rpcs.charge` |
| 回充 | `Return` |
| 离桩 | `Leave` |
| 取消 / 查状态 | `Cancel` / `GetStatus` |
| 充电站 ID | `station_id` |
| 过程反馈 | `ChargeFeedback` |
| 粗状态 | `ChargeState`（取值语义同原 `DockingState`） |

## 3. 服务面

```protobuf
package automsgs.rpcs.charge;

service ChargeService {
  rpc Return(ReturnRequest) returns (stream ChargeFeedback);
  rpc Leave(LeaveRequest) returns (stream ChargeFeedback);
  rpc Cancel(CancelRequest) returns (CancelResponse);
  rpc GetStatus(GetStatusRequest) returns (GetStatusResponse);
}
```

流式语义不变：`Return`/`Leave` server-stream；终态关流；不以充满电关流。

## 4. 状态枚举（改前缀，不改序）

| 原 `DockingState` | 新 `ChargeState` |
|-------------------|------------------|
| `DOCKING_STATE_UNKNOWN` | `CHARGE_STATE_UNKNOWN` |
| `IDLE` | `IDLE` |
| `DOCKING` | `RETURNING`（回充过程中） |
| `UNDOCKING` | `LEAVING` |
| `DOCKED_NOT_CHARGING` | `DOCKED_NOT_CHARGING` |
| `CHARGING` | `CHARGING` |
| `FULL` | `FULL` |
| `FAILED` / `CANCELLED` | 同名 |

## 5. 关联改动

| 位置 | 变更 |
|------|------|
| 删除 `docking.proto`，新增 `charge.proto` | 全文更名 |
| `system.GoalKind` | `GOAL_KIND_DOCKING` → `GOAL_KIND_CHARGE` |
| `system.Capabilities` | `supports_docking` → `supports_charge` |
| `common.Code` 注释 | charging/docking → charge（码段 500 保留） |
| docs / cli | `DockingService` → `ChargeService` |

## 6. Bridge 映射

| Charge RPC | bridge `DockCommand` |
|------------|----------------------|
| `Return` | `DOCK_CMD_START` |
| `Leave` | `DOCK_CMD_UNDOCK` |
| `Cancel` | `DOCK_CMD_CANCEL` |

`station_id` ↔ `dock_station_id`。

## 7. 非目标

- 不改 bridge / task proto 对外历史名
- 不恢复 `DockingPhase`
- 不加 ListStations / target SoC
