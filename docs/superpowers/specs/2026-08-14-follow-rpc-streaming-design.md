# FollowService 流式 RPC 设计 — 泛化目标 + server streaming

日期：2026-08-14  
状态：已批准  
范围：`automsgs/proto/rpcs/follow.proto`、`automsgs/proto/rpcs/README.md`  
相关：`2026-08-14-docking-rpc-streaming-design.md`（混合 unary/stream 模式）、`2026-08-14-automsgs-rpcs-overview-design.md`

## 1. 背景与目标

当前 `FollowService` 仅 unary `StartFollow`，目标语义偏「跟人」，进度靠轮询，与已升级的 `DockingService` 流式形态不一致。

**目标：**

- 泛化目标：`FollowTargetType` + `target_id` + 可选 `hint_pose`。
- **执行 = server streaming（单端流）**；**Cancel / GetStatus = unary**。
- 会话期开流；`LOST_TARGET` 不关流；注释完善、Google protobuf 风格、言简意赅。

**非目标：** 双端流、`Retarget`、多目标并行、语音唤醒跟随、改 `CODE_FOLLOW_*` 数值。

## 2. 已确认决策

| 项 | 选择 |
|---|---|
| 目标模型 | **C**：`target_type` + `target_id` + 可选 `hint_pose` |
| 关流 | **A**：会话流；Cancel / FAILED / 正常结束才关 |
| `LOST_TARGET` | **A**：保持开流，可重锁定 |
| API 形态 | **方案 1**：`Follow` stream + Cancel/GetStatus unary；无 Retarget |

## 3. 服务面

```text
service FollowService {
  rpc Follow(FollowRequest) returns (stream FollowFeedback);
  rpc Cancel(CancelRequest) returns (CancelResponse);
  rpc GetStatus(GetStatusRequest) returns (GetStatusResponse);
}
```

### 3.1 流式约定

1. 无双端流。取消走 unary `Cancel(goal_id)`（空 = 当前活动目标）。
2. 首条 Feedback 尽快含最终 `goal_id`；拒绝则非 OK `status` 并关流。
3. **关流：** Cancel 成功后的末条、`FAILED`、会话正常结束（`COMPLETED`）。  
   **`LOST_TARGET` 不关流**；实现可进入 `REACQUIRING` 再回到 `TRACKING`。
4. 全局最多一个活动跟随；忙时首条 `CODE_FOLLOW_BUSY`（902）并关流。
5. `GetStatus` 可与 stream 并行。

### 3.2 命名与兼容

- RPC：`Follow`（取代 `StartFollow`）。
- 删除 `StartFollowRequest` / `StartFollowResponse`（breaking；字段迁入 `FollowRequest`）。
- 保留 package `automsgs.rpcs.follow`、文件名 `follow.proto`。

## 4. 消息与枚举

### 4.1 FollowTargetType

| 值 | 含义 |
|---|---|
| `FOLLOW_TARGET_TYPE_UNKNOWN` | 未指定；实现默认策略 |
| `FOLLOW_TARGET_TYPE_PERSON` | 人体 |
| `FOLLOW_TARGET_TYPE_OBJECT` | 一般物体 |
| `FOLLOW_TARGET_TYPE_VEHICLE` | 车辆 / 移动平台 |
| `FOLLOW_TARGET_TYPE_CUSTOM` | 自定义（语义由 `target_id` 约定） |

### 4.2 FollowRequest

| 字段 | 类型 | 说明 |
|---|---|---|
| `header` | `Header` | 标准头 |
| `target_type` | `FollowTargetType` | 目标类型 |
| `target_id` | `string` | 空 = 实现自选（如最近 PERSON） |
| `hint_pose` | `Pose` | 可选；辅助锁定 |
| `goal_id` | `string` | 可选客户端 id |
| `desired_distance_m` | `float` | 可选跟距（米）；`0` = 默认 |

### 4.3 FollowState

| 值 | 含义 |
|---|---|
| `UNKNOWN` | 未设置 |
| `IDLE` | 空闲 |
| `ACQUIRING` | 获取 / 锁定目标中 |
| `FOLLOWING` | 跟随中 |
| `LOST_TARGET` | 目标丢失（流保持） |
| `FAILED` | 失败（关流） |
| `CANCELLED` | 已取消（关流） |

`active == true` 当 `state ∈ {ACQUIRING, FOLLOWING, LOST_TARGET}`。

### 4.4 FollowPhase

| 值 | 含义 |
|---|---|
| `UNKNOWN` | 未设置 |
| `IDLE` | 无过程 |
| `SEARCHING` | 搜索目标 |
| `LOCKING` | 锁定中 |
| `TRACKING` | 稳定跟踪跟随 |
| `REACQUIRING` | 丢失后重获 |
| `COMPLETED` | 会话正常结束（末条） |
| `FAILED` / `CANCELLED` | 失败 / 取消 |

无 `progress`（0..1）：跟随为会话型，用 `phase` + `distance_m` 即可。

### 4.5 FollowFeedback

| 字段 | 类型 | 说明 |
|---|---|---|
| `status` | `Status` | 接受 / 本条 / 终态 |
| `state` | `FollowState` | 粗状态 |
| `phase` | `FollowPhase` | 细阶段 |
| `goal_id` | `string` | |
| `target_type` | `FollowTargetType` | |
| `target_id` | `string` | 解析后的目标 id |
| `distance_m` | `float` | 到目标距离（米）；未知可为 0 |
| `active` | `bool` | |
| `message` | `string` | 可选说明 |

### 4.6 GetStatusResponse

与 Feedback 对齐的快照字段（含 `phase`、`distance_m`、`target_type`）。

### 4.7 Cancel

与现网一致：`CancelRequest.goal_id`（空 = 当前）；`CancelResponse.status`。  
`Cancel` 成功 → `CODE_OK`；流末条 `state`/`phase` = `CANCELLED`，Feedback.`status` = `CODE_FOLLOW_CANCELLED`（903）。

## 5. 错误码

沿用 `common.proto`：

- `CODE_FOLLOW_TARGET_NOT_FOUND` = 900  
- `CODE_FOLLOW_FAILED` = 901  
- `CODE_FOLLOW_BUSY` = 902  
- `CODE_FOLLOW_CANCELLED` = 903  

**不改动** Code 数值。

## 6. 注释与风格

- Google Style：PascalCase 类型/RPC；`snake_case` 字段；枚举 `FOLLOW_*` 前缀 + `_UNKNOWN` 零值。
- 英文短注释：service 流式语义、空 `target_id`、`hint_pose`、关流与 `LOST_TARGET`。
- 通用缩写保留（`id`、`RPC`）；不造难懂缩写。

## 7. 改动文件

| 文件 | 变更 |
|---|---|
| `automsgs/proto/rpcs/follow.proto` | 按 §3–§4 重写 |
| `automsgs/proto/rpcs/README.md` | Follow 流式例外（对齐 Docking 小节风格） |
| `common.proto` | 不改 |

## 8. 验收

- [ ] `Follow` = `returns (stream FollowFeedback)`；Cancel/GetStatus unary；无 bidi  
- [ ] `FollowTargetType` + 可选 `hint_pose`；`FollowPhase`；无 `progress`  
- [ ] 文档写明：会话流、`LOST_TARGET` 不关流  
- [ ] 无 `StartFollow*`；注释齐全  
- [ ] 无 Retarget / ListTargets 等非目标内容
