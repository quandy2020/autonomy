# ExplorationService RPC 设计（方案 B）

范围：`automsgs/proto/rpcs/exploration.proto`；同步 `common.proto` Code、`system.proto`、rpcs/cli README。

## 1. 边界

| 职责 | 服务 |
|------|------|
| 自主探索覆盖 + 会话内存图 | `ExplorationService` |
| 手动建图会话 + 地图 CRUD | `MapService` |

对齐 bridge `SendExplorationCommand`：START/PAUSE/RESUME/CANCEL/SET_AREA/SAVE_MAP。

## 2. API（Navigation 风格）

```protobuf
service ExplorationService {
  rpc Explore(ExploreRequest) returns (stream ExploreResponse);
  rpc Pause(GoalRequest) returns (Status);
  rpc Resume(GoalRequest) returns (Status);
  rpc Cancel(GoalRequest) returns (Status);
  rpc GetStatus(GetStatusRequest) returns (ExploreResponse);
  rpc SetArea(SetAreaRequest) returns (Status);
  rpc SaveMap(SaveMapRequest) returns (SaveMapResponse);
}
```

- **关流：** COMPLETED / FAILED / CANCELLED / 拒收 / 断开。**PAUSED 不关流。**
- **单一活动目标**；忙 → `CODE_EXPLORATION_BUSY`。
- **无 bidi**；无 ListMaps（走 MapService）。

## 3. 状态 / 选项 / 反馈

- `ExplorationState`：UNKNOWN / IDLE / PLANNING / EXPLORING / PAUSED / COMPLETED / FAILED / CANCELLED
- `ExploreOptions`：限速、超时、覆盖阈值、是否自动开 SLAM
- `ExploreResponse`：status、state、goal_id、current_pose、explored_area_m2、progress、frontier_count、map_name

## 4. Code / System

- Code **1300–1399**：BUSY / CANCELLED / FAILED / NO_AREA / SAVE_FAILED / NOT_RUNNING
- `GOAL_KIND_EXPLORATION`；`supports_exploration`

## 5. Bridge 映射

| RPC | Dock/ExplorationCommand |
|-----|-------------------------|
| Explore | START（+ area/map_name） |
| Pause/Resume/Cancel | PAUSE/RESUME/CANCEL |
| SetArea | SET_AREA |
| SaveMap | SAVE_MAP |
