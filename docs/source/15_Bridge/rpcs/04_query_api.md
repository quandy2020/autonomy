(rpc-query-api)=
# 查询接口 Query

无副作用只读 RPC，用于连通性验证、能力探测与状态快照。**调用任意 Command 前应先执行 `GetCapabilities`。**

## 4.1 方法列表

| RPC | 响应 | 用途 |
|-----|------|------|
| `GetCapabilities` | `Capabilities` | 能力矩阵、插件 ID、版本 |
| `GetRobotSnapshot` | `RobotState` | 单次快照，等同 Stream 一帧 |
| `GetActiveTask` | `ActiveTaskInfo` | 当前活跃 Command；`type=NONE` 表示空闲 |

## 4.2 Capabilities

| 字段 | 类型 | 说明 |
|------|------|------|
| `planner_ids` | `string[]` | 可用全局规划器，填入 [07 §7.3 plugins](07_navigation_command.md#73-字段说明) |
| `controller_ids` | `string[]` | 可用局部控制器 |
| `smoother_ids` | `string[]` | 可用速度平滑器 |
| `supports_navigation` | `bool` | 可否调用 `SendNavigationCommand` |
| `supports_follow` | `bool` | 可否调用 `SendFollowCommand` |
| `supports_teleop` | `bool` | 可否调用 `SendTeleopCommand` |
| `supports_exploration` | `bool` | 可否调用 `SendExplorationCommand` |
| `supports_docking` | `bool` | 可否调用 `SendDockCommand` |
| `supports_map_management` | `bool` | 可否调用 `SendMapCommand` |
| `bridge_version` | `string` | Bridge 版本 |
| `autonomy_version` | `string` | 机载栈版本 |

`supports_*` 与 Command 对照见 [02 §2.5](02_service_overview.md#25-command-与-tasktype-对照)。

## 4.3 ActiveTaskInfo

| 字段 | 说明 |
|------|------|
| `type` | `TaskType`；`TASK_TYPE_NONE`(0) = 无活跃任务 |
| `status` | `TaskStatus` |
| `cmd_id` | 活跃命令 ID |
| `client_id` | 发起方 |
| `started_at` | 开始时间 |
| `progress` | 进度 [0, 1]，可选 |

## 4.4 grpcurl

JSON 样例 → [01 §1.4](01_connection_guide.md#14-响应-json-参考) · 完整用例 → [13 §13.3](13_integration_tests.md#133-query-测试用例)。

**GetCapabilities**

```bash
# GetCapabilities — 能力探测（Query · Unary · 无副作用）
grpcurl -plaintext $PROTO_OPTS \
  -d '{}' \
  $BRIDGE \
  $SVC/GetCapabilities
```


**GetRobotSnapshot**

```bash
# GetRobotSnapshot — 机器人状态快照（Query · Unary）
grpcurl -plaintext $PROTO_OPTS \
  -d '{}' \
  $BRIDGE \
  $SVC/GetRobotSnapshot
```


**GetActiveTask**

```bash
# GetActiveTask — 活跃任务查询（Query · Unary）
grpcurl -plaintext $PROTO_OPTS \
  -d '{}' \
  $BRIDGE \
  $SVC/GetActiveTask
```
