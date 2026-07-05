(rpc-integration-tests)=
# 集成测试参考

面向**系统测试**与**集成开发**的完整 grpcurl 用例库：每条用例含**命令**、**预期响应**与**判定标准**。

前置：完成 [01 §1.1](01_connection_guide.md#11-环境配置) 环境变量导出。

```bash
export REPO=/path/to/autonomy
export BRIDGE=127.0.0.1:5005
export PROTO_OPTS="-import-path $REPO -proto autonomy/bridge/proto/external_command_service.proto"
export SVC=autonomy.bridge.proto.AutonomyService
new_cmd_id() { echo "test-$(date +%s)-$RANDOM"; }

# JSON 请求体：heredoc + -d @-（标准输入，无需转义）
grpc_call() {
  grpcurl -plaintext $PROTO_OPTS -d @- "$BRIDGE" "$SVC/$1"
}
```

---

## 13.1 服务发现

服务发现与 schema 查看 → [01 §1.2](01_connection_guide.md#12-服务发现)（`list` / `describe` 命令与判定标准）。

---

## 13.2 响应判读约定

用例含 **命令**、**预期响应** JSON 与 **判定**。Command Stream 以末帧 `ack.final=true` 结束。

| 类型 | 结束判据 | 成功判据 |
|------|----------|----------|
| Query / System Unary | 单条 JSON | 字段符合下表；System 看 `success=true` |
| Command Stream | **`ack.final=true`** 末帧 | 末帧 `ack.success=true` |
| Push Stream | 无末帧，Ctrl+C 结束 | 周期输出且字段非空 |

**TaskType 整型**：导航=1 · 跟随=2 · 遥操=3 · 探索=4 · 对接=5 · 地图=6

**TaskStatus 整型**：IDLE=1 · RUNNING=2 · PAUSED=3 · SUCCEEDED=4 · FAILED=5 · CANCELED=6

每条 **TC-*** 用例采用与 Command 页相同的卡片：上方 **发送指令**（蓝）、下方 **收到结果**（绿）；纯步骤/判定类用例仅展示绿色结果区。

---

## 13.3 Query 测试用例

(tc-q-001)=
### TC-Q-001 GetCapabilities — 能力探测

<div class="nav-card">

<div class="nav-demo-grid">

<details class="nav-demo-panel nav-send">
<summary>发送指令</summary>

```bash
# TC-Q-001 — GetCapabilities — 能力探测
grpcurl -plaintext $PROTO_OPTS \
  -d '{}' \
  $BRIDGE \
  $SVC/GetCapabilities
```

</details>

<details class="nav-demo-panel nav-recv">
<summary>收到结果 · 响应</summary>

**预期响应**（字段齐全；具体 ID 因机载配置而异）：

```json
{
  "plannerIds": ["navfn_planner"],
  "controllerIds": ["graceful_controller"],
  "smootherIds": [],
  "supportsNavigation": true,
  "supportsFollow": false,
  "supportsTeleop": true,
  "supportsExploration": false,
  "supportsDocking": false,
  "supportsMapManagement": true,
  "bridgeVersion": "0.1.0",
  "autonomyVersion": "0.1.0"
}
```

**判定**：HTTP 200；`bridgeVersion` 非空；待测 Command 对应 `supports*` 为 `true`。

</details>

</div>
</div>

---

(tc-q-002)=
### TC-Q-002 GetRobotSnapshot — 空闲快照

<div class="nav-card">

<div class="nav-demo-grid">

<details class="nav-demo-panel nav-send">
<summary>发送指令</summary>

```bash
# TC-Q-002 — GetRobotSnapshot — 空闲快照
grpcurl -plaintext $PROTO_OPTS \
  -d '{}' \
  $BRIDGE \
  $SVC/GetRobotSnapshot
```

</details>

<details class="nav-demo-panel nav-recv">
<summary>收到结果 · 响应</summary>

**预期响应**（无活跃任务时）：

```json
{
  "timestamp": { "sec": "1710000000", "nanosec": 0 },
  "pose": {
    "header": { "frameId": "map" },
    "pose": {
      "position": { "x": 0.0, "y": 0.0, "z": 0.0 },
      "orientation": { "w": 1.0 }
    }
  },
  "batteryPercent": 85.0,
  "activeTaskType": "ROBOT_TASK_NONE",
  "activeTaskStatus": "ROBOT_TASK_STATUS_IDLE",
  "activeCmdId": "",
  "motionEnabled": true,
  "isDocked": false,
  "isCharging": false
}
```

**判定**：`activeTaskType` 为 `ROBOT_TASK_NONE` 或整型 `0`；`pose.header.frameId` 通常为 `map`。

</details>

</div>
</div>

---

(tc-q-003)=
### TC-Q-003 GetActiveTask — 无活跃任务

<div class="nav-card">

<div class="nav-demo-grid">

<details class="nav-demo-panel nav-send">
<summary>发送指令</summary>

```bash
# TC-Q-003 — GetActiveTask — 无活跃任务
grpcurl -plaintext $PROTO_OPTS \
  -d '{}' \
  $BRIDGE \
  $SVC/GetActiveTask
```

</details>

<details class="nav-demo-panel nav-recv">
<summary>收到结果 · 响应</summary>

**预期响应**：

```json
{
  "type": "TASK_TYPE_NONE",
  "status": "TASK_STATUS_IDLE"
}
```

**判定**：`type` 为 `TASK_TYPE_NONE`（0）；下发新 Command 前应处于此状态。

</details>

</div>
</div>

---

(tc-q-004)=
### TC-Q-004 GetActiveTask — 导航进行中

先执行 [TC-NAV-001](#tc-nav-001) 且在 Stream 未结束前，**另开终端**：

<div class="nav-card">

<div class="nav-demo-grid">

<details class="nav-demo-panel nav-send">
<summary>发送指令</summary>

```bash
# TC-Q-004 — GetActiveTask — 导航进行中
grpcurl -plaintext $PROTO_OPTS \
  -d '{}' \
  $BRIDGE \
  $SVC/GetActiveTask
```

</details>

<details class="nav-demo-panel nav-recv">
<summary>收到结果 · 响应</summary>

**预期响应**：

```json
{
  "type": "TASK_TYPE_NAVIGATION",
  "status": "TASK_STATUS_RUNNING",
  "cmdId": "test-1710000001-1234",
  "progress": 0.35
}
```

**判定**：`type=1` · `status=2` · `cmdId` 与 START 请求 `header.cmdId` 一致。

</details>

</div>
</div>

---

## 13.4 Stream 测试用例

(tc-s-001)=
### TC-S-001 ReceiveBotStates — 周期状态

<div class="nav-card">

<div class="nav-demo-grid">

<details class="nav-demo-panel nav-send">
<summary>发送指令</summary>

```bash
# 终端 A：持续订阅（Ctrl+C 结束）
grpcurl -plaintext $PROTO_OPTS \
  -d '{}' \
  $BRIDGE \
  $SVC/ReceiveBotStates
```

</details>

<details class="nav-demo-panel nav-recv">
<summary>收到结果</summary>

**预期**：每 20～100 ms 输出一帧 JSON；字段含 `pose`、`twist`、`batteryPercent`。

**判定**：连续 10 帧 `timestamp.sec` 单调递增；导航任务期间 `activeTaskType` 与 Command 一致。

</details>

</div>
</div>

---

(tc-s-002)=
### TC-S-002 ReceiveBotEvents — 任务生命周期事件

<div class="nav-card">

<div class="nav-demo-grid">

<details class="nav-demo-panel nav-send">
<summary>发送指令</summary>

```bash
# TC-S-002 — ReceiveBotEvents — 任务生命周期事件
grpcurl -plaintext $PROTO_OPTS \
  -d '{}' \
  $BRIDGE \
  $SVC/ReceiveBotEvents
```

</details>

<details class="nav-demo-panel nav-recv">
<summary>收到结果 · 事件序列</summary>

在终端 B 执行导航 START→完成，**预期事件序列**（顺序可能交错）：

```json
{ "type": "ROBOT_EVENT_TASK_STARTED", "severity": "EVENT_SEVERITY_INFO", "taskType": "ROBOT_TASK_NAVIGATION", "cmdId": "..." }
{ "type": "ROBOT_EVENT_TASK_COMPLETED", "severity": "EVENT_SEVERITY_INFO", "taskType": "ROBOT_TASK_NAVIGATION", "cmdId": "..." }
```

急停场景额外出现：

```json
{ "type": "ROBOT_EVENT_EMERGENCY_STOP", "severity": "EVENT_SEVERITY_WARNING", "message": "..." }
```

</details>

</div>
</div>

---

(tc-s-003)=
### TC-S-003 Snapshot 与 Stream 一致性

终端 A 订阅 `ReceiveBotStates`，终端 B 调用 `GetRobotSnapshot`：

<div class="nav-card">

<div class="nav-demo-grid">

<details class="nav-demo-panel nav-send">
<summary>发送指令</summary>

```bash
# 同一时刻对比（误差 ≤1 帧）
grpcurl -plaintext $PROTO_OPTS \
  -d '{}' \
  $BRIDGE \
  $SVC/GetRobotSnapshot
```

</details>

<details class="nav-demo-panel nav-recv">
<summary>收到结果</summary>

**判定**：`pose`、`activeTaskType`、`activeCmdId` 与 Stream 最近一帧一致（误差 ≤1 帧）。

</details>

</div>
</div>

---

## 13.5 System 测试用例

(tc-sys-001)=
### TC-SYS-001 EmergencyStop — 空闲时急停

<div class="nav-card">

<div class="nav-demo-grid">

<details class="nav-demo-panel nav-send">
<summary>发送指令</summary>

```bash
export CMD_ID=$(new_cmd_id)
grpcurl -plaintext $PROTO_OPTS \
  -d @- \
  $BRIDGE \
  $SVC/EmergencyStop <<EOF
{
  "header": {
    "cmd_id": "${CMD_ID}",
    "client_id": "integration-test"
  },
  "reason": "TC-SYS-001 idle estop"
}
EOF
```

</details>

<details class="nav-demo-panel nav-recv">
<summary>收到结果 · 响应</summary>

**预期响应**：

```json
{
  "success": true,
  "message": "Emergency stop acknowledged",
  "errorCode": "OK",
  "cmdId": "test-1710000002-5678",
  "taskType": "TASK_TYPE_NONE",
  "taskStatus": "TASK_STATUS_IDLE"
}
```

**判定**：Unary 单帧；`success=true`；随后 `GetActiveTask.type=NONE`。

</details>

</div>
</div>

---

(tc-sys-002)=
### TC-SYS-002 EmergencyStop — 导航中抢占

1. 终端 A 发起 [TC-NAV-001](#tc-nav-001)（保持 Stream 打开）
2. 终端 B 执行 TC-SYS-001

<div class="nav-card nav-card-single">

<div class="nav-demo-grid">

<details class="nav-demo-panel nav-recv">
<summary>收到结果</summary>

**预期**：

- 终端 A Stream 收到末帧：`ack.final=true`，`success=false` 或 `true`（实现可报 CANCELED），`taskStatus=CANCELED`
- 终端 B 急停：`success=true`
- `GetActiveTask.type=NONE`

</details>

</div>
</div>

---

(tc-sys-003)=
### TC-SYS-003 CancelAllTasks — 取消全部

<div class="nav-card">

<div class="nav-demo-grid">

<details class="nav-demo-panel nav-send">
<summary>发送指令</summary>

```bash
export CMD_ID=$(new_cmd_id)
grpcurl -plaintext $PROTO_OPTS \
  -d @- \
  $BRIDGE \
  $SVC/CancelAllTasks <<EOF
{
  "header": {
    "cmd_id": "${CMD_ID}"
  }
}
EOF
```

</details>

<details class="nav-demo-panel nav-recv">
<summary>收到结果 · 响应</summary>

**预期响应**：

```json
{
  "success": true,
  "message": "All tasks canceled",
  "taskStatus": "TASK_STATUS_IDLE"
}
```

</details>

</div>
</div>

---

(tc-sys-004)=
### TC-SYS-004 CancelAllTasks — 按类型取消

<div class="nav-card">

<div class="nav-demo-grid">

<details class="nav-demo-panel nav-send">
<summary>发送指令</summary>

```bash
export CMD_ID=$(new_cmd_id)
grpcurl -plaintext $PROTO_OPTS \
  -d @- \
  $BRIDGE \
  $SVC/CancelAllTasks <<EOF
{
  "header": {
    "cmd_id": "${CMD_ID}"
  },
  "task_types": [
    1,
    4
  ]
}
EOF
```

</details>

<details class="nav-demo-panel nav-recv">
<summary>收到结果</summary>

**预期**：仅取消导航(1)与探索(4)；其他类型任务不受影响（若存在）。

</details>

</div>
</div>

---

## 13.6 SendNavigationCommand 测试用例

(tc-nav-001)=
### TC-NAV-001 单点导航 START

<div class="nav-card">

<div class="nav-demo-grid">

<details class="nav-demo-panel nav-send">
<summary>发送指令</summary>

```bash
export CMD_ID=$(new_cmd_id)
grpcurl -plaintext $PROTO_OPTS \
  -d @- \
  $BRIDGE \
  $SVC/SendNavigationCommand <<EOF
{
  "header": {
    "cmd_id": "${CMD_ID}",
    "client_id": "integration-test"
  },
  "command": 1,
  "mode": 1,
  "goals": [
    {
      "header": {
        "frame_id": "map"
      },
      "pose": {
        "position": {
          "x": 1.0,
          "y": 2.0,
          "z": 0.0
        },
        "orientation": {
          "w": 1.0
        }
      }
    }
  ],
  "plugins": {
    "planner_id": "navfn_planner",
    "controller_id": "graceful_controller"
  },
  "timeout_sec": 300
}
EOF
```

</details>

<details class="nav-demo-panel nav-recv">
<summary>收到结果 · Stream</summary>

**预期 Stream 序列**：

| 帧序 | 关键字段 | 说明 |
|------|----------|------|
| 1 | `ack.final=false`, `status=NAV_STATUS_PLANNING` 或 `NAV_STATUS_NAVIGATING` | 任务已接受 |
| 2…n | `distanceRemaining` 递减, `currentPose` 更新 | 进度帧 |
| 末帧 | `ack.final=true`, `ack.success=true`, `status=NAV_STATUS_SUCCEEDED` | 到达目标 |

**末帧示例**：

```json
{
  "ack": {
    "success": true,
    "final": true,
    "cmdId": "test-1710000001-1234",
    "taskType": "TASK_TYPE_NAVIGATION",
    "taskStatus": "TASK_STATUS_SUCCEEDED"
  },
  "status": "NAV_STATUS_SUCCEEDED",
  "distanceRemaining": 0.0
}
```

</details>

</div>
</div>

---

(tc-nav-002)=
### TC-NAV-002 多点巡航 START

<div class="nav-card">

<div class="nav-demo-grid">

<details class="nav-demo-panel nav-send">
<summary>发送指令</summary>

```bash
export CMD_ID=$(new_cmd_id)
grpcurl -plaintext $PROTO_OPTS \
  -d @- \
  $BRIDGE \
  $SVC/SendNavigationCommand <<EOF
{
  "header": {
    "cmd_id": "${CMD_ID}"
  },
  "command": 1,
  "mode": 2,
  "goals": [
    {
      "header": {
        "frame_id": "map"
      },
      "pose": {
        "position": {
          "x": 1.0,
          "y": 1.0
        }
      }
    },
    {
      "header": {
        "frame_id": "map"
      },
      "pose": {
        "position": {
          "x": 3.0,
          "y": 1.0
        }
      }
    },
    {
      "header": {
        "frame_id": "map"
      },
      "pose": {
        "position": {
          "x": 3.0,
          "y": 3.0
        }
      }
    }
  ]
}
EOF
```

</details>

<details class="nav-demo-panel nav-recv">
<summary>收到结果</summary>

**预期**：`totalWaypoints=3`；`currentWaypointIndex` 从 0 递增至 2；末帧 `SUCCEEDED`。

</details>

</div>
</div>

---

(tc-nav-003)=
### TC-NAV-003 PAUSE / RESUME

导航进行中（Stream 未结束），**新 RPC 调用**（若实现支持同任务控制）或先发 PAUSE：

<div class="nav-card">

<div class="nav-demo-grid">

<details class="nav-demo-panel nav-send">
<summary>发送指令</summary>

```bash
export CMD_ID=$(new_cmd_id)
grpcurl -plaintext $PROTO_OPTS \
  -d @- \
  $BRIDGE \
  $SVC/SendNavigationCommand <<EOF
{
  "header": {
    "cmd_id": "${CMD_ID}"
  },
  "command": 5
}
EOF
```

</details>

<details class="nav-demo-panel nav-recv">
<summary>收到结果</summary>

**预期**：进度帧 `status=NAV_STATUS_PAUSED`，`ack.taskStatus=TASK_STATUS_PAUSED`。

RESUME（`command=4`）后恢复 `NAV_STATUS_NAVIGATING`。

</details>

</div>
</div>

---

(tc-nav-004)=
### TC-NAV-004 STOP — 停止当前导航

<div class="nav-card">

<div class="nav-demo-grid">

<details class="nav-demo-panel nav-send">
<summary>发送指令</summary>

```bash
export CMD_ID=$(new_cmd_id)
grpcurl -plaintext $PROTO_OPTS \
  -d @- \
  $BRIDGE \
  $SVC/SendNavigationCommand <<EOF
{
  "header": {
    "cmd_id": "${CMD_ID}"
  },
  "command": 2
}
EOF
```

</details>

<details class="nav-demo-panel nav-recv">
<summary>收到结果 · 末帧</summary>

**预期末帧**：

```json
{
  "ack": { "success": true, "final": true, "taskStatus": "TASK_STATUS_IDLE" },
  "status": "NAV_STATUS_IDLE"
}
```

</details>

</div>
</div>

---

(tc-nav-005)=
### TC-NAV-005 CANCEL — 取消并清理

<div class="nav-card">

<div class="nav-demo-grid">

<details class="nav-demo-panel nav-send">
<summary>发送指令</summary>

```bash
export CMD_ID=$(new_cmd_id)
grpcurl -plaintext $PROTO_OPTS \
  -d @- \
  $BRIDGE \
  $SVC/SendNavigationCommand <<EOF
{
  "header": {
    "cmd_id": "${CMD_ID}"
  },
  "command": 3
}
EOF
```

</details>

<details class="nav-demo-panel nav-recv">
<summary>收到结果 · 末帧</summary>

**预期末帧**：`status=NAV_STATUS_CANCELED` · `taskStatus=TASK_STATUS_CANCELED`。

</details>

</div>
</div>

---

(tc-nav-006)=
### TC-NAV-006 REPLAN — 保持目标重规划

导航进行中：

<div class="nav-card">

<div class="nav-demo-grid">

<details class="nav-demo-panel nav-send">
<summary>发送指令</summary>

```bash
export CMD_ID=$(new_cmd_id)
grpcurl -plaintext $PROTO_OPTS \
  -d @- \
  $BRIDGE \
  $SVC/SendNavigationCommand <<EOF
{
  "header": {
    "cmd_id": "${CMD_ID}"
  },
  "command": 6
}
EOF
```

</details>

<details class="nav-demo-panel nav-recv">
<summary>收到结果</summary>

**预期**：出现 `status=NAV_STATUS_PLANNING` 帧，随后回到 `NAV_STATUS_NAVIGATING`；`goals` 不变。

</details>

</div>
</div>

---

(tc-nav-neg-001)=
### TC-NAV-NEG-001 缺少 header — 参数错误

<div class="nav-card">

<div class="nav-demo-grid">

<details class="nav-demo-panel nav-send">
<summary>发送指令</summary>

```bash
# TC-NAV-NEG-001 — 缺少 header — 参数错误
grpcurl -plaintext $PROTO_OPTS \
  -d @- \
  $BRIDGE \
  $SVC/SendNavigationCommand <<EOF
{
  "command": 2
}
EOF
```

</details>

<details class="nav-demo-panel nav-recv">
<summary>收到结果</summary>

**预期**：gRPC `INVALID_ARGUMENT`；无 Stream 末帧。

</details>

</div>
</div>

---

(tc-nav-neg-002)=
### TC-NAV-NEG-002 任务互斥 — 重复 START

任务 A 未结束时再 START：

<div class="nav-card nav-card-single">

<div class="nav-demo-grid">

<details class="nav-demo-panel nav-recv">
<summary>收到结果</summary>

**预期**：gRPC `FAILED_PRECONDITION` 或首帧 `ack.success=false`，`errorCode` 含占用提示。

</details>

</div>
</div>

---

(tc-nav-neg-003)=
### TC-NAV-NEG-003 不可达目标

<div class="nav-card">

<div class="nav-demo-grid">

<details class="nav-demo-panel nav-send">
<summary>发送指令</summary>

```bash
export CMD_ID=$(new_cmd_id)
grpcurl -plaintext $PROTO_OPTS \
  -d @- \
  $BRIDGE \
  $SVC/SendNavigationCommand <<EOF
{
  "header": {
    "cmd_id": "${CMD_ID}"
  },
  "command": 1,
  "mode": 1,
  "goals": [
    {
      "header": {
        "frame_id": "map"
      },
      "pose": {
        "position": {
          "x": 9999,
          "y": 9999
        }
      }
    }
  ]
}
EOF
```

</details>

<details class="nav-demo-panel nav-recv">
<summary>收到结果 · 末帧</summary>

**预期末帧**：

```json
{
  "ack": {
    "success": false,
    "final": true,
    "errorCode": "PLANNING_NO_PATH_FOUND",
    "message": "No path found"
  },
  "status": "NAV_STATUS_FAILED"
}
```

</details>

</div>
</div>

---

## 13.7 SendExplorationCommand 测试用例

(tc-exp-001)=
### TC-EXP-001 START 探索

<div class="nav-card">

<div class="nav-demo-grid">

<details class="nav-demo-panel nav-send">
<summary>发送指令</summary>

```bash
export CMD_ID=$(new_cmd_id)
grpcurl -plaintext $PROTO_OPTS \
  -d @- \
  $BRIDGE \
  $SVC/SendExplorationCommand <<EOF
{
  "header": {
    "cmd_id": "${CMD_ID}"
  },
  "command": 1
}
EOF
```

</details>

<details class="nav-demo-panel nav-recv">
<summary>收到结果</summary>

**预期**：`status=EXPLORATION_STATUS_EXPLORING`；`explorationProgress` 从 0 增大。

</details>

</div>
</div>

---

(tc-exp-002)=
### TC-EXP-002 PAUSE / RESUME

<div class="nav-card">

<div class="nav-demo-grid">

<details class="nav-demo-panel nav-send">
<summary>发送指令</summary>

```bash
# PAUSE command=3 → status=EXPLORATION_STATUS_PAUSED
# RESUME command=4 → status=EXPLORATION_STATUS_EXPLORING
```

</details>

<details class="nav-demo-panel nav-recv">
<summary>收到结果</summary>

**预期**：PAUSE `command=3` → `status=EXPLORATION_STATUS_PAUSED`；RESUME `command=4` → `status=EXPLORATION_STATUS_EXPLORING`。

</details>

</div>
</div>

---

(tc-exp-003)=
### TC-EXP-003 SAVE_MAP

<div class="nav-card">

<div class="nav-demo-grid">

<details class="nav-demo-panel nav-send">
<summary>发送指令</summary>

```bash
export CMD_ID=$(new_cmd_id)
grpcurl -plaintext $PROTO_OPTS \
  -d @- \
  $BRIDGE \
  $SVC/SendExplorationCommand <<EOF
{
  "header": {
    "cmd_id": "${CMD_ID}"
  },
  "command": 6,
  "map_name": "warehouse_floor1"
}
EOF
```

</details>

<details class="nav-demo-panel nav-recv">
<summary>收到结果 · 末帧</summary>

**预期末帧**：`success=true` · `status=EXPLORATION_STATUS_IDLE` 或 `COMPLETED`。

</details>

</div>
</div>

---

(tc-exp-004)=
### TC-EXP-004 SET_AREA 限定区域

<div class="nav-card">

<div class="nav-demo-grid">

<details class="nav-demo-panel nav-send">
<summary>发送指令</summary>

```bash
export CMD_ID=$(new_cmd_id)
grpcurl -plaintext $PROTO_OPTS \
  -d @- \
  $BRIDGE \
  $SVC/SendExplorationCommand <<EOF
{
  "header": {
    "cmd_id": "${CMD_ID}"
  },
  "command": 7,
  "area": {
    "points": [
      {
        "x": 0,
        "y": 0,
        "z": 0
      },
      {
        "x": 20,
        "y": 0,
        "z": 0
      },
      {
        "x": 20,
        "y": 15,
        "z": 0
      },
      {
        "x": 0,
        "y": 15,
        "z": 0
      }
    ]
  }
}
EOF
```

</details>

<details class="nav-demo-panel nav-recv">
<summary>收到结果</summary>

**预期**：`success=true`；后续 START 仅在该 Polygon 内探索。

</details>

</div>
</div>

---

## 13.8 SendFollowCommand 测试用例

(tc-fol-001)=
### TC-FOL-001 跟随行人模式 START

<div class="nav-card">

<div class="nav-demo-grid">

<details class="nav-demo-panel nav-send">
<summary>发送指令</summary>

```bash
export CMD_ID=$(new_cmd_id)
grpcurl -plaintext $PROTO_OPTS \
  -d @- \
  $BRIDGE \
  $SVC/SendFollowCommand <<EOF
{
  "header": {
    "cmd_id": "${CMD_ID}"
  },
  "command": 1,
  "mode": 1,
  "follow_distance": 1.5,
  "follow_angle": 0.0,
  "session_timeout_sec": 600,
  "reacquire_on_lost": true
}
EOF
```

</details>

<details class="nav-demo-panel nav-recv">
<summary>收到结果</summary>

**预期**：`status=FOLLOW_STATUS_FOLLOWING`；`distanceToTarget` 接近 `followDistance`。

</details>

</div>
</div>

---

(tc-fol-002)=
### TC-FOL-002 跟随目标位姿

<div class="nav-card">

<div class="nav-demo-grid">

<details class="nav-demo-panel nav-send">
<summary>发送指令</summary>

```bash
export CMD_ID=$(new_cmd_id)
grpcurl -plaintext $PROTO_OPTS \
  -d @- \
  $BRIDGE \
  $SVC/SendFollowCommand <<EOF
{
  "header": {
    "cmd_id": "${CMD_ID}"
  },
  "command": 1,
  "mode": 2,
  "follow_distance": 2.0,
  "target_pose": {
    "header": {
      "frame_id": "map"
    },
    "pose": {
      "position": {
        "x": 5.0,
        "y": 3.0
      },
      "orientation": {
        "w": 1.0
      }
    }
  }
}
EOF
```

</details>

<details class="nav-demo-panel nav-recv">
<summary>收到结果</summary>

**预期**：`status=FOLLOW_STATUS_FOLLOWING`（mode=2 位姿模式）。

</details>

</div>
</div>

---

(tc-fol-003)=
### TC-FOL-003 UPDATE_TARGET 动态更新

<div class="nav-card">

<div class="nav-demo-grid">

<details class="nav-demo-panel nav-send">
<summary>发送指令</summary>

```bash
export CMD_ID=$(new_cmd_id)
grpcurl -plaintext $PROTO_OPTS \
  -d @- \
  $BRIDGE \
  $SVC/SendFollowCommand <<EOF
{
  "header": {
    "cmd_id": "${CMD_ID}"
  },
  "command": 6,
  "mode": 2,
  "target_pose": {
    "header": {
      "frame_id": "map"
    },
    "pose": {
      "position": {
        "x": 8.0,
        "y": 4.0
      }
    }
  }
}
EOF
```

</details>

<details class="nav-demo-panel nav-recv">
<summary>收到结果</summary>

**预期**：Stream 进度帧中 `targetPose` 更新；状态保持 `FOLLOWING`。

</details>

</div>
</div>

---

(tc-fol-004)=
### TC-FOL-004 目标丢失

目标离开视野且 `reacquire_on_lost=false`：

<div class="nav-card nav-card-single">

<div class="nav-demo-grid">

<details class="nav-demo-panel nav-recv">
<summary>收到结果</summary>

**预期**：`status=FOLLOW_STATUS_TARGET_LOST` 或末帧 `FAILED`。

</details>

</div>
</div>

---

## 13.9 SendTeleopCommand 测试用例

grpcurl 对 Bidi Stream 能力有限；**完整测试请用 Python**（见 [§13.14](#1314-python-集成测试脚本)）。

(tc-tel-001)=
### TC-TEL-001 START 探测

<div class="nav-card">

<div class="nav-demo-grid">

<details class="nav-demo-panel nav-send">
<summary>发送指令</summary>

```bash
export CMD_ID=$(new_cmd_id)
grpcurl -plaintext $PROTO_OPTS \
  -d @- \
  $BRIDGE \
  $SVC/SendTeleopCommand <<EOF
{
  "header": {
    "cmd_id": "${CMD_ID}"
  },
  "command": 1,
  "watchdog_timeout_sec": 2.0,
  "max_linear_speed": 0.5,
  "max_angular_speed": 1.0
}
EOF
```

</details>

<details class="nav-demo-panel nav-recv">
<summary>收到结果 · 首帧 / 超时</summary>

**预期首帧**：`status=TELEOP_STATUS_ACTIVE` 或 `IDLE`（视实现）。

**预期超时**（无 VELOCITY 帧 2s 后）：末帧 `status=TELEOP_STATUS_TIMEOUT`。

</details>

</div>
</div>

---

(tc-tel-002)=
### TC-TEL-002 VELOCITY 连续发送（Python）

<div class="nav-card">

<div class="nav-demo-grid">

<details class="nav-demo-panel nav-send">
<summary>发送指令</summary>

```python
# 见 §13.14 — 每 100ms 发送 command=3 + twist
```

</details>

<details class="nav-demo-panel nav-recv">
<summary>收到结果</summary>

**预期**：`status=TELEOP_STATUS_ACTIVE`；`GetActiveTask.type=TELEOP(3)`

</details>

</div>
</div>

---

(tc-tel-003)=
### TC-TEL-003 STOP

<div class="nav-card">

<div class="nav-demo-grid">

<details class="nav-demo-panel nav-send">
<summary>发送指令</summary>

```python
# command=2 → TELEOP_CMD_STOP
```

</details>

<details class="nav-demo-panel nav-recv">
<summary>收到结果 · 末帧</summary>

**预期**：`status=TELEOP_STATUS_IDLE`；`GetActiveTask.type=NONE`。

</details>

</div>
</div>

---

## 13.10 SendDockCommand 测试用例

(tc-dck-001)=
### TC-DCK-001 按站点 ID 对接

<div class="nav-card">

<div class="nav-demo-grid">

<details class="nav-demo-panel nav-send">
<summary>发送指令</summary>

```bash
export CMD_ID=$(new_cmd_id)
grpcurl -plaintext $PROTO_OPTS \
  -d @- \
  $BRIDGE \
  $SVC/SendDockCommand <<EOF
{
  "header": {
    "cmd_id": "${CMD_ID}"
  },
  "command": 1,
  "dock_station_id": "charge_station_a",
  "max_search_radius": 5.0,
  "max_retry_count": 3
}
EOF
```

</details>

<details class="nav-demo-panel nav-recv">
<summary>收到结果 · 状态序列</summary>

**预期状态序列**：`SEARCHING` → `APPROACHING` → `DOCKING` → `CHARGING` → 末帧 `SUCCEEDED`。

**末帧示例**：

```json
{
  "ack": { "success": true, "final": true, "taskStatus": "TASK_STATUS_SUCCEEDED" },
  "status": "DOCK_STATUS_SUCCEEDED",
  "batteryPercent": 45.0,
  "dockStationId": "charge_station_a"
}
```

</details>

</div>
</div>

---

(tc-dck-002)=
### TC-DCK-002 按位姿对接

<div class="nav-card">

<div class="nav-demo-grid">

<details class="nav-demo-panel nav-send">
<summary>发送指令</summary>

```bash
export CMD_ID=$(new_cmd_id)
grpcurl -plaintext $PROTO_OPTS \
  -d @- \
  $BRIDGE \
  $SVC/SendDockCommand <<EOF
{
  "header": {
    "cmd_id": "${CMD_ID}"
  },
  "command": 1,
  "dock_pose": {
    "header": {
      "frame_id": "map"
    },
    "pose": {
      "position": {
        "x": 0.5,
        "y": 0.0
      },
      "orientation": {
        "w": 1.0
      }
    }
  }
}
EOF
```

</details>

<details class="nav-demo-panel nav-recv">
<summary>收到结果 · 末帧</summary>

**预期**：`SEARCHING` → `APPROACHING` → `DOCKING` → 末帧 `DOCK_STATUS_SUCCEEDED`（使用 `dock_pose` 定位）。

</details>

</div>
</div>

---

(tc-dck-003)=
### TC-DCK-003 UNDOCK

<div class="nav-card">

<div class="nav-demo-grid">

<details class="nav-demo-panel nav-send">
<summary>发送指令</summary>

```bash
export CMD_ID=$(new_cmd_id)
grpcurl -plaintext $PROTO_OPTS \
  -d @- \
  $BRIDGE \
  $SVC/SendDockCommand <<EOF
{
  "header": {
    "cmd_id": "${CMD_ID}"
  },
  "command": 6
}
EOF
```

</details>

<details class="nav-demo-panel nav-recv">
<summary>收到结果</summary>

**预期**：`status=DOCK_STATUS_UNDOCKING` → `SUCCEEDED`；`GetRobotSnapshot.isDocked=false`。

</details>

</div>
</div>

---

## 13.11 SendMapCommand 测试用例

(tc-map-001)=
### TC-MAP-001 LOAD 地图

<div class="nav-card">

<div class="nav-demo-grid">

<details class="nav-demo-panel nav-send">
<summary>发送指令</summary>

```bash
export CMD_ID=$(new_cmd_id)
grpcurl -plaintext $PROTO_OPTS \
  -d @- \
  $BRIDGE \
  $SVC/SendMapCommand <<EOF
{
  "header": {
    "cmd_id": "${CMD_ID}"
  },
  "command": 1,
  "map_name": "warehouse_floor1"
}
EOF
```

</details>

<details class="nav-demo-panel nav-recv">
<summary>收到结果 · 末帧</summary>

**预期末帧**：

```json
{
  "ack": { "success": true, "final": true },
  "status": "MAP_STATUS_SUCCEEDED",
  "currentMapName": "warehouse_floor1"
}
```

</details>

</div>
</div>

---

(tc-map-002)=
### TC-MAP-002 SWITCH 地图

<div class="nav-card">

<div class="nav-demo-grid">

<details class="nav-demo-panel nav-send">
<summary>发送指令</summary>

```bash
export CMD_ID=$(new_cmd_id)
grpcurl -plaintext $PROTO_OPTS \
  -d @- \
  $BRIDGE \
  $SVC/SendMapCommand <<EOF
{
  "header": {
    "cmd_id": "${CMD_ID}"
  },
  "command": 2,
  "map_name": "warehouse_floor2"
}
EOF
```

</details>

<details class="nav-demo-panel nav-recv">
<summary>收到结果 · 末帧</summary>

**预期末帧**：`status=MAP_STATUS_SUCCEEDED`，`currentMapName=warehouse_floor2`。

</details>

</div>
</div>

---

(tc-map-003)=
### TC-MAP-003 SET_INITIAL_POSE 重定位

<div class="nav-card">

<div class="nav-demo-grid">

<details class="nav-demo-panel nav-send">
<summary>发送指令</summary>

```bash
export CMD_ID=$(new_cmd_id)
grpcurl -plaintext $PROTO_OPTS \
  -d @- \
  $BRIDGE \
  $SVC/SendMapCommand <<EOF
{
  "header": {
    "cmd_id": "${CMD_ID}"
  },
  "command": 3,
  "initial_pose": {
    "header": {
      "frame_id": "map"
    },
    "pose": {
      "pose": {
        "position": {
          "x": 2.0,
          "y": 3.0,
          "z": 0.0
        },
        "orientation": {
          "w": 1.0
        }
      },
      "covariance": [
        0.25,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0685
      ]
    }
  }
}
EOF
```

</details>

<details class="nav-demo-panel nav-recv">
<summary>收到结果</summary>

**预期**：`success=true`；随后 `GetRobotSnapshot.pose` 接近设定值；`localizationQuality` 上升。

</details>

</div>
</div>

---

(tc-map-004)=
### TC-MAP-004 CLEAR_COSTMAP

<div class="nav-card">

<div class="nav-demo-grid">

<details class="nav-demo-panel nav-send">
<summary>发送指令</summary>

```bash
export CMD_ID=$(new_cmd_id)
grpcurl -plaintext $PROTO_OPTS \
  -d @- \
  $BRIDGE \
  $SVC/SendMapCommand <<EOF
{
  "header": {
    "cmd_id": "${CMD_ID}"
  },
  "command": 4
}
EOF
```

</details>

<details class="nav-demo-panel nav-recv">
<summary>收到结果 · 末帧</summary>

**预期末帧**：`status=MAP_STATUS_SUCCEEDED`（Unary→Stream 单帧或短 Stream）。

</details>

</div>
</div>

---

(tc-map-005)=
### TC-MAP-005 ADD / REMOVE 禁行区

<div class="nav-card">

<div class="nav-demo-grid">

<details class="nav-demo-panel nav-send">
<summary>发送指令</summary>

**ADD command=5**

```bash
export CMD_ID=$(new_cmd_id)

grpcurl -plaintext $PROTO_OPTS \
  -d @- \
  $BRIDGE \
  $SVC/SendMapCommand <<EOF
{
  "header": {
    "cmd_id": "${CMD_ID}"
  },
  "command": 5,
  "keep_out_zone": {
    "points": [
      {
        "x": 5,
        "y": 5,
        "z": 0
      },
      {
        "x": 6,
        "y": 5,
        "z": 0
      },
      {
        "x": 6,
        "y": 6,
        "z": 0
      }
    ]
  }
}
EOF
```

**REMOVE command=6**

```bash
export CMD_ID=$(new_cmd_id)

grpcurl -plaintext $PROTO_OPTS \
  -d @- \
  $BRIDGE \
  $SVC/SendMapCommand <<EOF
{
  "header": {
    "cmd_id": "${CMD_ID}"
  },
  "command": 6,
  "zone_id": "zone-001"
}
EOF
```

</details>

<details class="nav-demo-panel nav-recv">
<summary>收到结果 · 末帧</summary>

**预期**：ADD / REMOVE 两次调用均 `status=MAP_STATUS_SUCCEEDED`；`ack.success=true`。

</details>

</div>
</div>

---

## 13.12 端到端集成场景

(e2e-001)=
### E2E-001 标准导航闭环

| 步骤 | RPC | 预期 |
|------|-----|------|
| 1 | `GetCapabilities` | `supportsNavigation=true` |
| 2 | `GetActiveTask` | `type=NONE` |
| 3 | `SendNavigationCommand` START | Stream 进度帧 |
| 4 | `GetActiveTask` | `type=NAVIGATION`, `RUNNING` |
| 5 | `ReceiveBotStates` | `activeCmdId` 匹配 |
| 6 | Stream 末帧 | `SUCCEEDED` |
| 7 | `GetActiveTask` | `type=NONE` |

---

(e2e-002)=
### E2E-002 急停打断导航

| 步骤 | RPC | 预期 |
|------|-----|------|
| 1 | START 导航 | `NAVIGATING` |
| 2 | `EmergencyStop` | `success=true` |
| 3 | 导航 Stream | 末帧 `CANCELED` 或 `FAILED` |
| 4 | `ReceiveBotEvents` | `ROBOT_EVENT_EMERGENCY_STOP` |
| 5 | `GetActiveTask` | `type=NONE` |

---

(e2e-003)=
### E2E-003 探索 → 保存地图 → 加载导航

| 步骤 | RPC | 预期 |
|------|-----|------|
| 1 | `SendExplorationCommand` START | `EXPLORING` |
| 2 | `SendExplorationCommand` SAVE_MAP | `map_name` 写入 |
| 3 | `SendMapCommand` LOAD 同名地图 | `currentMapName` 一致 |
| 4 | `SendNavigationCommand` START | 在新地图上 `SUCCEEDED` |

---

(e2e-004)=
### E2E-004 重定位后导航

| 步骤 | RPC | 预期 |
|------|-----|------|
| 1 | `SendMapCommand` SET_INITIAL_POSE | `success=true` |
| 2 | `GetRobotSnapshot` | `pose` 与设定一致 |
| 3 | `SendNavigationCommand` START | 正常规划 |

---

## 13.13 一键冒烟序列（bash）

复制整段执行；每步打印 `[OK]` / `[FAIL]`（需 Bridge 运行中）：

```bash
set -e
REPO=${REPO:-/path/to/autonomy}
BRIDGE=${BRIDGE:-127.0.0.1:5005}
PROTO_OPTS="-import-path $REPO -proto autonomy/bridge/proto/external_command_service.proto"
SVC=autonomy.bridge.proto.AutonomyService

run() { echo ">>> $1"; shift; "$@" && echo "[OK] $1" || echo "[FAIL] $1"; }

run "list services" grpcurl -plaintext $PROTO_OPTS $BRIDGE list
run "GetCapabilities" grpcurl -plaintext $PROTO_OPTS -d '{}' $BRIDGE $SVC/GetCapabilities
run "GetRobotSnapshot" grpcurl -plaintext $PROTO_OPTS -d '{}' $BRIDGE $SVC/GetRobotSnapshot
run "GetActiveTask" grpcurl -plaintext $PROTO_OPTS -d '{}' $BRIDGE $SVC/GetActiveTask

CID="smoke-$(date +%s)"
run "Nav STOP" grpcurl -plaintext $PROTO_OPTS -d @- $BRIDGE $SVC/SendNavigationCommand <<EOF
{
  "header": { "cmd_id": "${CID}-stop" },
  "command": 2
}
EOF

run "EmergencyStop" grpcurl -plaintext $PROTO_OPTS -d @- $BRIDGE $SVC/EmergencyStop <<EOF
{
  "header": { "cmd_id": "${CID}-estop" },
  "reason": "smoke"
}
EOF

run "CancelAllTasks" grpcurl -plaintext $PROTO_OPTS -d @- $BRIDGE $SVC/CancelAllTasks <<EOF
{
  "header": { "cmd_id": "${CID}-cancel" }
}
EOF

echo "Smoke done. For START nav / Stream subscribe, see sections 13.6 / 13.4."
```

---

## 13.14 Python 集成测试脚本

依赖：`pip install grpcio grpcio-tools`

```python
#!/usr/bin/env python3
"""AutonomyService 集成测试示例 — 覆盖 Query / Nav STOP / Stream 采样。"""
import sys
import time
import grpc
from google.protobuf import empty_pb2

# 假设已 protoc 生成到 PYTHONPATH
from autonomy.bridge.proto import external_command_service_pb2 as pb
from autonomy.bridge.proto import external_command_service_pb2_grpc as stubs

BRIDGE = "127.0.0.1:5005"
CLIENT_ID = "integration-test"


def main() -> int:
    ch = grpc.insecure_channel(BRIDGE)
    stub = stubs.AutonomyServiceStub(ch)

    # TC-Q-001
    caps = stub.GetCapabilities(empty_pb2.Empty())
    print("Capabilities:", caps.bridge_version, "nav=", caps.supports_navigation)
    assert caps.bridge_version, "bridge_version empty"

    # TC-Q-003
    task = stub.GetActiveTask(empty_pb2.Empty())
    print("ActiveTask:", task.type, task.status)

    # TC-NAV-004 STOP
    cmd_id = f"py-{int(time.time())}"
    final = None
    for resp in stub.SendNavigationCommand(pb.NavigationCommandRequest(
        header=pb.RequestHeader(cmd_id=cmd_id, client_id=CLIENT_ID),
        command=pb.NAV_CMD_STOP,
    )):
        print("Nav STOP ack:", resp.ack.success, resp.ack.final, resp.status)
        if resp.ack.final:
            final = resp
            break
    assert final and final.ack.final, "missing final frame"

    # TC-S-001 采样 3 帧
    stream = stub.ReceiveBotStates(empty_pb2.Empty())
    for i in range(3):
        state = next(stream)
        print(f"State[{i}]: bat={state.battery_percent:.0f}% task={state.active_task_type}")

    # TC-SYS-001
    ack = stub.EmergencyStop(pb.EmergencyStopRequest(
        header=pb.RequestHeader(cmd_id=f"estop-{cmd_id}", client_id=CLIENT_ID),
        reason="python integration test",
    ))
    print("Estop:", ack.success, ack.message)
    assert ack.success

    print("ALL PASSED")
    return 0


if __name__ == "__main__":
    sys.exit(main())
```

**Teleop Bidi 示例**（TC-TEL-002）：

```python
from autonomy.commsgs.proto import geometry_msgs_pb2

def teleop_velocity_test(stub, duration_sec=3.0):
    cmd_id = f"teleop-{int(time.time())}"

    def requests():
        yield pb.TeleopCommandRequest(
            header=pb.RequestHeader(cmd_id=cmd_id, client_id=CLIENT_ID),
            command=pb.TELEOP_CMD_START,
            watchdog_timeout_sec=1.0,
            max_linear_speed=0.3,
        )
        t0 = time.time()
        while time.time() - t0 < duration_sec:
            yield pb.TeleopCommandRequest(
                header=pb.RequestHeader(cmd_id=cmd_id, client_id=CLIENT_ID),
                command=pb.TELEOP_CMD_VELOCITY,
                velocity=geometry_msgs_pb2.TwistStamped(
                    twist=geometry_msgs_pb2.Twist(
                        linear=geometry_msgs_pb2.Vector3(x=0.1),
                    )
                ),
            )
            time.sleep(0.1)
        yield pb.TeleopCommandRequest(
            header=pb.RequestHeader(cmd_id=cmd_id, client_id=CLIENT_ID),
            command=pb.TELEOP_CMD_STOP,
        )

    for resp in stub.SendTeleopCommand(requests()):
        if resp.ack.final:
            assert resp.status == pb.TELEOP_STATUS_IDLE or resp.ack.success
            break
```

---

## 13.15 用例索引

| 分类 | 用例 ID | 章节 |
|------|---------|------|
| Query | TC-Q-001 … 004 | §13.3 |
| Stream | TC-S-001 … 003 | §13.4 |
| System | TC-SYS-001 … 004 | §13.5 |
| Navigation | TC-NAV-001 … 006, TC-NAV-NEG-001 … 003 | §13.6 |
| Exploration | TC-EXP-001 … 004 | §13.7 |
| Follow | TC-FOL-001 … 004 | §13.8 |
| Teleop | TC-TEL-001 … 003 | §13.9 |
| Dock | TC-DCK-001 … 003 | §13.10 |
| Map | TC-MAP-001 … 005 | §13.11 |
| E2E | E2E-001 … 004 | §13.12 |
