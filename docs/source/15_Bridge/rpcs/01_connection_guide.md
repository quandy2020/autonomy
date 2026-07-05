(bridge-connection-guide)=
# 接入与验证

系统测试与研发联调 AutonomyService 的**标准接入流程**。完整用例库（含预期 JSON、E2E 场景）见 **[13 集成测试参考](13_integration_tests.md)**。

读完本章后按 [02 服务概述](02_service_overview.md) → [03 公共消息类型](03_common_types.md) 继续。

## 1.1 环境配置

| 参数 | 默认值 | 说明 |
|------|--------|------|
| 地址 | `127.0.0.1:5005` | 机载 [§1 Bridge 配置](../01_options.md) |
| 服务 | `autonomy.bridge.proto.AutonomyService` | gRPC 全名 |
| Proto | `autonomy/bridge/proto/external_command_service.proto` | 需 `-import-path` 指向仓库根 |

```bash
export REPO=/path/to/autonomy          # 改为本机仓库路径
export BRIDGE=127.0.0.1:5005
export PROTO_OPTS="-import-path $REPO -proto autonomy/bridge/proto/external_command_service.proto"
export SVC=autonomy.bridge.proto.AutonomyService
new_cmd_id() { echo "test-$(date +%s)-$RANDOM"; }

# 推荐：JSON 用 heredoc + -d @-，避免 shell 转义导致可读性下降
grpc_call() {
  grpcurl -plaintext $PROTO_OPTS -d @- "$BRIDGE" "$SVC/$1"
}
```

**安装 grpcurl**：`brew install grpcurl` 或 `go install github.com/fullstorydev/grpcurl/cmd/grpcurl@latest`

## 1.2 服务发现

确认 Bridge 可达、Proto 路径正确（无需机载任务）。

| 命令 | 判定 |
|------|------|
| `list` | 输出含 `autonomy.bridge.proto.AutonomyService` |
| `list $SVC` | 13 个 RPC 方法 |
| `describe $SVC.Method` | 请求/响应类型签名 |
| `describe autonomy.bridge.proto.Message` | 消息字段定义 |

```bash
grpcurl -plaintext $PROTO_OPTS $BRIDGE list
grpcurl -plaintext $PROTO_OPTS $BRIDGE list $SVC
grpcurl -plaintext $PROTO_OPTS $BRIDGE describe $SVC.GetCapabilities
grpcurl -plaintext $PROTO_OPTS $BRIDGE describe autonomy.bridge.proto.NavigationCommandRequest
```

## 1.3 快速冒烟（5 步）

Query → Nav STOP → EmergencyStop → Stream。响应 JSON → [§1.4](#14-响应-json-参考)；自动化脚本 → [13 §13.13](13_integration_tests.md#1313-一键冒烟序列bash)。

| # | RPC | 判定 |
|---|-----|------|
| 1 | `GetCapabilities` | `bridgeVersion` 非空 |
| 2 | `GetActiveTask` | `type=NONE`, `status=IDLE` |
| 3 | `SendNavigationCommand` STOP(2) | 末帧 `NAV_STATUS_IDLE` |
| 4 | `EmergencyStop` | `success=true` |
| 5 | `ReceiveBotStates` | 持续输出 JSON，Ctrl+C 结束 |

```bash
# 1 GetCapabilities
grpcurl -plaintext $PROTO_OPTS -d '{}' $BRIDGE $SVC/GetCapabilities

# 2 GetActiveTask
grpcurl -plaintext $PROTO_OPTS -d '{}' $BRIDGE $SVC/GetActiveTask

# 3 Nav STOP
export CMD_ID=$(new_cmd_id)
grpcurl -plaintext $PROTO_OPTS -d @- $BRIDGE $SVC/SendNavigationCommand <<EOF
{ "header": { "cmd_id": "${CMD_ID}" }, "command": 2 }
EOF

# 4 EmergencyStop
export CMD_ID=$(new_cmd_id)
grpcurl -plaintext $PROTO_OPTS -d @- $BRIDGE $SVC/EmergencyStop <<EOF
{ "header": { "cmd_id": "${CMD_ID}" }, "reason": "smoke" }
EOF

# 5 ReceiveBotStates（Ctrl+C 结束）
grpcurl -plaintext $PROTO_OPTS -d '{}' $BRIDGE $SVC/ReceiveBotStates
```

## 1.4 响应 JSON 参考

### GetCapabilities

```json
{
  "plannerIds": ["navfn_planner"],
  "controllerIds": ["graceful_controller"],
  "supportsNavigation": true,
  "supportsMapManagement": true,
  "bridgeVersion": "0.1.0",
  "autonomyVersion": "0.1.0"
}
```

### GetActiveTask（空闲）

```json
{ "type": "TASK_TYPE_NONE", "status": "TASK_STATUS_IDLE" }
```

### Command Stream — 进度帧

```json
{
  "ack": {
    "success": true,
    "final": false,
    "cmdId": "test-1710000001-1234",
    "taskType": "TASK_TYPE_NAVIGATION",
    "taskStatus": "TASK_STATUS_RUNNING"
  },
  "status": "NAV_STATUS_NAVIGATING",
  "distanceRemaining": 12.5,
  "currentWaypointIndex": 0,
  "totalWaypoints": 1
}
```

### Command Stream — 成功末帧

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

### Command Stream — 失败末帧

```json
{
  "ack": {
    "success": false,
    "final": true,
    "errorCode": "PLANNING_NO_PATH_FOUND",
    "message": "No path found",
    "taskStatus": "TASK_STATUS_FAILED"
  },
  "status": "NAV_STATUS_FAILED"
}
```

### System — EmergencyStop

```json
{
  "success": true,
  "message": "Emergency stop acknowledged",
  "errorCode": "OK",
  "taskType": "TASK_TYPE_NONE",
  "taskStatus": "TASK_STATUS_IDLE"
}
```

任务结束判据：**Command Stream 末帧 `ack.final=true`**。失败时 `ack.success=false`，查 `errorCode`（[03 §3.6](03_common_types.md#36-错误码)）。

## 1.5 客户端开发（Python）

Stub 调用与 Bidi Teleop 示例 → [13 §13.14](13_integration_tests.md#1314-python-集成测试脚本)。

## 1.6 代码生成

```bash
pip install grpcio-tools
python -m grpc_tools.protoc -I$REPO \
  --python_out=./gen --grpc_python_out=./gen \
  autonomy/bridge/proto/external_command_service.proto \
  autonomy/commsgs/proto/builtin_interfaces.proto \
  autonomy/commsgs/proto/geometry_msgs.proto \
  autonomy/commsgs/proto/vehicle_msgs.proto \
  autonomy/commsgs/proto/error_code.proto \
  autonomy/commsgs/proto/std_msgs.proto
```
