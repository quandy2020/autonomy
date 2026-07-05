(rpc-stream-api)=
# 推送接口 Stream

长连接 Server Stream，周期推送机器人状态与事件。适合**系统测试持续观测**与**算法联调闭环**。

## 5.1 方法列表

| RPC | 响应类型 | 说明 |
|-----|----------|------|
| `ReceiveBotStates` | `vehicle_msgs.RobotState` | 周期状态，10～50 Hz |
| `ReceiveBotEvents` | `vehicle_msgs.RobotEvent` | 事件驱动 |

## 5.2 grpcurl

完整用例 → [13 §13.4](13_integration_tests.md#134-stream-测试用例)。

**ReceiveBotStates**

```bash
# ReceiveBotStates — 周期状态流（Empty→Stream · Ctrl+C 结束）
grpcurl -plaintext $PROTO_OPTS \
  -d '{}' \
  $BRIDGE \
  $SVC/ReceiveBotStates
```


**ReceiveBotEvents**

```bash
# ReceiveBotEvents — 任务/异常事件流（Empty→Stream · Ctrl+C 结束）
grpcurl -plaintext $PROTO_OPTS \
  -d '{}' \
  $BRIDGE \
  $SVC/ReceiveBotEvents
```



## 5.3 vehicle_msgs

定义：`autonomy/commsgs/proto/vehicle_msgs.proto`

**RobotState**（关键字段）

| 字段 | 说明 |
|------|------|
| `pose` / `twist` | 位姿与速度 |
| `battery_percent` | 电量 |
| `active_task_type` / `active_task_status` | 与 Command 对齐 |
| `active_cmd_id` | 当前命令 ID |
| `is_docked` / `is_charging` | docking 状态 |

**RobotEvent**：`type` · `severity` · `task_type` · `cmd_id` · `message` · `error_code`

`TaskType` 数值见 [03 §3.3](03_common_types.md#33-tasktype-taskstatus)。

MQTT 对应 Topic 见 [mqtt/04 §4.2](../mqtt/04_topic_protocol.md#42-载荷约定)。
