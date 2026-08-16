# rpc-cli.py — automsgs RPC 测试手册

用 [`grpcurl`](https://github.com/fullstorydev/grpcurl) 调用 `automsgs/proto/rpcs` 全部域 Service。本文说明每个服务的全名、请求/响应字段，以及可逐条执行的测试命令。

## 1. 准备

```bash
# 依赖：Python 3.9+、grpcurl 在 PATH
cd automsgs/tools/cli
chmod +x rpc-cli.py

export AUTOMSGS_RPC_TARGET=localhost:50051   # 按实际改
# 可选：export AUTOMSGS_PROTO_INCLUDE=/path/to/build/.../proto_include
```

| 全局选项 | 含义 |
|----------|------|
| `-t` / `--target` | `host:port` |
| `--tls` | 启用 TLS（默认 plaintext） |
| `-v` | 打印底层 grpcurl 命令 |
| `--import-path` | proto include 根目录 |
| `--max-time SEC` | 流式 RPC 超时（秒） |

| 子命令 | 作用 |
|--------|------|
| `list [Service]` | 本地列出全部/某服务方法（无需服务器） |
| `describe <符号>` | 查看 proto 定义 |
| `call <Method> -d JSON` | 调用；`-d @file.json` 从文件读 |

Method 写法：`SystemService/Heartbeat` 或全名  
`automsgs.rpcs.system.SystemService/Heartbeat`。  
**重名方法**（`GetStatus` / `Cancel`）必须带 `Service/` 前缀。

公共响应头：几乎所有响应含 `status.code` / `status.message`（`automsgs.rpcs.common.Status`；**`code=OK(0)` 为成功**；码表见 `status_msgs.StatusCode`）。

常用嵌套类型（JSON）：

```json
"header": { "stamp": { "sec": 0, "nanosec": 0 }, "frame_id": "map" }
"pose": {
  "position": { "x": 0.0, "y": 0.0, "z": 0.0 },
  "orientation": { "x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0 }
}
```

---

## 2. 先验证 CLI 本身（无需服务器）

```bash
./rpc-cli.py list
./rpc-cli.py list SystemService
./rpc-cli.py list NavigationService
./rpc-cli.py describe SystemService/Heartbeat
./rpc-cli.py describe automsgs.rpcs.navigation.NavigateRequest
./rpc-cli.py list --grpcurl    # 需已安装 grpcurl；校验 proto 加载
```

期望：列出全部 Service（含 `TeleopService` / 扩展后的 `SystemService`）。

---

## 3. SystemService

| 项 | 值 |
|----|-----|
| Package | `automsgs.rpcs.system` |
| 全名 | `automsgs.rpcs.system.SystemService` |
| Proto | `system.proto` |

### 3.1 Heartbeat — 保活

**请求 `HeartbeatRequest`**

| 字段 | 类型 | 说明 |
|------|------|------|
| `sequence` | uint64 | 客户端序号，原样回显 |

**响应 `HeartbeatResponse`**：`status`、`sequence`、`robot_time_ns`（机器人时间，纳秒）

```bash
./rpc-cli.py call SystemService/Heartbeat -d '{"sequence":1}'
./rpc-cli.py call SystemService/Heartbeat -d '{"sequence":2}' -v
```

### 3.2 GetInfo — 身份与版本

**请求**：空 `{}`  
**响应**：`model`、`serial_number`、`firmware_version`、`software_version`、`hostname`

```bash
./rpc-cli.py call SystemService/GetInfo -d '{}'
```

### 3.3 GetStatus — 总状态 + 电量

**请求**：空 `{}`  
**响应**

| 字段 | 说明 |
|------|------|
| `state` | `SYSTEM_STATE_IDLE` / `BUSY` / `ERROR` / `ESTOP` |
| `detail` | 文字说明 |
| `battery` | `sensor_msgs.BatteryState` |

```bash
./rpc-cli.py call SystemService/GetStatus -d '{}'
```

### 3.4 GetHealth — 系统健康（对齐 autonomy/system/monitor）

**响应 `SystemHealth`**

| 字段 | 对齐 |
|------|------|
| `hazard_level` | `HazardMonitor`：OK/WARN/ERROR |
| `mrm_active` | `MrmHandler` 是否发零速 |
| `emergency_stop_latched` | 急停闩锁 |
| `host` | CPU/Mem/Disk/Load/NTP |
| `channels` / `latencies` | Channel/Latency watches |

```bash
./rpc-cli.py call SystemService/GetHealth -d '{}'
```

### 3.5 EmergencyStop — 急停（慎用）

| 字段 | 说明 |
|------|------|
| `reason` | 可选原因字符串 |

**响应**：`Status`

```bash
./rpc-cli.py call SystemService/EmergencyStop -d '{"reason":"rpc-cli smoke"}'
./rpc-cli.py call SystemService/GetStatus -d '{}'
```

### 3.6 ClearEmergencyStop — 解除急停

| 字段 | 说明 |
|------|------|
| `reason` | 可选 |
| `confirmation_token` | 部署若要求确认令牌则填写 |

```bash
./rpc-cli.py call SystemService/ClearEmergencyStop -d '{
  "reason":"resume",
  "confirmation_token":""
}'
./rpc-cli.py call SystemService/GetStatus -d '{}'
```

### 3.7 CancelAllGoals — 取消活动域目标

| 字段 | 说明 |
|------|------|
| `goal_kinds` | 空 = 全部；元素如 `GOAL_KIND_NAVIGATION` / `FOLLOW` / `CHARGE` / `MAPPING` / `TELEOP` / `EXPLORATION` |
| `reason` | 可选 |

```bash
./rpc-cli.py call SystemService/CancelAllGoals -d '{"goal_kinds":[],"reason":"smoke"}'
./rpc-cli.py call SystemService/CancelAllGoals -d '{
  "goal_kinds": ["GOAL_KIND_NAVIGATION", "GOAL_KIND_FOLLOW"],
  "reason": "stop motion"
}'
```

### 3.8 GetActiveGoal

**响应 `ActiveGoal`**：`kind`、`goal_id`、`detail`

```bash
./rpc-cli.py call SystemService/GetActiveGoal -d '{}'
```

### 3.9 GetCapabilities

```bash
./rpc-cli.py call SystemService/GetCapabilities -d '{}'
```

---

## 4. NavigationService

| 项 | 值 |
|----|-----|
| Package | `automsgs.rpcs.navigation` |
| 全名 | `automsgs.rpcs.navigation.NavigationService` |
| Proto | `navigation.proto` |

**状态 `NavigationState`**：`IDLE` / `PLANNING` / `RUNNING` / `PAUSED` / `ARRIVED` / `FAILED` / `CANCELLED`  
**关流**：`FAILED` / `CANCELLED` / Cancel / 拒收；**`ARRIVED` 不关流**。

### 4.1 Navigate — 流式导航

**请求 `NavigateRequest`**

| 字段 | 类型 | 说明 |
|------|------|------|
| `header` | Header | 本 RPC 时间/默认坐标系 |
| `goal_id` | string | 可选客户端 ID |
| `waypoints` | PoseStamped[] | **非空**；1=单点，N=途经点 |
| `options.maximum_linear_speed` | optional float | m/s |
| `options.maximum_angular_speed` | optional float | rad/s |
| `options.goal_position_tolerance_meters` | optional float | 到位位置容差 |
| `options.goal_heading_tolerance_radians` | optional float | 到位朝向容差 |
| `options.timeout_seconds` | optional float | 总超时；0/缺省=实现默认 |

**流元素 `NavigateResponse`**：`status`、`state`、`goal_id`、`current_pose`、  
`remaining_distance_meters`、`estimated_time_remaining_seconds`、`waypoint_index`、`number_of_waypoints`

```bash
./rpc-cli.py call NavigationService/Navigate -d '{
  "header": {"frame_id": "map"},
  "goal_id": "nav-demo-1",
  "waypoints": [
    {
      "header": {"frame_id": "map"},
      "pose": {
        "position": {"x": 1.0, "y": 0.0, "z": 0.0},
        "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
      }
    },
    {
      "header": {"frame_id": "map"},
      "pose": {
        "position": {"x": 2.0, "y": 1.0, "z": 0.0},
        "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
      }
    }
  ],
  "options": {
    "maximum_linear_speed": 0.5,
    "maximum_angular_speed": 0.8,
    "goal_position_tolerance_meters": 0.1,
    "goal_heading_tolerance_radians": 0.17,
    "timeout_seconds": 120.0
  }
}' --max-time 15
```

空航点（应失败）：

```bash
./rpc-cli.py call NavigationService/Navigate -d '{"waypoints":[]}' --max-time 5
```

### 4.2 Pause / Resume / Replan / Cancel

共用 **`GoalRequest`**：`goal_id` 空 = 当前目标。响应均为 `Status`。

```bash
./rpc-cli.py call NavigationService/GetStatus -d '{}'
./rpc-cli.py call NavigationService/Pause -d '{"goal_id":""}'
./rpc-cli.py call NavigationService/GetStatus -d '{}'
./rpc-cli.py call NavigationService/Resume -d '{"goal_id":""}'
./rpc-cli.py call NavigationService/Replan -d '{"goal_id":"nav-demo-1"}'
./rpc-cli.py call NavigationService/Cancel -d '{"goal_id":""}'
./rpc-cli.py call NavigationService/GetStatus -d '{}'
```

### 4.3 GetStatus

**请求**：空。**响应**：同 `NavigateResponse`。

```bash
./rpc-cli.py call NavigationService/GetStatus -d '{}'
```

---

## 5. MapService

| 项 | 值 |
|----|-----|
| Package | `automsgs.rpcs.mapping` |
| 全名 | `automsgs.rpcs.mapping.MapService` |
| Proto | `mapping.proto` |

**状态 `MappingState`**：`UNKNOWN` / `IDLE` / `MAPPING` / `SAVING` / `FAILED` / `CANCELLED`  
优先 `map_identifier`；`map_name` 为别名。忙 → `CODE_MAPPING_BUSY`（1100）。

### 5.1 StartMapping — 开始建图会话

| 字段 | 说明 |
|------|------|
| `header` | 可选 |
| `map_name` | 目标地图名 |
| `goal_id` | 可选客户端会话 ID |

**响应**：`Status`

```bash
./rpc-cli.py call MapService/StartMapping -d '{
  "header": {"frame_id": "map"},
  "map_name": "floor1",
  "goal_id": "map-1"
}'
```

### 5.2 GetMappingStatus → `MappingStatus`

**请求**：空。**响应**：`state`、`goal_id`、`map_identifier`、`map_name`、`detail`、`optional progress`

```bash
./rpc-cli.py call MapService/GetMappingStatus -d '{}'
```

### 5.3 FinishMapping — 结束并可选落盘

| 字段 | 说明 |
|------|------|
| `goal_id` | 空 = 当前会话 |
| `persist` | `true` = 结束前持久化 |

**响应**：`status`、`map_identifier`

```bash
./rpc-cli.py call MapService/FinishMapping -d '{"goal_id":"","persist":true}'
```

### 5.4 CancelMapping — 中止（不要求落盘）

```bash
./rpc-cli.py call MapService/CancelMapping -d '{"goal_id":""}'
```

### 5.5 ListMaps

| 字段 | 说明 |
|------|------|
| `header` | 可选 |

**响应 `maps[]`**：`map_identifier`、`name`、`resolution`、`width`、`height`、`is_current`

```bash
./rpc-cli.py call MapService/ListMaps -d '{}'
```

### 5.6 GetMap / GetMapMetadata

| 字段 | 说明 |
|------|------|
| `map_identifier` | 优先 |
| `map_name` | identifier 为空时的别名 |

```bash
./rpc-cli.py call MapService/GetMapMetadata -d '{"map_name":"floor1"}'
./rpc-cli.py call MapService/GetMap -d '{"map_identifier":"","map_name":"floor1"}'
```

### 5.7 SaveMap

| 字段 | 说明 |
|------|------|
| `header` | 可选 |
| `map_identifier` / `map_name` | 标识 |
| `map` | `OccupancyGrid`；空 = 保存当前 SLAM 缓冲 |

**响应**：`status`、`map_identifier`

```bash
./rpc-cli.py call MapService/SaveMap -d '{"map_name":"floor1","map_identifier":""}'
```

### 5.8 SetCurrentMap / DeleteMap

```bash
./rpc-cli.py call MapService/SetCurrentMap -d '{"map_identifier":"floor1"}'
./rpc-cli.py call MapService/DeleteMap -d '{"map_identifier":"floor1"}'   # 破坏性，慎用
```

---

## 6. LocalizationService

| 项 | 值 |
|----|-----|
| Package | `automsgs.rpcs.localization` |
| 全名 | `automsgs.rpcs.localization.LocalizationService` |
| Proto | `localization.proto` |

**状态 `LocalizationState`**：`UNKNOWN` / `LOCALIZED` / `LOST` / `INITIALIZING`  
**Code**：800–802；`CODE_LOCALIZATION_BUSY = 803`（INITIALIZING 时再设初值）

`PoseWithCovariance` = `PoseStamped`（`header` + `pose`）+ `covariance[36]`。

### 6.1 GetPose

| 字段 | 说明 |
|------|------|
| `map_frame_id` | 空 = 默认 map 系 |

**响应**：`status`、`pose`（含 header/位姿/协方差）、`optional confidence`（[0,1]）

```bash
./rpc-cli.py call LocalizationService/GetPose -d '{}'
./rpc-cli.py call LocalizationService/GetPose -d '{"map_frame_id":"map"}'
```

### 6.2 GetStatus → `LocalizationStatus`

**请求**：空。**响应**：`state`、`map_id`、`detail`、`optional confidence`

```bash
./rpc-cli.py call LocalizationService/GetStatus -d '{}'
```

### 6.3 SetInitialPose — 重定位

| 字段 | 说明 |
|------|------|
| `pose.pose.header` | 坐标系（如 `map`）与时间戳 |
| `pose.pose.pose` | 位置 + 四元数 |
| `pose.covariance` | 6×6 行主序，长度 0 或 36；未知可省略/全 0 |
| `map_id` | 空 = MapService 当前图 |

**响应**：`Status`

```bash
./rpc-cli.py call LocalizationService/SetInitialPose -d '{
  "pose": {
    "pose": {
      "header": {"frame_id": "map"},
      "pose": {
        "position": {"x": 0.0, "y": 0.0, "z": 0.0},
        "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
      }
    },
    "covariance": [
      0.25, 0, 0, 0, 0, 0,
      0, 0.25, 0, 0, 0, 0,
      0, 0, 0.01, 0, 0, 0,
      0, 0, 0, 0.01, 0, 0,
      0, 0, 0, 0, 0.01, 0,
      0, 0, 0, 0, 0, 0.07
    ]
  },
  "map_id": ""
}'
./rpc-cli.py call LocalizationService/GetStatus -d '{}'
./rpc-cli.py call LocalizationService/GetPose -d '{"map_frame_id":"map"}'
```

---

## 7. ChargeService

| 项 | 值 |
|----|-----|
| Package | `automsgs.rpcs.charge` |
| 全名 | `automsgs.rpcs.charge.ChargeService` |
| Proto | `charge.proto` |

**状态 `ChargeState`**：`IDLE` / `RETURNING` / `LEAVING` / `DOCKED_*` / `CHARGING` / `FULL` / `PAUSED`（**不关流**）/ `FAILED` / `CANCELLED`  
形态对齐 Navigation：`Return`/`Leave` 流；`Pause`/`Resume`/`Cancel`/`GetStatus` unary。

### 7.1 Return — 回充（流）

| 字段 | 说明 |
|------|------|
| `header` | 可选 |
| `station_id` | 空 = 默认充电站 |
| `goal_id` | 可选客户端 ID |

**流 / 快照 `ChargeResponse`**：`status`、`state`、`progress`、`goal_id`、`station_id`、`battery_pct`、`active`、`message`

```bash
./rpc-cli.py call ChargeService/Return -d '{
  "header": {"frame_id": "map"},
  "station_id": "",
  "goal_id": "charge-1"
}' --max-time 20
```

### 7.2 Leave — 离桩（流）

```bash
./rpc-cli.py call ChargeService/Leave -d '{"goal_id":"leave-1"}' --max-time 20
```

### 7.3 Pause / Resume / Cancel / GetStatus

```bash
./rpc-cli.py call ChargeService/GetStatus -d '{}'
./rpc-cli.py call ChargeService/Pause -d '{"goal_id":""}'
./rpc-cli.py call ChargeService/Resume -d '{"goal_id":""}'
./rpc-cli.py call ChargeService/Cancel -d '{"goal_id":""}'
```

---

## 8. FollowService

| 项 | 值 |
|----|-----|
| Package | `automsgs.rpcs.follow` |
| 全名 | `automsgs.rpcs.follow.FollowService` |
| Proto | `follow.proto` |

**目标类型 `FollowTargetType`**：`UNKNOWN` / `PERSON` / `OBJECT` / `VEHICLE` / `CUSTOM`  
**状态**：`ACQUIRING` / `FOLLOWING` / `LOST_TARGET`（**不关流**）/ `PAUSED`（**不关流**）/ `FAILED` / `CANCELLED`  
无 `FollowPhase`；`Pause`/`Resume`/`Cancel`/`GetStatus` 对齐 Navigation。

### 8.1 Follow（跟随）

| 字段 | 说明 |
|------|------|
| `header` | 可选 |
| `goal_id` | 可选 |
| `target_type` | 枚举 |
| `target_id` | 空 = 实现自选 |
| `hint_pose` | 可选锁定提示 |
| `options.desired_distance_m` | 跟随距离 |

**流 / 快照 `FollowResponse`**：`state`、`goal_id`、`target_type`、`target_id`、`distance_m`、`active`、`message`

```bash
./rpc-cli.py call FollowService/Follow -d '{
  "header": {"frame_id": "base_link"},
  "target_type": "FOLLOW_TARGET_TYPE_PERSON",
  "target_id": "",
  "goal_id": "follow-1",
  "options": {"desired_distance_m": 1.2}
}' --max-time 15

./rpc-cli.py call FollowService/Follow -d '{
  "target_type": "FOLLOW_TARGET_TYPE_OBJECT",
  "target_id": "obj-42",
  "hint_pose": {
    "position": {"x": 1.0, "y": 0.0, "z": 0.0},
    "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
  },
  "options": {"desired_distance_m": 0.8}
}' --max-time 10
```

### 8.2 Pause / Resume / Cancel / GetStatus

```bash
./rpc-cli.py call FollowService/GetStatus -d '{}'
./rpc-cli.py call FollowService/Pause -d '{"goal_id":""}'
./rpc-cli.py call FollowService/Resume -d '{"goal_id":""}'
./rpc-cli.py call FollowService/Cancel -d '{"goal_id":""}'
```

---

## 9. SensorService

| 项 | 值 |
|----|-----|
| Package | `automsgs.rpcs.sensor` |
| 全名 | `automsgs.rpcs.sensor.SensorService` |
| Proto | `sensor.proto` |

**传感器类型**：`CAMERA` / `DEPTH_CAMERA` / `LASER_SCAN` / `POINT_CLOUD` / `IMU`  
**录制状态**：`RECORDING` / `UPLOADING` / `SUCCEEDED` / `FAILED` / `CANCELLED`  
栅格走 `MapService`，不在本服务。

### 9.1 ListSensors

| 字段 | 说明 |
|------|------|
| `header` | 可选 |

**响应 `sensors[]`**：`sensor_id`、`frame_id`、`type`、`topic`、`parameter_names[]`

```bash
./rpc-cli.py call SensorService/ListSensors -d '{}'
```

记下返回的 `sensor_id`，替换下文 `camera_front`。

### 9.2 GetSample — 最新一帧

| 字段 | 说明 |
|------|------|
| `sensor_id` | 必填 |
| `prefer_compressed` | 相机优先压缩图 |

**响应 `oneof data`**：`image` / `compressed_image` / `laser_scan` / `point_cloud` / `imu`

```bash
./rpc-cli.py call SensorService/GetSample -d '{
  "sensor_id": "camera_front",
  "prefer_compressed": true
}'
./rpc-cli.py call SensorService/GetSample -d '{"sensor_id":"lidar_top","prefer_compressed":false}'
```

### 9.3 GetParameters / SetParameters

| Get 字段 | 说明 |
|----------|------|
| `sensor_id` | 传感器 |
| `names` | 空数组 = 全部参数 |

| Set 字段 | 说明 |
|----------|------|
| `sensor_id` | 传感器 |
| `parameters[]` | `{name, value}`，value 为字符串（如 `"15"`、`"true"`） |

常用约定 key：`enabled`、`frame_rate_hz`、`exposure`、`range_max`

```bash
./rpc-cli.py call SensorService/GetParameters -d '{
  "sensor_id": "camera_front",
  "names": []
}'
./rpc-cli.py call SensorService/GetParameters -d '{
  "sensor_id": "camera_front",
  "names": ["enabled", "frame_rate_hz"]
}'
./rpc-cli.py call SensorService/SetParameters -d '{
  "sensor_id": "camera_front",
  "parameters": [
    {"name": "frame_rate_hz", "value": "15"},
    {"name": "enabled", "value": "true"}
  ]
}'
```

### 9.4 SaveParameters / LoadParameters

| 字段 | 说明 |
|------|------|
| `sensor_id` | 空 = 全部传感器配置 |

```bash
./rpc-cli.py call SensorService/SaveParameters -d '{"sensor_id":"camera_front"}'
./rpc-cli.py call SensorService/LoadParameters -d '{"sensor_id":"camera_front"}'
./rpc-cli.py call SensorService/SaveParameters -d '{"sensor_id":""}'
./rpc-cli.py call SensorService/LoadParameters -d '{}'
```

### 9.5 Record（流）/ CancelRecord / GetRecordStatus

**RecordRequest**

| 字段 | 说明 |
|------|------|
| `header` | 可选 |
| `record_id` | 可选客户端 ID |
| `sensor_ids` | 空 = 实现默认集合 |
| `uri` | 本地路径或远程 URI |
| `duration_seconds` | optional；时长上限 |
| `maximum_bytes` | optional；字节上限 |

**RecordResponse**：`state`、`record_id`、`uri`、`bytes_transferred`、`progress`、`active`、`final`  
（`final=true` 关流）

```bash
./rpc-cli.py call SensorService/Record -d '{
  "record_id": "rec-1",
  "sensor_ids": ["camera_front"],
  "uri": "/tmp/automsgs_record",
  "duration_seconds": 5.0,
  "maximum_bytes": 10485760
}' --max-time 20

./rpc-cli.py call SensorService/GetRecordStatus -d '{}'
./rpc-cli.py call SensorService/CancelRecord -d '{"record_id":""}'
./rpc-cli.py call SensorService/GetRecordStatus -d '{}'
```

---

## 10. TeleopService

| 项 | 值 |
|----|-----|
| Package | `automsgs.rpcs.teleop` |
| 全名 | `automsgs.rpcs.teleop.TeleopService` |
| Proto | `teleop.proto` |

**状态 `TeleopState`**：`IDLE` / `ACTIVE` / `PAUSED`（相对，**不关流**）/ `TIMEOUT` / `REJECTED` / `SUCCEEDED` / `CANCELLED` / `FAILED`  
同时最多一个 teleop 目标。`VelocityOptions` / `RelativeOptions` 已拆分。

**`TeleopResponse`**：`current_pose`、`commanded_twist`、`actual_twist`、剩余距离/偏航。

### 10.1 Velocity（bidi）

```bash
./rpc-cli.py call TeleopService/Velocity -d '{
  "command": "VELOCITY_COMMAND_START",
  "goal_id": "teleop-1",
  "options": {"maximum_linear_speed": 0.5, "watchdog_timeout_seconds": 0.5}
}' --max-time 3

./rpc-cli.py call TeleopService/Velocity -d '{
  "header": {"frame_id": "base_link"},
  "command": "VELOCITY_COMMAND_TWIST",
  "goal_id": "teleop-1",
  "twist": {
    "header": {"frame_id": "base_link"},
    "twist": {"linear": {"x": 0.2}, "angular": {"z": 0.1}}
  }
}' --max-time 2

./rpc-cli.py call TeleopService/Velocity -d '{
  "command": "VELOCITY_COMMAND_STOP", "goal_id": "teleop-1"
}' --max-time 2
```

### 10.2 DriveOnHeading / BackUp / Spin

```bash
./rpc-cli.py call TeleopService/DriveOnHeading -d '{
  "goal_id": "doh-1", "distance_meters": 0.5,
  "options": {"maximum_linear_speed": 0.3, "timeout_seconds": 30,
              "position_tolerance_meters": 0.05}
}' --max-time 15

./rpc-cli.py call TeleopService/BackUp -d '{
  "goal_id": "bu-1", "distance_meters": 0.3,
  "options": {"maximum_linear_speed": 0.2, "timeout_seconds": 20}
}' --max-time 15

./rpc-cli.py call TeleopService/Spin -d '{
  "goal_id": "spin-1", "target_yaw_radians": 1.5708,
  "options": {"maximum_angular_speed": 0.5, "yaw_tolerance_radians": 0.05}
}' --max-time 15
```

### 10.3 Pause / Resume / Cancel / GetStatus

```bash
./rpc-cli.py call TeleopService/GetStatus -d '{}'
./rpc-cli.py call TeleopService/Pause -d '{"goal_id":""}'
./rpc-cli.py call TeleopService/Resume -d '{"goal_id":""}'
./rpc-cli.py call TeleopService/Cancel -d '{"goal_id":""}'
```

Code：`BUSY`(1200) / `REJECTED`(1201) / `TIMEOUT`(1202) / `CANCELLED`(1203) / `COLLISION`(1204) / `NOT_READY`(1205)。

---

## 11. ExplorationService

| 项 | 值 |
|----|-----|
| Package | `automsgs.rpcs.exploration` |
| 全名 | `automsgs.rpcs.exploration.ExplorationService` |
| Proto | `exploration.proto` |

**形态对齐 Navigation：** `Explore` server-stream；`Pause`/`Resume`/`Cancel`/`GetStatus` unary；另加 `SetArea` / `SaveMap`（方案 B）。

**状态 `ExplorationState`**：`IDLE` / `PLANNING` / `EXPLORING` / `PAUSED`（**不关流**）/ `COMPLETED` / `FAILED` / `CANCELLED`

### 11.1 Explore（流）

| 字段 | 说明 |
|------|------|
| `header` | 可选 |
| `goal_id` | 可选 |
| `area` | 探索多边形；空 = 实现默认 |
| `map_name` | 存图偏好名 |
| `options` | 限速 / 超时 / `coverage_target` / `enable_mapping` |

**流 / 快照 `ExploreResponse`**：`state`、`current_pose`、`explored_area_m2`、`progress`、`frontier_count`、`map_name`、`detail`

```bash
./rpc-cli.py call ExplorationService/Explore -d '{
  "goal_id": "explore-1",
  "map_name": "office",
  "area": {
    "points": [
      {"x": 0, "y": 0, "z": 0},
      {"x": 10, "y": 0, "z": 0},
      {"x": 10, "y": 8, "z": 0},
      {"x": 0, "y": 8, "z": 0}
    ]
  },
  "options": {"coverage_target": 0.9, "timeout_seconds": 600}
}' --max-time 30
```

### 11.2 Pause / Resume / Cancel / GetStatus

```bash
./rpc-cli.py call ExplorationService/GetStatus -d '{}'
./rpc-cli.py call ExplorationService/Pause -d '{"goal_id":""}'
./rpc-cli.py call ExplorationService/Resume -d '{"goal_id":""}'
./rpc-cli.py call ExplorationService/Cancel -d '{"goal_id":""}'
```

### 11.3 SetArea / SaveMap

```bash
./rpc-cli.py call ExplorationService/SetArea -d '{
  "goal_id": "",
  "area": {"points": [{"x":0,"y":0,"z":0},{"x":5,"y":0,"z":0},{"x":5,"y":5,"z":0}]}
}'
./rpc-cli.py call ExplorationService/SaveMap -d '{"goal_id":"","map_name":"office"}'
```

Code：`BUSY`(1300) / `CANCELLED`(1301) / `FAILED`(1302) / `NO_AREA`(1303) / `SAVE_FAILED`(1304) / `NOT_RUNNING`(1305)。

---

## 12. 推荐联调顺序（尽量覆盖全部功能）

在**测试机**上按序执行；流式命令用短 `--max-time`，急停/删图最后再测。

```bash
CLI=./rpc-cli.py
# 0) 目录
$CLI list

# 1) System（先读后写）
$CLI call SystemService/Heartbeat -d '{"sequence":1}'
$CLI call SystemService/GetInfo -d '{}'
$CLI call SystemService/GetStatus -d '{}'
$CLI call SystemService/GetHealth -d '{}'
$CLI call SystemService/GetCapabilities -d '{}'
$CLI call SystemService/GetActiveGoal -d '{}'
$CLI call SystemService/CancelAllGoals -d '{"goal_kinds":[],"reason":"smoke reset"}'

# 2) Localization
$CLI call LocalizationService/GetStatus -d '{}'
$CLI call LocalizationService/GetPose -d '{"map_frame_id":"map"}'
$CLI call LocalizationService/SetInitialPose -d '{
  "pose":{
    "pose":{
      "header":{"frame_id":"map"},
      "pose":{"position":{"x":0,"y":0,"z":0},"orientation":{"x":0,"y":0,"z":0,"w":1}}
    },
    "covariance":[0.25,0,0,0,0,0, 0,0.25,0,0,0,0, 0,0,0.01,0,0,0, 0,0,0,0.01,0,0, 0,0,0,0,0.01,0, 0,0,0,0,0,0.07]
  },
  "map_id":""
}'

# 3) Sensor 配置与采样
$CLI call SensorService/ListSensors -d '{}'
$CLI call SensorService/GetSample -d '{"sensor_id":"camera_front","prefer_compressed":true}'
$CLI call SensorService/GetParameters -d '{"sensor_id":"camera_front","names":[]}'
$CLI call SensorService/SetParameters -d '{
  "sensor_id":"camera_front",
  "parameters":[{"name":"frame_rate_hz","value":"10"}]
}'
$CLI call SensorService/SaveParameters -d '{"sensor_id":"camera_front"}'
$CLI call SensorService/LoadParameters -d '{"sensor_id":"camera_front"}'
$CLI call SensorService/Record -d '{
  "record_id":"rec-smoke","sensor_ids":["camera_front"],
  "uri":"/tmp/automsgs_record","duration_seconds":3.0
}' --max-time 10
$CLI call SensorService/GetRecordStatus -d '{}'
$CLI call SensorService/CancelRecord -d '{}'

# 4) Mapping
$CLI call MapService/ListMaps -d '{}'
$CLI call MapService/StartMapping -d '{"map_name":"smoke","goal_id":"m1"}'
$CLI call MapService/GetMappingStatus -d '{}'
$CLI call MapService/FinishMapping -d '{"goal_id":"","persist":true}'
$CLI call MapService/GetMapMetadata -d '{"map_name":"smoke"}'
$CLI call MapService/GetMap -d '{"map_name":"smoke"}'
$CLI call MapService/SaveMap -d '{"map_name":"smoke"}'
$CLI call MapService/SetCurrentMap -d '{"map_identifier":"smoke"}'

# 5) Navigation（短流）
$CLI call NavigationService/Navigate -d '{
  "goal_id":"n1",
  "waypoints":[{
    "header":{"frame_id":"map"},
    "pose":{"position":{"x":0.5,"y":0,"z":0},"orientation":{"x":0,"y":0,"z":0,"w":1}}
  }],
  "options":{"timeout_seconds":30.0}
}' --max-time 8
$CLI call NavigationService/GetStatus -d '{}'
$CLI call NavigationService/Pause -d '{}'
$CLI call NavigationService/Resume -d '{}'
$CLI call NavigationService/Replan -d '{}'
$CLI call NavigationService/Cancel -d '{}'

# 6) Follow / Dock（短流）
$CLI call FollowService/Follow -d '{
  "target_type":"FOLLOW_TARGET_TYPE_PERSON","goal_id":"f1",
  "options":{"desired_distance_m":1.0}
}' --max-time 8
$CLI call FollowService/GetStatus -d '{}'
$CLI call FollowService/Pause -d '{}'
$CLI call FollowService/Resume -d '{}'
$CLI call FollowService/Cancel -d '{}'

$CLI call ChargeService/Return -d '{"station_id":"","goal_id":"c1"}' --max-time 8
$CLI call ChargeService/GetStatus -d '{}'
$CLI call ChargeService/Pause -d '{}'
$CLI call ChargeService/Resume -d '{}'
$CLI call ChargeService/Cancel -d '{}'
$CLI call ChargeService/Leave -d '{"goal_id":"l1"}' --max-time 8

# 6b) Exploration（短流）
$CLI call ExplorationService/Explore -d '{
  "goal_id":"e1","map_name":"explore_smoke",
  "options":{"coverage_target":0.5,"timeout_seconds":60}
}' --max-time 8
$CLI call ExplorationService/GetStatus -d '{}'
$CLI call ExplorationService/Pause -d '{}'
$CLI call ExplorationService/Resume -d '{}'
$CLI call ExplorationService/SaveMap -d '{"map_name":"explore_smoke"}'
$CLI call ExplorationService/Cancel -d '{}'

# 7) 破坏性 / 高优先级（单独确认后再跑）
# $CLI call MapService/DeleteMap -d '{"map_identifier":"smoke"}'
# $CLI call SystemService/EmergencyStop -d '{"reason":"rpc-cli end"}'
# $CLI call SystemService/GetStatus -d '{}'
# $CLI call SystemService/ClearEmergencyStop -d '{"reason":"resume","confirmation_token":""}'
```

---

## 13. 方法速查

| Service | Methods |
|---------|---------|
| `SystemService` | Heartbeat, GetInfo, GetStatus, GetHealth, EmergencyStop, ClearEmergencyStop, CancelAllGoals, GetActiveGoal, GetCapabilities |
| `TeleopService` | Velocity（bidi）, DriveOnHeading, BackUp, Spin, Pause, Resume, Cancel, GetStatus |
| `ExplorationService` | Explore, Pause, Resume, Cancel, GetStatus, SetArea, SaveMap |
| `NavigationService` | Navigate, Pause, Resume, Replan, Cancel, GetStatus |
| `MapService` | StartMapping, FinishMapping, CancelMapping, GetMappingStatus, ListMaps, GetMap, GetMapMetadata, SaveMap, DeleteMap, SetCurrentMap |
| `LocalizationService` | GetPose, GetStatus, SetInitialPose |
| `ChargeService` | Return, Leave, Pause, Resume, Cancel, GetStatus |
| `FollowService` | Follow, Pause, Resume, Cancel, GetStatus |
| `SensorService` | ListSensors, GetSample, GetParameters, SetParameters, SaveParameters, LoadParameters, Record, CancelRecord, GetRecordStatus |

```bash
./rpc-cli.py list    # 打印带 package 的全名
```

## 13. 环境变量与说明

| 变量 | 含义 |
|------|------|
| `AUTOMSGS_RPC_TARGET` | 默认 `host:port` |
| `AUTOMSGS_PROTO_INCLUDE` | 现成 CMake `proto_include`（含 `automsgs/rpcs`） |

- 首次运行会在 `.proto_include/automsgs/` 建指向 `proto/{msgs,rpcs,...}` 的符号链接。  
- 安装后命令名：`rpc-cli`（源文件仍为 `rpc-cli.py`）。  
- 枚举可用名字（如 `FOLLOW_TARGET_TYPE_PERSON`）或数字。  
- 需目标上有对应 gRPC 实现；否则 `call` 会连接失败（`list`/`describe` 仍可用）。
