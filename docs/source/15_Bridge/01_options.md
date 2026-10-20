---
orphan: true
---
(bridge-options)=
# 1. 参数配置

机载 `bridge/proto/*_options.proto`（**非**外部 RPC）。对外 API 见 [rpcs/](rpcs/index.rst)。

| 文件 | 消息 |
|------|------|
| `grpc_options.proto` | `GrpcOptions` |
| `robot_identity_options.proto` | `RobotIdentityOptions` |
| `capabilities_options.proto` | `CapabilitiesOptions` |
| `bridge_options.proto` | `BridgeOptions`（聚合上三者） |

Lua → `proto::BridgeOptions` → `BridgeServer`。

### 1.1 RobotIdentityOptions

| 字段 | 说明 | 默认 |
|------|------|------|
| `robot_id` | 车队内唯一 ID | hostname |
| `model` / `serial_number` | 型号 / 序列号 | model → `autonomy` |
| `firmware_version` / `software_version` | 版本 | software → `autonomy` |
| `fleet_id` / `site_id` | 车队 / 场站 | 空 |

### 1.2 CapabilitiesOptions

`optional bool`：**未设置 = 宣称支持**；`false` = 广告不支持。

| 字段 | 影响 |
|------|------|
| `supports_navigation` / `follow` / `teleop` / `exploration` | 域能力位 |
| `supports_docking` / `supports_charge` | 对接 / 充电 |
| `supports_map_management` / `mapping` / `localization` / … | Map / 定位等 |

### 1.3 GrpcOptions

| 字段 | 说明 | 默认 |
|------|------|------|
| `host` / `port` | 监听 | `127.0.0.1` / `5005` |
| `num_grpc_threads` / `num_event_threads` | CQ / EQ | `5` / `5` |
| `enable_ssl_encryption` | TLS | — |
| `num_worker_threads` | Action 池；`0` → `max(2, event/2)` | `0` |

```lua
AUTONOMY_BRIDGE = {
    identity = { robot_id = "amr-001", model = "autonomy", fleet_id = "demo" },
    capabilities = { supports_exploration = false },
    grpc = { host = "127.0.0.1", port = 5005 },
}
```
