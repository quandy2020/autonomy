# 1. 参数配置

Bridge 机载选项来自 `bridge/proto/*_options.proto`（**非**对外 RPC 契约）。写客户端前仍应读 [rpcs/](rpcs/index.rst)；本节解决监听地址、机器人身份、以及对云 **宣称** 哪些 `supports_*` 能力。

Lua 生成 `proto::BridgeOptions` 后交给 `BridgeServer`。`CapabilitiesOptions` 里 **未设置的 bool 视为宣称支持**；显式 `false` 才会让 `GetCapabilities` 报不支持。机载 Task 的 `enable_*` 与 Capabilities 宜一致，避免云侧发了 Command 而 Task 未启用。

### proto 文件

| 文件 | 消息 |
|------|------|
| `grpc_options.proto` | `GrpcOptions` |
| `robot_identity_options.proto` | `RobotIdentityOptions` |
| `capabilities_options.proto` | `CapabilitiesOptions` |
| `bridge_options.proto` | `BridgeOptions`（聚合上三者） |

### RobotIdentityOptions

| 字段 | 说明 | 默认 |
|------|------|------|
| `robot_id` | 车队内唯一 ID | hostname |
| `model` / `serial_number` | 型号 / 序列号 | model → `autonomy` |
| `firmware_version` / `software_version` | 版本 | software → `autonomy` |
| `fleet_id` / `site_id` | 车队 / 场站 | 空 |

### CapabilitiesOptions

| 字段 | 影响 |
|------|------|
| `supports_navigation` / `follow` / `teleop` / `exploration` | 域能力位 |
| `supports_docking` / `supports_charge` | 对接 / 充电 |
| `supports_map_management` / `mapping` / `localization` / … | Map / 定位等 |

### GrpcOptions

| 字段 | 说明 | 默认 |
|------|------|------|
| `host` / `port` | 监听 | `127.0.0.1` / `5005` |
| `num_grpc_threads` / `num_event_threads` | CQ / EQ | `5` / `5` |
| `enable_ssl_encryption` | TLS | 默认关；上生产再开 |
| `num_worker_threads` | Action 池；`0` → `max(2, event/2)` | `0` |

### Lua 示例

```lua
AUTONOMY_BRIDGE = {
    identity = { robot_id = "amr-001", model = "autonomy", fleet_id = "demo" },
    capabilities = { supports_exploration = false },
    grpc = { host = "127.0.0.1", port = 5005 },
}
```

需 **`BUILD_GRPC=ON`** 才会编进 `autonomy.bridge`。
