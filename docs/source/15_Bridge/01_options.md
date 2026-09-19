---
orphan: true
---
(bridge-options)=
# 1. 参数配置

> **§1** · `bridge/proto/*_options.proto` · 机载部署配置（**非** 外部 RPC；对外 API 为 automsgs.rpcs）。集成方远程调 API 通常无需阅读本章；入口见 [§0.3 配置入口](00_guide.md#03-配置入口)。

一消息一文件，由 `bridge_options.proto` 聚合：

| 文件 | 消息 |
|------|------|
| `grpc_options.proto` | `GrpcOptions` |
| `robot_identity_options.proto` | `RobotIdentityOptions` |
| `capabilities_options.proto` | `CapabilitiesOptions` |
| `bridge_options.proto` | `BridgeOptions`（import 上三者） |

Lua → `proto::BridgeOptions` → `BridgeServer`。

## 1.1 消息结构

```protobuf
// bridge_options.proto
message BridgeOptions {
  GrpcOptions grpc = 3;
  RobotIdentityOptions identity = 4;       // GetRobotFullInfo / GetInfo
  CapabilitiesOptions capabilities = 5;  // GetCapabilities 广告开关
}
```

## 1.2 RobotIdentityOptions

| 字段 | 说明 | 默认 |
|------|------|------|
| `robot_id` | 车队内唯一 ID | 空 → hostname |
| `model` / `serial_number` | 型号 / 序列号 | model 空 → `autonomy` |
| `firmware_version` / `software_version` | 固件 / 软件版本 | software 空 → `autonomy` |
| `fleet_id` / `site_id` | 车队 / 场站 | 空 |

## 1.3 CapabilitiesOptions

均为 `optional bool`：**未设置 = 对外宣称支持**；显式 `false` = 广告为不支持（SKU / 裁剪）。

| 字段 | 影响表面 |
|------|----------|
| `supports_navigation` / `follow` / `teleop` / `exploration` | Autonomy + Rpc |
| `supports_docking` | Autonomy `supports_docking` |
| `supports_charge` | Rpc ChargeService |
| `supports_map_management` | Autonomy Map 命令 |
| `supports_mapping` / `localization` / `sensor_record` / `system_monitor` | Rpc |

## 1.4 GrpcOptions

| 字段 | 说明 | `bridge.lua` 默认 |
|------|------|-------------------|
| `host` / `port` | 监听地址 | `127.0.0.1` / `5005` |
| `num_grpc_threads` / `num_event_threads` | 工作 / CQ 线程 | `5` / `5` |
| `enable_ssl_encryption` | TLS | — |
| `uplink_server_address` 等 | 上行预留 | — |
| `num_worker_threads` | Action 阻塞池；`0` → `max(2, event/2)` | `0` |

```lua
AUTONOMY_BRIDGE = {
    identity = {
        robot_id = "amr-001",
        model = "autonomy",
        fleet_id = "demo-fleet",
        site_id = "warehouse-a",
    },
    capabilities = {
        supports_exploration = false,  -- 可选裁剪
    },
    grpc = { host = "127.0.0.1", port = 5005 },
}
```

---

**导航**：[← §0.3 配置入口](00_guide.md#03-配置入口) · [§2 架构设计 →](02_architecture.md)
