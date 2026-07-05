(bridge-architecture)=
# 2. 架构设计

> **`AutonomyService`** 数据流与 13 RPC 契约（`external_command_service.proto`）。上手 [§0](00_guide.md) · [§3 RPC](rpcs/index.rst)

<div class="nav-costmap-banner">
  <strong>Bridge 在 Autonomy 栈中的位置</strong>
  <span class="nav-costmap-detail">Client → BridgeServer → GrpcBridgeServer / MqttBridge → 机载栈</span>
  <span class="nav-costmap-arrow">不参与规划控制 →</span>
</div>

---

## 2.1 数据流

<div class="nav-state-diagram bridge-arch-flow">

```mermaid
%%{init: {"theme":"base","themeVariables":{"fontSize":"13px","primaryColor":"#e0f2f2","primaryTextColor":"#1a4d6e","primaryBorderColor":"#2d9294","lineColor":"#e53935","edgeLabelBackground":"#ffffff","labelTextColor":"#455a64"},"flowchart":{"nodeSpacing":24,"rankSpacing":28,"padding":8,"htmlLabels":true,"useMaxWidth":false}}}%%
flowchart TB
    Client["<div style='width:18rem;height:4.5rem;display:flex;align-items:center;justify-content:center;text-align:center;line-height:1.35;font-size:13px;box-sizing:border-box;padding:0.45rem 0.65rem;overflow:hidden;background:#e3f2fd;border-radius:6px;color:#0d47a1'>L1<br/>Client</div>"]
    BS["<div style='width:18rem;height:4.5rem;display:flex;align-items:center;justify-content:center;text-align:center;line-height:1.35;font-size:13px;box-sizing:border-box;padding:0.45rem 0.65rem;overflow:hidden;background:#e0f2f2;border-radius:6px;color:#1a4d6e'>L2<br/>BridgeServer</div>"]
    TX["<div style='width:18rem;height:4.5rem;display:flex;align-items:center;justify-content:center;text-align:center;line-height:1.35;font-size:13px;box-sizing:border-box;padding:0.45rem 0.65rem;overflow:hidden;background:#fff8e1;border-radius:6px;color:#e65100'>L3<br/>GrpcBridgeServer / MqttBridgeServer</div>"]
    HD["<div style='width:18rem;height:4.5rem;display:flex;align-items:center;justify-content:center;text-align:center;line-height:1.35;font-size:13px;box-sizing:border-box;padding:0.45rem 0.65rem;overflow:hidden;background:#f3e5f5;border-radius:6px;color:#6a1b9a'>L3<br/>Handler + FSM/BT</div>"]
    Stack["<div style='width:18rem;height:4.5rem;display:flex;align-items:center;justify-content:center;text-align:center;line-height:1.35;font-size:13px;box-sizing:border-box;padding:0.45rem 0.65rem;overflow:hidden;background:#e8f5e9;border-radius:6px;color:#2e7d32'>L4<br/>Navigator · Exploration · Map</div>"]

    Client -->|"RPC 下行"| BS --> TX --> HD --> Stack
    Stack -.-> HD -.-> TX -.-> BS -.->|"响应上行"| Client

    classDef l1 fill:#e3f2fd,stroke:#1565c0,color:#0d47a1
    classDef l2 fill:#e0f2f2,stroke:#2d9294,color:#1a4d6e
    classDef l3tx fill:#fff8e1,stroke:#fb8c00,color:#e65100
    classDef l3hd fill:#f3e5f5,stroke:#8e24aa,color:#6a1b9a
    classDef l4 fill:#e8f5e9,stroke:#43a047,color:#2e7d32
    class Client l1
    class BS l2
    class TX l3tx
    class HD l3hd
    class Stack l4
```

</div>

## 2.2 RPC Service

与 proto `service AutonomyService` 块顺序一致：

| 分类 | RPC | 请求 → 响应 | 模式 | 文档 |
|------|-----|-------------|------|------|
| Push | `ReceiveBotStates` | `Empty` → stream `RobotState` | Empty → Stream | [rpcs/05](rpcs/05_stream_api.md) |
| | `ReceiveBotEvents` | `Empty` → stream `RobotEvent` | Empty → Stream | [rpcs/05](rpcs/05_stream_api.md) |
| Query | `GetRobotSnapshot` | `Empty` → `RobotState` | Unary → Unary | [rpcs/04](rpcs/04_query_api.md) |
| | `GetActiveTask` | `Empty` → `ActiveTaskInfo` | Unary → Unary | [rpcs/04](rpcs/04_query_api.md) |
| | `GetCapabilities` | `Empty` → `Capabilities` | Unary → Unary | [rpcs/04](rpcs/04_query_api.md) |
| System | `EmergencyStop` | `EmergencyStopRequest` → `CommandAck` | Unary → Unary | [rpcs/06](rpcs/06_system_api.md) |
| | `CancelAllTasks` | `CancelAllTasksRequest` → `CommandAck` | Unary → Unary | [rpcs/06](rpcs/06_system_api.md) |
| Command | `SendNavigationCommand` | `NavigationCommandRequest` → stream `NavigationCommandResponse` | Unary → Stream | [rpcs/07](rpcs/07_navigation_command.md) |
| | `SendFollowCommand` | `FollowCommandRequest` → stream `FollowCommandResponse` | Unary → Stream | [rpcs/09](rpcs/09_follow_command.md) |
| | `SendTeleopCommand` | stream `TeleopCommandRequest` → stream `TeleopCommandResponse` | Stream ↔ Stream | [rpcs/10](rpcs/10_teleop_command.md) |
| | `SendExplorationCommand` | `ExplorationCommandRequest` → stream `ExplorationCommandResponse` | Unary → Stream | [rpcs/08](rpcs/08_exploration_command.md) |
| | `SendDockCommand` | `DockCommandRequest` → stream `DockCommandResponse` | Unary → Stream | [rpcs/11](rpcs/11_dock_command.md) |
| | `SendMapCommand` | `MapCommandRequest` → stream `MapCommandResponse` | Unary → Stream | [rpcs/12](rpcs/12_map_command.md) |

---

**导航**：[← §0 指南](00_guide.md) · [rpcs/ API →](rpcs/index.rst)
