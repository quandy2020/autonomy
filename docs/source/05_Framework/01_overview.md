(framework-overview)=
# 1. 框架概览

各业务域的 **Server** 怎么装配、配置怎么灌进去、和 Autolink 的边界在哪。通信 API 细节在 [03 Communication](../03_Communication/01_architecture.md)；消息类型在 [commsgs](../14_Commsgs/01_overview.md)。

用一张职责表建立索引，细节跳到专题页，不在概览里展开命令序列。

```text
各域 *Server · 插件 · CreateOptions
        ↓
     commsgs（struct + proto）
        ↓
     autolink（Node / Channel / Service / Action）
```

进程内曾经的 `CreateAutonomy` 已拿掉；车上用 `autolink_launch autonomy.launch` 起多进程。

## 你会碰到的组件

| 组件 | 目录 | 干什么 |
|------|------|--------|
| 域进程入口 | `autonomy/*/*_main.cpp` | 每个域一个可执行文件 |
| 共享 Options | `autonomy/system/options.*` | `AutonomyOptions` / CreateOptions |
| MapServer | `autonomy/map/` | 静态图、OccupancyGrid |
| PlannerServer | `autonomy/planning/` | 全局规划、costmap |
| ControllerServer | `autonomy/control/` | 局部跟踪、`cmd_vel` |
| Task | `autonomy/task/` | BT 编排 |
| Transform | `autonomy/transform/` | TF |
| Bridge | `autonomy/bridge/` | 对外 gRPC |

支撑模块（传感整理、底盘桥、麦阵）见 [09](09_sensor_vehicle_audio.md)。配置管线见 [04](04_configuration.md)；插件规则见 [06](06_plugin_system.md)。
