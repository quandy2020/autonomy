# 4. 配置说明

仓库里看到 `visualization.lua`，想知道改它会不会影响运行——**默认不会**。

记录 `config/visualization/visualization.lua` 里**预留**的 WebSocket/Foxglove 风格字段，供将来若有原生可视化进程时使用。

- 该文件 **未被** `autonomy.lua` include，C++ 侧 **无** `LoadOptions("visualization")`。  
- 改 `host`/`port`/`topic_subscriptions` **不会**改变当前 Autoviz/Foxglove/RViz 行为。  
- Docker 映射 **8765** 是为 **foxglove_bridge**（ROS），不是未实现的 VisualizationServer。

怎么选调试手段：

| 需求 | 用啥 |
|------|------|
| 本机 Autolink 桌面 | [Autoviz](09_autoviz.md) |
| 浏览器 WS | foxglove_bridge + [§6](06_foxglove.md) |
| ROS RViz | [§7](07_rviz2_ros2.md) |
| 改 visualization.lua 立刻生效 | **不支持** |

### 文件内字段（参考）

| 字段 | 默认 | 含义 |
|------|------|------|
| `server_name` | `"AutonomyViewer"` | 预留服务名 |
| `host` / `port` | `0.0.0.0` / `8765` | 与 Foxglove 常用端口对齐 |
| `enable_autolink` 等 | true | 预留转发开关 |

`topic_subscriptions` 里列了 `/planning/path`、`/map` 等示例名；实际栈里可能是 `/plan`、`map`，以 [common.lua](../05_Framework/04_configuration.md) 与 launch 为准。

### 若将来接入（未做）

```lua
-- 仅为规划片段，当前仓库未启用
include "visualization/visualization.lua"
AUTONOMY = { visualization = AUTONOMY_VISUALIZATION }
```

### 常见坑

| 误解 | 事实 |
|------|------|
| 8765 没人听 = Server 挂了 | 可能未起 bridge，或 lua 本就不加载 |
| 配了订阅列表就有数据 | 无进程读该 lua |

### 延伸阅读

[§5 架构](05_architecture.md) · [§6 Foxglove](06_foxglove.md)
