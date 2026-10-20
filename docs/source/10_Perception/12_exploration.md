# 12. 探索

配置里有 `PerceptionOptions.enable_rgbd_exploration`、`exploration_*.lua`，但当前构建**没有**探索后端。打开开关时 `PerceptionServer::Start()` 只会打印：

```text
PerceptionServer: exploration backend is not built.
```

探索能力属占位，不能用于验收或演示 TARE/主动 SLAM。视觉跟人与检测应走 **Base + Follow**；导航避障仍用 **costmap**。

### 日志对照

确认 WARN 与上文一致即可，无需再找探索话题。

### 可用替代

| 需求 | 入口 |
|------|------|
| 检测 / 深度 / 跟踪 | [§10 Base](10_base.md) |
| 跟人 + GridMap | [§11 Follow](11_follow.md) |
| 避障导航 | [§8 costmap](08_obstacle_perception.md) |

### 勿用于验收

- `exploration.pb.txt`、探索 Lua **无**对应 C++ 实现。  
- 勿与产品路线图上的探索 Server 混淆。
