# 源码模块

路径以包内 `autoviz/autoviz/` 为根。对外链接目标：`autoviz`（`libautoviz`）、入口 `autoviz_app`。

```text
autoviz/
├── main.cpp                 # 应用入口（autoviz_app）
├── common/                  # 框架与会话
├── display/                 # Display 实现
├── integration/             # Autolink 集成
├── rendering/               # 渲染后端与几何
├── tools/                   # 交互工具
├── transform/               # TF
├── ui/                      # Qt 界面
├── commsgs/                 # 消息类型工具
└── platform/                # OpenGL / 平台初始化
```

## common/

| 组件 | 职责 |
|------|------|
| `VisualizationManager` | Display / View / Tool 生命周期与全局选项 |
| `DisplayContext` | 向插件暴露 frame、选择、Autolink 等 |
| `FrameManager` | Fixed Frame 与 TF 查询 |
| `DisplayRegistry` / `ToolRegistry` / `ViewControllerRegistry` | 类型注册与工厂 |
| `SessionConfig` + YAML IO | `.autoviz` / `.rviz` |
| `PluginLoader` | `AUTOVIZ_PLUGIN_PATH` 动态库 |

## display/

Channel 型 Display 多继承 `ChannelDisplay`：属性中的 Channel 名对应 Autolink channel。内建类型包括 Grid、TF、LaserScan、PointCloud2、Map、Marker、RobotModel、Image、Camera、Path、Odometry 等；完整清单见 [parity/displays-panels.md](../parity/displays-panels.md)。

## integration/

| 组件 | 职责 |
|------|------|
| Autolink 上下文 | Node 生命周期 |
| ChannelManager | 拓扑发现与候选列表 |
| ChannelReader / Writer 注册表 | 订阅与发布 |
| PlaybackController | `.record` 回放 |
| Service / Teleop 辅助 | 面板用服务与遥控 channel 约定 |

## rendering/

| 组件 | 职责 |
|------|------|
| `RenderWindow` / 后端 | **Ogre 1.x**（`OgreRenderWindow`；无纯 GL 视口） |
| `objects/` | Arrow、Shape、PointCloud、MovableText 等 |
| Pick / FBO | GPU 拾取 |
| `ogre_media` | Ogre 材质与脚本（可选） |

详见 [rendering/backends.md](../rendering/backends.md)。

## ui/

| 区域 | 说明 |
|------|------|
| `frame*` | 主窗口、布局、会话、视口协作 |
| `displays/` | Displays 面板与属性树 |
| `views/` / `tools/` | 视图与工具栏 |
| `theme/` / `app/` | 主题、图标、翻译、偏好 |

## tools/ · transform/

- `tools/`：Interact、测距、Focus、NavGoal、初始位姿等
- `transform/`：`Buffer` + vendored tf2；订阅 TF 消息写入缓存

## 构建产物

| 目标 | 产物 |
|------|------|
| `autoviz` | `libautoviz`（`autoviz::core`） |
| `autoviz_app` | `bin/autoviz` |
