# 源码模块

路径以包内 `autoviz/autoviz/` 为根。对外链接目标：`autoviz`（`libautoviz`）、入口 `autoviz_app`。

```text
autoviz/
├── main.cpp                 # 应用入口（autoviz_app）
├── common/                  # 框架与会话（含 interaction/ Autolink 集成）
├── plugins/displays/        # Display 实现（RViz 分目录布局）
├── plugins/tools/           # 交互工具
├── plugins/view_controllers/# 相机控制器插件壳（数学在 rendering/）
├── rendering/               # Ogre 视口、几何与拾取
├── transform/               # TF（含 vendored tf2）
├── ui/                      # Qt 界面
├── commsgs/                 # 消息类型工具
├── bridge/                  # 内嵌 WebSocket 服务端
└── platform/                # OpenGL / 平台初始化
```

## common/

| 组件 | 职责 |
|------|------|
| `VisualizationManager` | Display / View / Tool 生命周期与全局选项 |
| `DisplayContext` | 向插件暴露 frame、选择、Autolink 等 |
| `FrameManager` | Fixed Frame 与 TF 查询 |
| `DisplayRegistry` / `ToolRegistry` / `ViewControllerRegistry` | 类型注册与工厂 |
| `factory/` | RViz 风格 `Factory` / `ClassIdRecordingFactory` / `PluginFactory` |
| `SessionConfig` + YAML IO | `.autoviz` / `.rviz` |
| `PluginLoader` | `AUTOVIZ_PLUGIN_PATH` 动态库 |
| `common/transformation/` | `FrameTransformer`、`TransformationManager`、Identity、tf2 时间/头转换（无 ROS） |
| `common/interaction/` | Autolink Node、Channel、回放（命名空间 `autoviz::integration`） |

## common/interaction/

| 组件 | 职责 |
|------|------|
| Autolink 上下文 | Node 生命周期 |
| ChannelManager | 拓扑发现与候选列表 |
| ChannelReader / Writer 注册表 | 订阅与发布 |
| PlaybackController | `.record` 回放 |
| Service / Teleop 辅助 | 面板用服务与遥控 channel 约定 |

## plugins/displays/

Channel 型 Display 多继承 `ChannelDisplay`：属性中的 Channel 名对应 Autolink channel。内建类型包括 Grid、TF、LaserScan、PointCloud2、Map、Marker、RobotModel、Image、Camera、Path、Odometry 等；完整清单见 [parity/displays-panels.md](../parity/displays-panels.md)。

## rendering/

| 组件 | 职责 |
|------|------|
| `OgreRenderWindow` / `OgreRenderBackend` | **唯一的 3D 视口**（Ogre 1.x；`render_window.*` 是已退役的纯 GL 路径，不参与编译） |
| `objects/` | Arrow、Shape、PointCloud、MovableText 等 |
| Pick / FBO | GPU 拾取 |
| `ogre_media` | Ogre 材质与脚本（可选） |

详见 [rendering/backends.md](../rendering/backends.md)。

## ui/

| 区域 | 说明 |
|------|------|
| `frame*` | 主窗口、布局、会话、视口协作 |
| `displays/` | Displays 面板与属性树 |
| `views/` | 视图与相机管理面板 |
| `theme/` / `app/` | 主题、图标、翻译、偏好 |

## plugins/tools/ · transform/

- `plugins/tools/`（RViz 子目录布局）：`move/`、`select/`、`interaction/`、`pose/`、`goal_pose/`、`pose_estimate/`、`measure/`、`focus/`、`point/`、`common/`、`failed/`
- `transform/`：`Buffer` + vendored tf2；订阅 TF 消息写入缓存

## 构建产物

| 目标 | 产物 |
|------|------|
| `autoviz` | `libautoviz`（`autoviz::core`） |
| `autoviz_app` | `bin/autoviz` |
