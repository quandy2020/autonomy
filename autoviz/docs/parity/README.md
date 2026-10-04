# RViz2 对齐

图例：✅ 已实现 · ⚠️ 部分 · ❌ 未实现。目标是 **概念与能力对齐**，非插件 ABI 兼容。

| 文档 | 内容 |
|------|------|
| [displays-panels.md](displays-panels.md) | Display / Panel / Tool / View |
| [framework.md](framework.md) | rviz_common 框架类对照 |
| [rendering.md](rendering.md) | rviz_rendering / Ogre 对象对照 |

## 原则

- Display、Tool、Panel、ViewController、Fixed Frame、属性树与 RViz2 同构
- 通信层为 Autolink channel，而非 ROS Topic
- 渲染固定为 Ogre 1.x；不再提供纯 OpenGL 视口
