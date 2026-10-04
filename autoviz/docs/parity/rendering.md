# 渲染对照（rviz_rendering）

| 维度 | rviz_rendering | Autoviz |
|------|----------------|---------|
| 默认引擎 | Ogre 1.12 | **Ogre 1.12**（强制；可 vendor） |
| Ogre 版本 | vendor 1.12.10 | 系统 14.x 或 vendor 1.12 |
| 引导 | RenderSystem + ogre_media | ✅ 同概念 |
| mesh_loader | Assimp + resource_retriever | ✅ Assimp + MeshResourceResolver |
| 场景模型 | 持久 SceneNode | ✅ OgreSceneHost |
| Shape / Line / Arrow | objects API | ✅ `OgreShape` / `OgreLine` / `OgreArrow` |
| Wrench / Screw / Effort / Covariance | Visual 类 | ✅ 对应 Ogre*Visual |
| MeshShape / TrianglePolygon | objects | ✅ |
| geometry / orthographic | 辅助 | ✅ |
| FSAA / GL 版本等 | RenderSystem 选项 | ✅ |

## 已覆盖能力

PointCloud、Pick、BillboardLine、MovableText、Entity / RobotModel / Marker、强度调色板、Assimp 网格、`package://` 解析。

## 已知差距

- Ogre 14 stub 材质下点云/Shape **像素级**外观与 rviz 不等价；对齐见 [rendering/ogre.md](../rendering/ogre.md)
- 无 ament `rviz_ogre_media_exports`；改用安装布局 + `MeshResourceResolver`

## 相关文档

- [backends.md](../rendering/backends.md) · [ogre.md](../rendering/ogre.md)
