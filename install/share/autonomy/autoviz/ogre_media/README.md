# ogre_media

Ogre 材质、GLSL 着色器与字体资源。资源命名空间为 **`aviz/`**（资源组 `aviz_rendering`）。
内容改编自 [rviz_rendering/ogre_media](https://github.com/ros2/rviz/tree/rolling/rviz_rendering/ogre_media)（BSD-3-Clause），并含 Autoviz 自有 PBR / overlay shader。

Autoviz `RenderSystem` 加载本目录后，可使用例如 `aviz/PointCloudSquare`、`aviz/DefaultPickAndDepth`、`AvizPBR` 等材质。

## 视觉模式

| Ogre 版本 | 行为 |
|-----------|------|
| **1.12.x** | 加载 `materials/scripts120/*.material` + GLSL → 完整点云 / pick / depth |
| **非 1.12** | 使用 C++ stub 材质 → 功能可用、外观简化 |

详见 [docs/rendering/ogre.md](../../docs/rendering/ogre.md)。

**启用完整材质脚本**：构建时加 `-DAUTOVIZ_OGRE_VENDOR=ON`（内建 Ogre 1.12.10），或系统 Ogre 为 1.12.x。

## 目录一览

- `materials/scripts120/point_cloud_*.material` — Square / FlatSquare / Sphere / Box
- `materials/scripts/` — 单像素点、瓦片、pick/depth、`AvizPBR*`
- `materials/glsl120/` — Ogre GLSL 1.20（点云 billboard、pick、depth、PBR）
- `materials/glsl330/` — Qt OpenGL 3.3（`SceneOverlay` / `GridRenderer`）
- `models/aviz_*.mesh` — cube / sphere / cylinder / cone 基础网格
