# ogre_media

Ogre 材质、GLSL 1.20 着色器与字体资源，源自 [rviz_rendering/ogre_media](https://github.com/ros2/rviz/tree/rolling/rviz_rendering/ogre_media)（BSD-3-Clause）。

Autoviz `RenderSystem` 以资源组名 `rviz_rendering` 加载，与 rviz 材质脚本（如 `rviz/PointCloudSquare`、`rviz/DefaultPickAndDepth`）兼容。

## 视觉模式

| Ogre 版本 | 行为 |
|-----------|------|
| **1.12.x** | 加载 `materials/scripts120/*.material` + GLSL → 与 rviz 一致 |
| **14.x（默认）** | 不解析脚本（会崩溃）；使用 C++ stub 材质 → 功能可用、外观简化 |

详见 [docs/OGRE_VISUAL_ALIGNMENT.md](../../docs/OGRE_VISUAL_ALIGNMENT.md)。

**与 rviz 点云视觉对齐**：构建时加 `-DAUTOVIZ_OGRE_VENDOR=ON`（内建 Ogre 1.12.10）。

## 点云材质一览

- `materials/scripts120/point_cloud_*.material` — Square / FlatSquare / Sphere / Box
- `materials/scripts/point_cloud_point.material` — 单像素点
- `materials/scripts/point_cloud_tile.material` — 瓦片
- `materials/glsl120/` — billboard、smooth_square、pick、depth shader
