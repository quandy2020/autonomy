# 渲染后端

Autoviz **视口固定为 Ogre 1.x**（与 RViz2 同代）。不再提供纯 OpenGL（`QOpenGLWidget`）视口。

底层仍由 Ogre 的 **OpenGL / GL3+ RenderSystem** 驱动 GPU；这与已移除的 Autoviz OpenGL 自绘路径无关。

## 版本策略

| 模式 | 条件 | 点云 / rviz 材质 |
|------|------|------------------|
| **推荐 · vendor 1.12.10** | `AUTOVIZ_OGRE_VENDOR=ON`，或默认 `AUTOVIZ_OGRE_AUTO_VENDOR=ON` 且系统非 1.12 | 完整 `ogre_media` GLSL |
| 系统 Ogre 1.12 | `AUTOVIZ_OGRE_ROOT` 或 pkg-config 为 1.12 | 同上 |
| 系统 Ogre 14.x | 未开 auto-vendor 时 | stub 材质（功能可用，像素不等价） |

**不要使用 Ogre Next**：API 与材质体系不兼容现有 `objects/*` 与 `ogre_media`。

## CMake 选项

| 选项 | 默认 | 说明 |
|------|------|------|
| `AUTOVIZ_OGRE_AUTO_VENDOR` | **ON** | 系统非 1.12 时 FetchContent 1.12.10 |
| `AUTOVIZ_OGRE_VENDOR` | OFF | 强制 FetchContent 1.12.10 |
| `AUTOVIZ_OGRE_ROOT` | — | 预编译 1.12 前缀 |
| `AUTOVIZ_USE_ASSIMP` | ON | mesh_loader 使用 Assimp |

```bash
# 默认即可（无 1.12 时自动 vendor）
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release

# 强制 vendor
cmake -S . -B build -DAUTOVIZ_OGRE_VENDOR=ON
```

## 相关文档

- [Ogre 视觉与验证](ogre.md) · [渲染对齐](../parity/rendering.md)
