# Autoviz ↔ rviz_rendering 3D 渲染对齐

> 参考：`rviz_rendering` · `rviz_rendering_tests` · `rviz_ogre_vendor`  
> Autoviz 路径：`src/autonomy/autoviz/autoviz/rendering/`

## 架构对照

| 维度 | rviz_rendering | Autoviz |
|------|----------------|------|
| 引擎 | Ogre 1.12 为主 | OpenGL 3.3 默认；`-DAUTOVIZ_USE_OGRE=ON` 可选 Ogre |
| Ogre 版本 | `rviz_ogre_vendor` 1.12.10 | **系统 Ogre 14.4+**（默认）；可选 vendor 1.12 |
| Ogre 引导 | `RenderSystem` 单例 + ogre_media | ✅ `RenderSystem` + `resources/ogre_media/` |
| mesh_loader | Assimp + `resource_retriever` | ✅ Assimp + `MeshResourceResolver`（`package://` / `file://` / `http(s)://`） |
| 场景模型 | 持久 `SceneNode` + `MovableObject` | ✅ `OgreSceneHost` 持久对象 |
| Shape/Line/Arrow | rviz objects API | ✅ `OgreShape` / `OgreLine` / `OgreArrow`（含 Capsule） |
| Wrench/Screw/Effort/Covariance | rviz Visual 类 | ✅ `OgreWrenchVisual` / `OgreScrewVisual` / `OgreEffortVisual` / `OgreCovarianceVisual` |
| MeshShape / TrianglePolygon | rviz objects | ✅ `OgreMeshShape` / `OgreTrianglePolygon` |
| geometry / orthographic | rviz 辅助 | ✅ `geometry` / `orthographic` |
| Ogre 日志注入 | `logging` + `OgreLogging` | ✅ `ogre_logging` + `OgreOgreLogging` |
| RenderSystem 选项 | FSAA / force GL / stereo | ✅ `disableAntiAliasing` / `forceGlVersion` / `forceNoStereo` |
| gtest | gtest + `rviz_rendering_tests` | ✅ mesh_loader 14 项 + rendering objects 8 项 |

## Ogre 14 vs rviz 1.12 材质

系统 **Ogre 14.x** 下不加载 rviz GLSL 材质脚本（会 segfault），改用 `ensureStubRvizMaterials()` 程序化材质。点云/Shape 等功能可用；与 rviz 完全一致的 shader 效果见 **[OGRE_VISUAL_ALIGNMENT.md](OGRE_VISUAL_ALIGNMENT.md)**（推荐 `-DAUTOVIZ_OGRE_VENDOR=ON`）。

## 已完成

| 项 | 状态 |
|----|------|
| PointCloud / Pick / BillboardLine / MovableText | ✅ |
| Entity 路径 / RobotModel / Marker | ✅ |
| indexed_8bit 强度调色板 | ✅ |
| Assimp mesh_loader（DAE/STL/GLB） | ✅ |
| `MeshResourceResolver`（package://） | ✅ |
| 系统 Ogre 14.4 默认路径 | ✅ |
| gtest（line/shape/point_cloud/mesh_loader） | ✅ `-DBUILD_AUTOVIZ_TESTS=ON` |

## 仍与 rviz 有差距（非阻塞）

- **点云/Shape 像素级视觉**：Ogre 14 默认 stub；对齐方案见 [OGRE_VISUAL_ALIGNMENT.md](OGRE_VISUAL_ALIGNMENT.md)（Ogre 1.12 vendor 或未来 GLSL 330 重写）
- ament `rviz_ogre_media_exports` 注册机制（Autoviz 用 CMake install + `MeshResourceResolver`）

## 推荐构建（系统 Ogre 14.4）

```bash
# 依赖：pkg-config OGRE + OGRE-Overlay（如 14.4.0）
cmake -S src/autonomy -B build/autonomy \
  -DBUILD_AUTOVIZ=ON -DAUTOVIZ_USE_OGRE=ON \
  -DBUILD_AUTOVIZ_TESTS=ON

cmake --build build/autonomy --target autoviz autoviz_mesh_loader_test autoviz_rendering_objects_test -j$(nproc)
DISPLAY=:1 ctest -R autoviz_ --test-dir build/autonomy
```

环境变量（可选）：

```bash
export AUTOVIZ_OGRE_PLUGIN_DIR=/usr/local/lib/OGRE
export AUTOVIZ_OGRE_MEDIA_PATH=$(pwd)/src/autonomy/autoviz/resources/ogre_media
```

## 遗留 rviz 1.12 对齐（可选）

```bash
cmake ... -DAUTOVIZ_OGRE_VENDOR=ON   # FetchContent Ogre 1.12.10 + patches
# 或
cmake ... -DAUTOVIZ_OGRE_ROOT=/opt/ogre-1.12
```

GUI 验证见 [OGRE_GUI_VERIFICATION.md](OGRE_GUI_VERIFICATION.md).
