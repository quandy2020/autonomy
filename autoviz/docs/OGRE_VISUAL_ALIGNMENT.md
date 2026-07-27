# Ogre 点云与 rviz 视觉对齐

> 点云、Pick、Depth 等依赖 `resources/ogre_media/` 下 rviz GLSL 1.20 材质脚本。  
> 默认 **系统 Ogre 14.x** 无法安全解析这些脚本，Autoviz 改用 **stub 程序化材质**（功能对齐、视觉不等价）。

## 三种模式

| 模式 | 条件 | 点云外观 | Pick/Depth |
|------|------|----------|------------|
| **A · rviz GLSL（推荐视觉对齐）** | Ogre **1.12.x** + `AUTOVIZ_OGRE_RVIZ_MEDIA` | 圆角方形 billboard、抗锯齿边缘 | ✅ Pick / Pick1 / Depth |
| **B · stub（默认 Ogre 14）** | 系统 Ogre 14.4+，无 vendor | 纯色平面 quad，无 smooth_square | ⚠️ 基础可见；Pick 依赖 stub 材质 |
| **C · Ogre 14 GLSL 重写（未实现）** | 见下文路线图 | 目标与 rviz 一致 | 需重写 shader + 程序化材质 |

运行时可在日志中确认：

```text
Autoviz RenderSystem ready (... visual=rviz_glsl ...)   # 模式 A
Autoviz RenderSystem ready (... visual=stub ...)       # 模式 B
```

---

## 模式 A：Ogre 1.12 vendor（与 rviz 像素级对齐）

### 方式 1 — CMake 内建 vendor（推荐）

首次配置会 FetchContent 拉取 Ogre **v1.12.10** 并打补丁，编译时间约 5–15 分钟。

```bash
cmake -S src/autonomy -B build/autonomy \
  -DBUILD_AUTOVIZ=ON \
  -DAUTOVIZ_USE_OGRE=ON \
  -DAUTOVIZ_OGRE_VENDOR=ON \
  -DBUILD_AUTOVIZ_TESTS=ON

cmake --build build/autonomy --target autoviz -j$(nproc)
```

配置输出应含：

```text
Autoviz point cloud visual: rviz ogre_media GLSL (Ogre 1.12.x)
```

### 方式 2 — 自动 vendor（系统非 1.12 时）

```bash
cmake ... -DAUTOVIZ_OGRE_AUTO_VENDOR=ON
```

检测到系统 Ogre 不是 1.12.x 时自动开启 vendor。

### 方式 3 — 预编译 Ogre 1.12

```bash
cmake ... \
  -DAUTOVIZ_OGRE_ROOT=/opt/ogre-1.12 \
  -DAUTOVIZ_OGRE_VENDOR=OFF
```

`AUTOVIZ_OGRE_ROOT` 下需有 `include/OGRE`、`lib/OgreMain`、`lib/OGRE/RenderSystem_GL.so` 等；版本须为 **1.12.x** 才会定义 `AUTOVIZ_OGRE_RVIZ_MEDIA`。

### 方式 4 — 系统 pkg-config 已是 1.12

若 `pkg-config --modversion OGRE` 为 `1.12.*`，CMake 自动启用 `AUTOVIZ_OGRE_RVIZ_MEDIA`，无需 vendor。

### 验证视觉对齐

```bash
export AUTOVIZ_OGRE_MEDIA_PATH=$(pwd)/src/autonomy/autoviz/resources/ogre_media
export AUTOVIZ_OGRE_PLUGIN_DIR=<ogre-1.12-plugin-dir>   # vendor 构建时 CMake 会注入
./build/bin/autoviz
```

1. View → **Ogre** 视口  
2. Add **PointCloud2**，Style = **Squares**  
3. 点应为 **圆角方形**（非生硬矩形），边缘有 smoothstep 过渡  
4. Pick 工具点击应返回正确 point index（见 [OGRE_GUI_VERIFICATION.md](OGRE_GUI_VERIFICATION.md)）

涉及材质（资源组 `rviz_rendering`）：

| 材质名 | 用途 |
|--------|------|
| `rviz/PointCloudSquare` | 默认方形点 |
| `rviz/PointCloudFlatSquare` | 平面方形 |
| `rviz/PointCloudSphere` | 圆形点 |
| `rviz/PointCloudPoint` | 单像素点 |
| `rviz/PointCloudTile` / `rviz/PointCloudBox` | 瓦片/盒 |

每个材质含 4 个 technique：`nogp`、scheme **Depth**、**Pick**、**Pick1**（见 `materials/scripts120/point_cloud_square.material`）。

---

## 模式 B：Ogre 14 默认 stub

系统 Ogre 14 解析 rviz `.material` 脚本时会在材质编译阶段 **segfault**（Ogre 14 材质/GLSL 管线与 1.12 脚本格式不兼容），故默认：

- **不**加载 `materials/scripts120/*.material`
- `OgreMaterialManager::ensureStubRvizMaterials()` 创建无光照、`TVC_DIFFUSE` 程序化材质
- 点云 **可见、可更新、可 pick（能力受限）**，但 **无** smooth_square 圆角与 highlight

CMake 提示：

```text
Autoviz Ogre backend: system Ogre 14.4.0
Autoviz point cloud visual: stub materials (see docs/OGRE_VISUAL_ALIGNMENT.md)
  For rviz-identical shaders: -DAUTOVIZ_OGRE_VENDOR=ON
```

无需额外环境变量；`AUTOVIZ_OGRE_MEDIA_PATH` 仍建议设置（字体、plugins.cfg）。

---

## 模式 C：Ogre 14 点云 GLSL 重写（路线图）

**当前未实现。** 若要在 Ogre 14 上不用 vendor 而达到 rviz 视觉，需完成：

### 1. Shader 语言升级

现有 shader 为 **GLSL 1.20**（`gl_Vertex` / `gl_Color` / `gl_TexCoord`），Ogre 14 OpenGL 3+ 需 **GLSL 330+**（`in`/`out`、`layout(location=…)`）。

待移植文件（`resources/ogre_media/materials/glsl120/`）：

| 文件 | 用途 |
|------|------|
| `nogp/billboard.vert` | 点云 billboard 顶点 |
| `smooth_square.frag` | 圆角方形片元 |
| `pass_color.frag` / `pickcolor.frag` | Pick1 / Pick |
| `depth.frag` + `include/pass_depth.vert` | Depth scheme |
| `point.vert` | Points 模式 |
| `shaded_circle.frag` / `flat_color*.frag` | Sphere / FlatSquare |

### 2. 材质创建方式

Ogre 14 不宜依赖 `.material` 脚本解析；应通过 **C++ 程序化** 创建：

```cpp
// 伪代码
GpuProgramManager::getSingleton().loadFromFile("billboard.vert", GPT_VERTEX_PROGRAM, "rviz_rendering");
MaterialPtr m = MaterialManager::getSingleton().create("rviz/PointCloudSquare", "rviz_rendering");
Technique* t = m->createTechnique();
Pass* p = t->createPass();
p->setVertexProgram("..."); p->setFragmentProgram("...");
// 复制 Depth / Pick / Pick1 scheme 与 uniform（size, alpha, highlight, worldviewproj_matrix）
```

### 3. 与 `OgrePointCloud` 的衔接

`ogre_point_cloud.cpp` 已通过名称引用材质，并依赖多 scheme（`Pick`、`Pick1`、`Depth`）。重写后须保持：

- 材质名不变（`rviz/PointCloudSquare` 等）
- `changingGeometrySupportIsNecessary()` 对 uniform 的处理不变
- `OgrePickRenderer` 的 RenderTexture + scheme 切换不变

### 4. 建议实现顺序

1. 仅 **Squares** 一种模式 + 默认 pass  
2. 补 **Pick / Pick1**（pick 功能）  
3. 补 **Depth**（深度 pass）  
4. Sphere / Tile / Box / indexed_8bit  

预估工作量：约 2–4 人日（含 Ogre 14 API 调试），且需 `DISPLAY` 下 gtest/GUI 回归。

**在此之前请使用模式 A（vendor 1.12）。**

---

## CMake 选项速查

| 选项 | 默认 | 说明 |
|------|------|------|
| `AUTOVIZ_USE_OGRE` | OFF | 启用 Ogre 后端 |
| `AUTOVIZ_OGRE_VENDOR` | OFF | 源码构建 Ogre 1.12.10 |
| `AUTOVIZ_OGRE_AUTO_VENDOR` | OFF | 非 1.12 系统自动 vendor |
| `AUTOVIZ_OGRE_ROOT` | "" | 预装 Ogre 前缀（1.12 可开 GLSL） |

编译宏：

| 宏 | 含义 |
|----|------|
| `AUTOVIZ_OGRE_RVIZ_MEDIA` | 加载 rviz GLSL 脚本；否则 stub |
| `AUTOVIZ_OGRE_MEDIA_DIR` | ogre_media 根目录 |
| `AUTOVIZ_OGRE_PLUGIN_DIR` | RenderSystem_GL 插件目录 |

---

## 故障排查

| 现象 | 处理 |
|------|------|
| 配置后仍 `visual=stub` | 确认未使用 Ogre 14；加 `-DAUTOVIZ_OGRE_VENDOR=ON` 重配 |
| vendor 构建 Ogre 失败 | 检查网络与 `cmake/ogre_patches/`；见构建日志 |
| 点云全白/不可见 | 检查 `AUTOVIZ_OGRE_MEDIA_PATH`；模式 A 下看 Ogre 日志材质加载 |
| 点云是矩形无圆角 | 当前为模式 B stub；切换模式 A |
| 启动 segfault（模式 A） | 插件路径错误；`AUTOVIZ_OGRE_PLUGIN_DIR` 须指向 **1.12** 插件，勿混用 14 |
| Pick index 恒 -1 | 模式 B 下 Pick scheme 不完整；优先模式 A |

---

## 相关文档

- [RVIZ_RENDERING_PARITY.md](RVIZ_RENDERING_PARITY.md) — API 对齐总表  
- [OGRE_GUI_VERIFICATION.md](OGRE_GUI_VERIFICATION.md) — GUI Pick 验证  
- [resources/ogre_media/README.md](../resources/ogre_media/README.md) — 资源目录说明
