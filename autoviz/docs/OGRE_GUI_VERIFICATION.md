# Ogre 视口 GUI 验证指南

> 适用于 `-DAUTOVIZ_USE_OGRE=ON` 构建，验证 PointCloud2 / LaserScan pick 与持久 Ogre 对象。

## 构建

依赖系统 **Ogre 14.4+**（`pkg-config OGRE OGRE-Overlay`）。点云与 rviz **视觉对齐**需 Ogre 1.12，见 [OGRE_VISUAL_ALIGNMENT.md](OGRE_VISUAL_ALIGNMENT.md)。

```bash
# 默认：Ogre 14 + stub 点云材质
cmake -S src/autonomy -B build/autonomy -DBUILD_AUTOVIZ=ON -DAUTOVIZ_USE_OGRE=ON -DBUILD_AUTOVIZ_TESTS=ON

# rviz 一致点云 GLSL（vendor Ogre 1.12，首次较慢）
cmake -S src/autonomy -B build/autonomy -DBUILD_AUTOVIZ=ON -DAUTOVIZ_USE_OGRE=ON \
  -DAUTOVIZ_OGRE_VENDOR=ON -DBUILD_AUTOVIZ_TESTS=ON

cmake --build build/autonomy --target autoviz autoviz_ogre_pick_verify
```

## 自动化（无 GUI）

```bash
./build/bin/autoviz_ogre_pick_verify
```

验证 `PickRegistry::lookupByDisplayAndPointIndex` 与 handle RGB 编解码（Ogre Pick1 解析依赖此逻辑）。

## GUI 验证：PointCloud2 Pick

### 1. 启动 Aviz（Ogre 视口）

```bash
export AUTOVIZ_OGRE_MEDIA_PATH=$(pwd)/src/autonomy/autoviz/resources/ogre_media
export AUTOVIZ_OGRE_PLUGIN_DIR=/usr/local/lib/OGRE   # 按系统调整
./build/bin/autoviz
```

在 **View** 面板选择 **Ogre** 视口（非 OpenGL 视口）。

### 2. 添加 Display

| 步骤 | 操作 |
|------|------|
| 1 | Add Display → **PointCloud2**，Topic 填点云 channel |
| 2 | Add Display → **LaserScan**（可选），Topic 填激光 channel |
| 4 | Add Display → **Path** / **TF** / **Marker**（可选） |
| 5 | Fixed Frame 设为 `base_link` |

### 3. 发布测试数据

```bash
python3 src/autonomy/autoviz/scripts/publish_test_sensors.py
```

默认发布：

- `/test/pointcloud` — 5 个沿 X 轴的点（index 0..4）
- `/test/laserscan` — 180° 扇形扫描
- `/test/path` — 折线路径
- `/test/marker` — ARROW + CUBE + SPHERE + TEXT_VIEW_FACING（ids 1–4，每周期同时发布）

在 Display 中将 Topic 设为上述 channel。

### 4. Pick 验证

| 检查项 | 预期 |
|--------|------|
| 点云可见 | 模式 A：`rviz/PointCloudSquare` 圆角方形；模式 B（Ogre 14 默认）：stub 矩形 quad |
| 选择工具点击第 i 个点 | Selection 面板显示 `Point Index: i` |
| 不同点 | index 随点击位置变化（0→4） |
| LaserScan | 同上，点击扫描线上的不同位置 |

### 5. 阶段 3 形状（Line / Arrow）

| Display | Ogre 路径预期 |
|---------|---------------|
| **Axes** / **TF** | 持久 Ogre 线段（RGB 轴） |
| **Path** | 折线；`line_width>0` 时 Ogre `BillboardLine` 宽线 |
| **Pose** | Ogre 线段 + 位置点 |
| **Marker** (ARROW/LINE_STRIP/POINTS) | Ogre 线段/点云 |
| **Marker** (CUBE/SPHERE/CYL/MESH) | Ogre ManualObject 实心/线框 mesh |
| **Marker** (ARROW, solid) | Ogre cylinder + cone 实心箭头 |
| **RobotModel** (solid, 无 PBR) | Ogre 持久 link mesh + 轴线段 |
| **RobotModel** (PBR/纹理) | Ogre 持久 PBR mesh（`AvizPBR` / `AvizPBRTextured`） |
| **Marker** (TEXT_VIEW_FACING) | Ogre `OgreMovableText`（Liberation Sans） |

## 故障排查

| 现象 | 可能原因 |
|------|----------|
| 点云不可见 | `AUTOVIZ_OGRE_MEDIA_PATH` 未设；检查终端 Ogre 材质加载错误 |
| Pick 无响应 | 未选 Ogre 视口；GPU pick 需硬件 OpenGL |
| Pick index 恒为 -1 | Pick1 pass 失败；确认 `rviz/PointCloudSquare` 含 `Pick1` technique |
| 回退 GL pick | Ogre pick 未命中时会读 GlPickFramebuffer（overlay 有 pick 几何时） |

## 相关文件

- `autoviz/rendering/ogre_pick_renderer.*` — Ogre RenderTexture Pick/Pick1
- `autoviz/display/ogre_colored_points_draw.*` — 点类 Display 共享路径
- `autoviz/display/ogre_overlay_draw.*` — Line/Arrow 共享路径
- `autoviz/display/ogre_mesh_draw.*` — Mesh primitive 共享路径
- `autoviz/display/ogre_label_draw.*` — MovableText 标签共享路径
- `autoviz/display/ogre_pbr_mesh_draw.*` — PBR mesh 共享路径
- `autoviz/rendering/objects/ogre_movable_text.*` — rviz MovableText 移植
- `autoviz/scripts/publish_test_sensors.py` — 测试数据发布
