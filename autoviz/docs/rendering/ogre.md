# Ogre：视觉对齐与验证

视口 **必须** 使用 Ogre 1.x。点云、Pick、Depth 依赖 `resources/ogre_media/`。

默认 `AUTOVIZ_OGRE_AUTO_VENDOR=ON`：系统若不是 **1.12.x**，自动 FetchContent **1.12.10**，以加载 rviz GLSL 1.20 材质。

## 模式

| 模式 | 条件 | 点云外观 | Pick / Depth |
|------|------|----------|--------------|
| **A · rviz GLSL** | Ogre 1.12.x（vendor / ROOT） | 圆角方形 billboard | 完整 |
| **B · stub** | 仅系统 Ogre 14 且关闭 auto-vendor | 平面 quad | 基础可见 |

日志确认：

```text
Autoviz RenderSystem ready (... visual=aviz_glsl ...)
Autoviz RenderSystem ready (... visual=stub ...)
```

## 强制 vendor 1.12

```bash
cmake -S . -B build -DAUTOVIZ_OGRE_VENDOR=ON
cmake --build build --target autoviz_app
```

或：`AUTOVIZ_OGRE_ROOT` 指向已安装的 1.12 前缀。

## GUI 抽检

1. `./build/bin/autoviz`
2. 添加 PointCloud2 / LaserScan，Fixed Frame 设为数据坐标系
3. 选择工具点击点：Selection 应显示 Point Index

```bash
export AUTOVIZ_OGRE_MEDIA_PATH=$PWD/resources/ogre_media
# vendor 构建时插件目录由 CMake 定义；系统包示例：
# export AUTOVIZ_OGRE_PLUGIN_DIR=/usr/local/lib/OGRE
```

## 相关文档

- [后端策略](backends.md) · [渲染对齐](../parity/rendering.md)
