# OrbisView frontend

Vite + React 前端。WS 载荷为 JSON。

```text
frontend/
├── CMakeLists.txt    # npm ci/build + install → share/.../www
├── assets/           # Vite publicDir（plugins 清单等）
├── package.json
├── vite.config.ts
└── src/
    ├── app.tsx
    ├── config/parameters.ts
    ├── components/
    ├── store/
    ├── renderer/map2d/
    ├── renderer/view3d/
    ├── plugins/
    └── styles/main.css
```

## 构建 / 安装

通常随 autonomy CMake 一起构建（见上级 `orbisview/README.md`）：

```bash
cmake --build build/autonomy --target orbisview_frontend -j
# 或随二进制：--target autonomy.orbisview
cmake --install build/autonomy   # → share/autonomy/orbisview/www
```

也可本地手动：

```bash
cd autonomy/orbisview/frontend
npm ci
npm run build      # → dist/
npm test
```

## 运行（开发）

```bash
npm start          # Vite :5173
```

浏览器 http://127.0.0.1:5173 → Connect → `ws://127.0.0.1:8766/ws`。

Map2D：costmap / footprint / DIFF HUD（`renderer/map2d`）。

View3D（`renderer/view3d`）：
1. Catalog 打开 View3D → Follow / Free / Reset
2. Layers 与 Map2D 共用：grid / robot / path / pointcloud / footprint / map / costmap / laser
3. 工具条：cloudColor、laserHeight、mapOpacity；点云 stale 徽章
