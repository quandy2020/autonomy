# OrbisView frontend

Vite + React 前端。WS 载荷为 JSON。

```text
frontend/
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

## 运行

```bash
cd autonomy/orbisview/frontend
npm install
npm start          # Vite :5173
# 或整栈：npm run dev:all
```

浏览器 http://127.0.0.1:5173 → Connect → `ws://127.0.0.1:8766/ws`。

Map2D：costmap / footprint / DIFF HUD（`renderer/map2d`）。

View3D（`renderer/view3d`）：
1. Catalog 打开 View3D → Follow / Free / Reset
2. Layers 与 Map2D 共用：grid / robot / path / pointcloud / footprint / map / costmap / laser
3. 工具条：cloudColor、laserHeight、mapOpacity；点云 stale 徽章

```bash
npm run build
npm test
```
