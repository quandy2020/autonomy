# OrbisView frontend

工程布局对齐 Apollo Dreamview `frontend/`（`assets` / `config` / `proto_bundle` / `setup.sh` / `src/{components,store,renderer,styles,utils}`）。
构建工具为 **Vite**（DV 为 webpack），路径别名与 DV `jsconfig` 一致。

```text
frontend/
├── assets/           # 静态资源（Vite publicDir）
├── config/           # parameters.js
├── proto_bundle/     # gen_pbjs 产物
├── gen_pbjs.sh
├── setup.sh
├── package.json      # start / build（对标 DV scripts）
├── jsconfig.json
├── vite.config.ts
└── src/
    ├── app.tsx       # 入口（对标 app.js）
    ├── components/   # Orbisview.tsx + panels
    ├── store/        # zustand + websocket/
    ├── renderer/
    ├── styles/main.css
    ├── utils/
    └── fonts/
```

## 运行

```bash
cd autonomy/orbisview/frontend
npm install
npm start          # setup + Vite :5173
# 或整栈：npm run dev:all
```

浏览器 http://127.0.0.1:5173 → **Connect** → `ws://127.0.0.1:8766/ws`。

Map2D P1：costmap 叠层、可流式 footprint、DIFF/ACKERMANN HUD（`renderer/map2d`）。

```bash
npm run build      # dist/，勿提交
npm test
```
