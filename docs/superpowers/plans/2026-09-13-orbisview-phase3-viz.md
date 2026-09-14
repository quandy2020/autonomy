# OrbisView Phase 3 Implementation Plan

> Spec §12 阶段三。概念移植 Dreamview Plus 面板壳，**不**搬 carviz/Cyber。

**Goal:** 浏览器可组合面板显示 map / pose / path / TF / laser / image；布局与图层可持久化。

### Task 1: 后端 mock 渲染流
- [x] 扩展 MockSource：map、tf、laser、image
- [x] 稳定 JSON 渲染协议（前端直接消费）

### Task 2: Vite 前端壳
- [x] package.json + vite + React + TS
- [x] WS client、panel registry、layout/layer localStorage

### Task 3: 面板
- [x] Map2D、View3D（three）、Image、ChannelLog
- [x] 图层开关 + 视角控制（3D 固定相机；2D 原点居中）

### Task 4: 验证
- [x] `npm install && npm run build` / `npm test`
- [x] mock map envelope smoke `P3_SMOKE_OK`
