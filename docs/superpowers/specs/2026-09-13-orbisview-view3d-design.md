# OrbisView 地面机 View3D 设计

**日期：** 2026-09-13

**状态：** 已确认（方案 ①：抽层 Three.js；单阶段交付图层 1–6）

**范围：** 将现有单体 `View3DPanel` 抽到 `renderer/view3d/`，补齐地面机 3D 主视口：网格/姿态、路径·目标、点云、footprint、occupancy·costmap 贴地、LaserScan 3D；相机默认跟随可切自由轨道；图层与 Map2D 共用主开关并带 3D 子选项。

**关联：**

- `docs/superpowers/specs/2026-09-13-orbisview-ground-robot-map2d-p1-design.md`
- `docs/superpowers/specs/2026-09-13-orbisview-layered-migration-design.md`

## 1. 背景与动机

OrbisView 已有 Map2D P1（`renderer/map2d/` + footprint/costmap/HUD）与粗糙的 `View3DPanel`（Three.js 内联：网格、锥体机器人、路径、goal、点云、手动绕原点轨道相机）。缺口：

- 绘制与生命周期堆在 Panel，无法单测坐标/图层更新；
- 无 footprint、occupancy/costmap、LaserScan 的 3D 表现；
- 相机只绕世界原点，不跟随机器人；
- 点云 wire JSON（`points[{x,y,z,i}]`）与 `render.proto` 的 `xyz` 字段表述不一致。

本设计对标 Map2D 的「Panel 订阅 + renderer 抽层」，不做 Dreamview 车模/HD Map 对齐。

## 2. 目标

单阶段完成后应满足：

1. View3D 绘制逻辑落在 `frontend/src/renderer/view3d/`，`View3DPanel` 只负责订阅、挂载、交互与 opts；
2. 图层：**网格 + 机器人姿态、路径 + 导航目标、点云、footprint、occupancy/costmap 贴地平面、LaserScan 3D**；
3. 相机：**默认跟随**机器人，可切换 **自由轨道**；支持 Reset view；
4. 图层主开关与 Map2D **共用** `useLayerStore`；3D 独有子选项（点云着色、laser 高度、地图透明度）单独持久化；
5. 继续使用已有 `three` 依赖，**不引入** R3F / Babylon；
6. 前端以 wire JSON `points[]`（可选 `i`）为点云契约；proto 注释与之一致；mock / Autolink converter 行为保持；
7. `npm test` + `npm run build` 通过；手动 mock 验收清单见 §7。

## 3. 非目标

- React Three Fiber、Babylon、车模 GLTF、多相机、点云点选/量测；
- 新建专用 Costmap proto 或提高点云上限（后端仍约 500 点采样）；
- 多楼层、充电桩 POI、与 Dreamview 资源对齐；
- 改 Autolink 订阅配置 UI 或真机 bag 联调（converter 已有路径即可）。

## 4. 方案选择

| 方案 | 结论 |
|------|------|
| ① 抽层 Three.js（`renderer/view3d`） | **采用** |
| ② React Three Fiber + drei | 拒绝（新栈、与现 Panel 不一致） |
| ③ 换 Babylon.js | 拒绝（重写成本无必要） |

交付节奏：**单阶段**做满图层 1–6（用户确认 A）。

## 5. 架构

```
View3DPanel (订阅 envelopes / layers / view3dOpts / 指针交互)
    ↓
renderer/view3d/
  createScene.ts       — WebGLRenderer / Scene / Lights / dispose
  cameraController.ts  — follow | free；yaw/pitch/distance；lookAt
  coords.ts            — map(x,y,z) → Three(x, z||0, −y)
  syncScene.ts         — View3DSceneInput → 更新各图层
  layers/*.ts          — grid, robot, path, goal, cloud, footprint,
                         occupancy, laser
  types.ts             — 输入类型与 opts
```

**边界：**

- Panel：DOM、`ResizeObserver`、pointer/wheel、读 store，组装 `View3DSceneInput`；
- Renderer：纯 Three 对象创建/更新/销毁，不依赖 React/Zustand；
- 数据：只消费现有 `orbisview.render.*` schema；
- 相机模式与已有 `useLayerStore.followRobot` 对齐（单一来源），避免双状态。

## 6. 模块与数据流

### 6.1 数据流

```
WS envelope → dataStore
                ↓
View3DPanel 选取：
  Pose2D, Path2D, Navigation(goal),
  PointCloud2, RobotFootprint,
  OccupancyGrid×2（map / costmap，规则同 Map2D pickMapAndCostmap）,
  LaserScan
                ↓
View3DSceneInput + layer flags + view3dOpts
                ↓
syncScene → rAF render
```

### 6.2 图层映射

| LayerKey / 数据 | 3D 表现 |
|-----------------|---------|
| `grid` | `GridHelper` |
| `robot` + Pose2D | 锥体/箭头 + yaw |
| `path` + Path2D | `Line`（贴地 y≈0.05） |
| Navigation goal | 橙色锥体 + 虚线（有 goal 即显示，与现逻辑一致） |
| `pointcloud` | `Points`；着色见 `view3dOpts.cloudColor` |
| `footprint` | Shape 挤出或贴地 `LineLoop`；无数据时用默认矩形 footprint |
| `map` / `costmap` | 贴地 `Plane` + `CanvasTexture`；costmap 更高半透明 |
| `laser` | 圆环/射线 `Points` 或 `LineSegments`，高度 `laserHeight` |

### 6.3 view3dOpts（持久化）

建议字段（名称可在实现时微调，语义固定）：

| 字段 | 含义 | 默认 |
|------|------|------|
| `cloudColor` | `'height' \| 'intensity'` | `'intensity'` |
| `laserHeight` | Laser 抬升高度 (m) | `0.1` |
| `mapOpacity` | 贴地 occupancy 透明度；costmap 实际使用 `mapOpacity * 0.85` | `0.55` |

相机跟随使用已有 `followRobot` / `setFollowRobot`，不另开平行开关。

### 6.4 点云契约

- **Wire JSON（权威）：** `{"points":[{"x":number,"y":number,"z":number,"i"?:number},...]}`  
  与 `mock_source.cc`、`ConvertPointCloud2` 现状一致。
- **`render.proto`：** 将 `PointCloud2` 注释/字段说明改为与 wire 一致（允许保留或废弃未使用的 `xyz` 表述，以避免前端双解析路径）。
- 前端不二次降采样；依赖后端约 500 点上限。

### 6.5 坐标

与现 `View3DPanel` 一致：

\[
(x_t, y_t, z_t) = (x_{\mathrm{map}},\, z_{\mathrm{map}}\ \mathrm{or}\ 0,\, -y_{\mathrm{map}})
\]

集中在 `coords.ts`，Map2D 与 3D 语义对齐（平面 x/y，3D 把 map-y 映到 Three −z）。

### 6.6 相机

| 模式 | 行为 |
|------|------|
| Follow（默认，`followRobot=true`） | lookAt = 机器人位置；拖拽改相对 yaw/pitch；滚轮改 distance |
| Free | lookAt 可平移（或固定世界点）；拖拽轨道；滚轮缩放 |
| Reset | 恢复默认 pitch/distance（及 free 下的目标点） |

### 6.7 Occupancy 性能

- 仅当对应 layer 开启时构建/更新纹理；
- Canvas 最长边上限 **512**（可调常量），避免大图卡顿；
- map 与 costmap 分两张 plane，z 略抬升避免 z-fighting（如 map `0.01`、costmap `0.02`）。

## 7. 验收

**手动（mock：`autonomy.orbisview --mock=true` + Vite Connect）：**

1. View3D 画布铺满面板，resize 不变形；
2. Connect 后可见网格、机器人、路径、goal、环形点云；
3. 关闭 `pointcloud` / `path` / `robot` / `grid` / `footprint` / `map` / `costmap` / `laser` → 对应对象消失，且与 Map2D 主开关同步；
4. Follow：机器人移动时相机目标跟随；切 Free 可绕转/缩放；Reset 恢复默认；
5. `cloudColor`、`mapOpacity`、`laserHeight` 切换有可见效果；
6. 点云 stale 时显示 stale 提示；
7. `npm test` && `npm run build` 通过。

**建议单测：** `coords.ts` 往返；可选 footprint 顶点 → Shape 路径。

## 8. 风险与对策

| 风险 | 对策 |
|------|------|
| Occupancy 纹理卡顿 | 最长边 ≤512；layer 关则不更新 |
| Laser + 点云视觉杂乱 | laser 默认略抬高；可关图层 |
| Follow/Free 状态分叉 | 只用 `followRobot` |
| Proto vs wire 不一致 | proto 对齐 wire `points[]`，不动 converter |

## 9. 实现顺序

1. `renderer/view3d` 骨架（createScene + coords + sync 空壳）+ Panel 挂载/resize；
2. `cameraController`（follow / free / reset）接 `followRobot`；
3. 迁移现有 robot / path / goal / cloud / grid；
4. footprint 层；
5. map / costmap 贴地纹理；
6. laser 层；
7. view3dOpts UI + Layers 侧或面板工具条；
8. proto 注释对齐 + README 验收说明；
9. 按 §7 验收。

## 10. 决策记录

| 议题 | 选择 |
|------|------|
| 产品形态 | 地面机主视口 + 传感器调试能力，单阶段做满 |
| 图层范围 | 网格/姿态、路径/目标、点云、footprint、occupancy/costmap、LaserScan |
| 相机 | 默认跟随，可切自由轨道 |
| 图层开关 | 共用 `useLayerStore` + 3D 子选项 |
| 技术栈 | 抽层 Three.js，不引入 R3F/Babylon |
| 点云契约 | wire `points[]` 为准 |
