# OrbisView 室内 Occupancy 地图展示（对齐 BICMap 观感）

**日期：** 2026-09-14

**状态：** 已确认（路径 **B + C + A**）

**范围：** 仅改实时 `OccupancyGrid`（map / costmap）在 Map2D / View3D 的**渲染观感与性能**；参考 [BICMap](https://github.com/Open-X-Humanoid/BICMap) 室内 SLAM 展示思路，**不**引入 MapLibre / Turf / pcl.js / `@x-humanoid-cloud/bic-map`，在现有 Canvas2D + Three.js 管线上移植「canvas 纹理整图」做法。

**关联：**

- `docs/superpowers/specs/2026-09-13-orbisview-ground-robot-map2d-p1-design.md`
- `docs/superpowers/specs/2026-09-13-orbisview-view3d-design.md`
- BICMap 参考：`src/bicMap/core/layers/index.js`（`loadSlamMap`）、`src/examples/indoor/slam/index.vue`

## 1. 背景与动机

OrbisView Map2D 当前用逐格 `fillRect` 画 OccupancyGrid（`drawOccupancy.ts`），大图卡顿且配色（`#334` / `#a33`）不像常见室内 SLAM 底图。View3D 已用 `CanvasTexture` 贴地（`occupancy.ts`），但 palette 与 2D 不一致，且每帧/每更新重建策略未与 2D 共享。

BICMap 室内示例将 SLAM 图绘入离屏 canvas，再作为整幅 raster 铺到地图上，并提供「适应地图」。本阶段把同一思路落到 orbisview 的笛卡尔 OccupancyGrid 上，去掉 lon/lat 与 MapLibre。

## 2. 目标

完成后应满足：

1. **2D / 3D 共用** Occupancy → RGBA 纹理构建（同一 palette、同一缓存键）；
2. Map2D 以一次 `drawImage`（或等价）将纹理贴到世界矩形，替代逐格 `fillRect`；
3. View3D `updateOccupancyPlane` 消费同一 builder；map 与 costmap 模式配色区分；
4. **适应地图（fit）** 优先按 map（无则 costmap）的角点计算 `scale` / `viewOffset`，语义对齐 BICMap `cameraForBounds`（笛卡尔版）；
5. Map 视口底色略提亮 + 淡网格（参考 BICMap `grid-bg`），不破坏 OrbisView 整体暗色壳；
6. 测距 / 导航 / 取点 / Channels 图层开关等**现有交互不变**；
7. 前端单测覆盖 palette 边界值与纹理尺寸；`npm test` / `npm run build` 通过。

## 3. 非目标（本阶段不做）

- 安装或 vendoring `@x-humanoid-cloud/bic-map`、MapLibre、Turf、pcl.js、urdf-loader；
- 静态 PNG/BMP SLAM 文件加载 UI（BICMap `imagePath`）；
- 建图帧回放 / 进度条（`buildMap.vue`）；
- 语义区、多楼层、室外瓦片 / HD Map、POI、A* 路径规划；
- 改后端 OccupancyGrid schema 或 mock 数据契约；
- 重做工具条、仪表盘、Layout。

## 4. 方案选择

| 方案 | 结论 |
|------|------|
| ① Occupancy → ImageData/canvas 纹理整图绘制 | **采用** |
| ② 继续逐格 `fillRect` + 抽稀 | 拒绝（大图性能与观感不足） |
| ③ 专用 WebGL occupancy shader | 推迟（过重，偏离本期「移植 BICMap canvas 层」） |

约束回顾：用户确认 **B**（不引入 BICMap 第三方引擎）+ **C**（先做室内 Occupancy）+ **A**（只改实时 OccupancyGrid 展示）。

## 5. 数据契约（不变）

继续使用现有 wire JSON：

```ts
interface OccupancyGridJson {
  resolution: number;
  width: number;
  height: number;
  origin: { x: number; y: number; yaw?: number };
  data: number[]; // 行主序，ROS 惯例：-1 unknown, 0 free, 1–100 occupied
}
```

通道仍由 Channels / `effectiveMapLayers` 驱动；本设计不改订阅。

## 6. 架构与数据流

```
OccupancyGrid envelope (map | costmap)
        │
        ▼
 occupancyTexture.buildOccupancyImage(grid, mode)
   · palette: map | costmap
   · ImageData → HTMLCanvasElement（或 OffscreenCanvas）
   · 缓存：key = mode + w + h + res + origin + dataRef/generation
        │
        ├─► Map2D drawOccupancy：world 矩形 + drawImage
        └─► View3D updateOccupancyPlane：CanvasTexture + PlaneGeometry
```

坐标：保持 orbisview 笛卡尔世界系（米）；**不**调用 BICMap `cartesianToGPS`。

角点（与 BICMap `getMapCorners` 语义对齐，实现自写）：

- `origin = (ox, oy)`，宽 `W = width * resolution`，高 `H = height * resolution`
- 矩形：`(ox, oy)` → `(ox+W, oy)` → `(ox+W, oy+H)` → `(ox, oy+H)`  
  （与当前 2D `drawOccupancy` / 3D plane 中心放置一致；若 yaw≠0 本期忽略旋转，与现状相同）

## 7. 配色（对齐 BICMap 室内 SLAM 观感）

目标：浅底上「未知灰、自由近透明、占据深色」；costmap 暖色半透明叠层。

| 模式 | 条件 | RGBA（建议初值，实现时可微调） |
|------|------|--------------------------------|
| map | `v < 0` unknown | `(160, 168, 176, 110)` |
| map | `v === 0` free | `(0, 0, 0, 0)` 全透明 |
| map | `v >= 100` occupied | `(40, 44, 52, 230)` |
| map | `0 < v < 100` | 按占用概率在 free→occupied 间插值 alpha |
| costmap | `v < 0` | 跳过（透明） |
| costmap | `v >= 100` lethal | `(255, 112, 67, 140)` |
| costmap | 其它 | `(255, 167, 38, 90)` 量级 |

3D 与 2D **必须**同一函数输出，避免双面板色差。

降采样：当 `max(width,height) > 1024` 时，可按边长上限（建议 1024，3D 现状 512 可统一为 1024 或参数化）最近邻缩略，保证帧率；缩略只影响显示分辨率，不影响世界尺寸。

## 8. 缓存与更新

- Panel / sync 传入的 `grid` 对象引用或 `data` 数组引用变化时重建纹理；
- 同一引用不重复 `createImageData`；
- 切换 layer 显隐不销毁纹理，仅跳过绘制 / `plane.visible`；
- map 与 costmap **各一份**缓存（mode 不同）。

## 9. 适应地图

`Map2DPanel.fitView`（及工具条「适应」）：

1. 若存在 map 且 `layers.map`：用 map 角点；
2. 否则若存在 costmap 且 `layers.costmap`：用 costmap 角点；
3. 否则回退现有逻辑（pose / path / laser 包围盒等）。

计算：世界 AABB + padding → `scale` / `viewOffset` 使矩形落入视口（与 BICMap padding≈30px 同级，可用比例 padding 如 1.08）。

View3D：本期不强制改相机 fit（可选 follow 已有）；若工具条 fit 在 3D 模式已有行为则保持，不阻塞 2D。

## 10. 视口样式

- `.map-viewport` / `.map-canvas`：略提亮底（仍偏暗主题可读），叠加低透明度 CSS 网格（参考 BICMap `.grid-bg` 的 40px 线网，颜色改用现有蓝灰 token）；
- 不引入 BICMap 的亮色全屏渐变壳；
- 绘制层内世界网格（`layers.grid`）逻辑保留。

## 11. 文件改动清单

| 路径 | 动作 |
|------|------|
| `frontend/src/renderer/map2d/occupancyTexture.ts` | **新增**：palette + build + cache API |
| `frontend/src/renderer/map2d/occupancyTexture.test.ts` | **新增**：unknown/free/occupied、尺寸、缓存命中 |
| `frontend/src/renderer/map2d/drawOccupancy.ts` | **改**：调用纹理 `drawImage` |
| `frontend/src/renderer/view3d/layers/occupancy.ts` | **改**：共用 builder |
| `frontend/src/renderer/map2d/drawScene.ts` | **改**（若签名需传 cache 句柄；尽量无感） |
| `frontend/src/components/Map2D/Map2DPanel.tsx` | **改**：`fitView` 角点优先 |
| `frontend/src/styles/main.css` | **改**：map 视口底 + 淡网格 |
| BICMap 源码 | **不拷贝进仓库**；仅逻辑对齐，注释可注明参考路径 |

## 12. 测试与验收

**自动：**

- `occupancyTexture.test.ts`：边界值像素色、空 grid、长宽不一致 warn 行为保持；
- 现有 `npm test` / `npm run build` 全绿。

**手动（mock）：**

1. 打开地面机布局，订阅 mock map：底图应为灰未知 + 透明自由 + 深色墙，非大红块；
2. 大图（若 mock 较大）平移/缩放流畅，无明显逐格卡顿；
3. 叠 costmap：暖色半透明，不遮死 map；
4. 点「适应」：整图落入视口；
5. 2D↔3D 切换：地图色调一致；测距/导航仍可用。

## 13. 风险与缓解

| 风险 | 缓解 |
|------|------|
| 每帧新 `data[]` 导致每帧重建纹理 | 用长度+采样指纹或后端稳定引用；必要时节流（如 5–10 Hz 重建上限） |
| 降采样锯齿 | `NearestFilter` + 文档说明；后续可加开关 |
| 暗色 UI 上全透明 free「看不见地图」 | unknown 保留可见灰底；自由透明露出视口网格 |

## 14. 后续（明确不在本期）

- 静态 SLAM 图加载（BICMap `loadSlamMap` 完整参数面）；
- 建图过程可视化；
- 语义区 / 多楼层 / 室外。
