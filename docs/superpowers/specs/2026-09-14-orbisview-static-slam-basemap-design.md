# OrbisView 静态 SLAM 底图加载（P1）

**日期：** 2026-09-14

**状态：** 已确认（路径 **B + 叠层 1 + 加载 C + 参数 C**；总路线 P1→P2→P3→P4 之 **P1**）

**范围：** 在现有 Map2D / View3D 上增加**静态 SLAM / Occupancy 图像底图**加载与渲染；与实时 OccupancyGrid（map / costmap）**分层叠放**；不引入 MapLibre / Turf / `@x-humanoid-cloud/bic-map`。

**关联：**

- `docs/superpowers/specs/2026-09-14-orbisview-indoor-occupancy-display-design.md`（A 期纹理管线）
- BICMap 参考：`src/bicMap/core/layers/index.js`（`loadSlamMap`）、`src/examples/indoor/slam/index.vue`
- 仓库内样例图：`autonomy/map/conf/turtlebot3_*.yaml` + `.pgm`、`simulation/maps/cave.png`

## 1. 背景与动机

A 期已将实时 OccupancyGrid 改为 canvas 纹理 blit。真机/仿真联调时常需先铺一张**已建好的 SLAM 图**（PNG/PGM/BMP），再叠实时 local map / costmap。BICMap 用 `loadSlamMap(imagePath + startX/Y + resolution + gridCount)`；orbisview 需在笛卡尔坐标系下提供等价能力，并兼容 Nav2 `map_server` YAML。

## 2. 目标

1. 支持加载静态图：**本地文件选择**、**URL**、**打包/已知资源快捷项**；
2. 几何参数来自 **YAML/JSON sidecar** 或 **表单手填**（可持久化上次手填）；
3. 静态底图作为最底层绘制；其上仍为实时 `map` → `costmap`（及现有机器人/激光等）；各层可独立开关；
4. Map2D 与 View3D **均**渲染静态底图；
5. 「适应」在存在静态底图时优先按其角点（再回退 A 期 map/costmap 逻辑）；
6. 单测覆盖 sidecar 解析与角点计算；`npm test` / `npm run build` 通过。

## 3. 非目标（本 P1 不做）

- 建图帧回放 / 进度条（**P2**）；
- 语义区 / 多楼层（**P3**）；
- POI / 交互绘制（**P4**）；
- MapLibre、GPS/`cartesianToGPS`、Turf；
- 改后端 OccupancyGrid schema；从后端推静态图字节流（可后续加 channel）。

## 4. 方案选择

| 方案 | 结论 |
|------|------|
| ① 静态图 → 离屏 canvas → 与 Occupancy 同套 blit | **采用** |
| ② DOM/CSS 背景图 | 拒绝（坐标与 3D 不对齐） |
| ③ 解码为假 OccupancyGrid 抢 map 通道 | 拒绝（与实时 map 冲突） |

用户锁定：

- **叠层**：静态底图 + 实时 map/costmap 可分别开关；
- **加载**：文件选择器 + URL/内置资源；
- **参数**：sidecar 优先，否则表单。

## 5. 数据契约

### 5.1 运行时规格

```ts
interface StaticSlamBasemap {
  /** Object URL or http(s)/相对路径；blob 会话内有效 */
  imageSrc: string;
  originX: number;
  originY: number;
  /** 米/像素 */
  resolution: number;
  /** 像素；缺省由图片 naturalWidth/Height 推断 */
  widthPx: number;
  heightPx: number;
  /** 可选显示名 */
  label?: string;
  /** 来源标记 */
  source: 'file' | 'url' | 'asset';
}
```

世界矩形（与 A 期 Occupancy 一致，忽略 yaw）：

- `W = widthPx * resolution`，`H = heightPx * resolution`
- 角点：`(originX, originY)` → `(originX+W, originY+H)`

### 5.2 Sidecar（YAML / JSON）

优先识别 Nav2 / ROS `map_server` 字段：

```yaml
image: turtlebot3_world.pgm
resolution: 0.05
origin: [-10.0, -10.0, 0.0]   # [x, y, yaw]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
```

兼容 BICMap / 显式别名（JSON 或 YAML）：

```json
{
  "startX": -58.99,
  "startY": -21.35,
  "resolution": 0.05,
  "xGridCount": 2752,
  "yGridCount": 1536,
  "image": "slam.png"
}
```

解析规则：

| 字段 | 映射 |
|------|------|
| `origin[0]`,`origin[1]` | `originX`,`originY` |
| `startX`,`startY` | 同上（若无 origin） |
| `resolution` | 必填（表单可补） |
| `xGridCount`/`yGridCount` 或图片尺寸 | `widthPx`/`heightPx` |
| `image` | 相对 sidecar 的文件名（本地多选时匹配；URL 模式拼目录） |

`negate` / thresh：P1 **不**做二值化重采样；原图像素直绘（透明 PNG 保留 alpha）。PGM 由浏览器/`createImageBitmap` 或 canvas 解码；若浏览器无法直接显示 PGM，P1 实现轻量 PGM→ImageData 解析（常见 ASCII/binary P5）。

### 5.3 Persist

- zustand persist（建议 key `orbisview-static-slam-v1`）：保存 `origin*`、`resolution`、`widthPx`/`heightPx`、`label`、`source`、**非 blob** 的 `imageSrc`（URL/asset path）；
- `source==='file'` 的 Object URL **不**跨会话恢复；刷新后需重新选文件，但可恢复上次表单参数。

## 6. 架构与数据流

```
[文件选择 / URL / asset]
        │
        ├─► 可选 sidecar 解析 ──┐
        │                       ▼
        └─► 表单补全 ──► StaticSlamBasemap store
                              │
                              ▼
              buildStaticSlamCanvas(image)  → HTMLCanvasElement
                              │
              ├─ Map2D drawScene：最先 drawImage（layers.basemap）
              └─ View3D：独立 plane（yLift 低于 map plane）
                              │
              其上：现有 map / costmap / robot / …
```

图层开关：新增 `LayerKey` **`basemap`**（默认 `true`），写入 `useLayerStore`（persist 版本 bump，如 `orbisview-layers-v6`，旧状态缺省补 `basemap: true`）。

Channels / `effectiveMapLayers`：`basemap` **不**依赖 channel；始终由 layer store + 是否已加载规格决定。

## 7. UI

- `MapFloatToolbar`（2D/3D 共用）：增加「加载底图」按钮 → 隐藏 `<input type="file" accept="image/*,.pgm,.yaml,.yml,.json" multiple>`；
  - 若同时选中图 + yaml/json → 自动解析；
  - 仅图 → 打开参数小面板（origin / resolution / 可选宽高）；
  - 另提供「从 URL 加载」与可选内置快捷（如指向 `…/turtlebot3_world` 的开发资源，若前端可访问；否则文档注明需用户本地选仓库文件）。
- 清除底图按钮（有 basemap 时显示）。
- 状态栏一句：`底图 12.0×12.0 m · res 0.05`。

不新增独立 Mosaic 面板（避免 P1 膨胀）；参数面板为 Map 浮层/popover。

## 8. 渲染细节

- **2D**：在 `drawScene` 中，世界网格之后、实时 map 之前绘制静态 canvas；Y 翻转与 A 期 Occupancy blit 一致。
- **3D**：`basemapPlane`，`yLift ≈ 0.005`（低于 map `0.01`）；`NearestFilter`；透明度跟图片 alpha。
- **着色**：静态图**不做** Occupancy `-1/0/100` 重映射；原图像素上屏（贴近 BICMap 直接贴 SLAM PNG）。
- 大图：最长边可降采样至与 A 期相同上限（1024）仅用于 GPU 纹理；世界尺寸仍用全分辨率 `widthPx * resolution`。

## 9. Fit

`fitView` 优先级：

1. `basemap` 已加载且 `layers.basemap` → 静态角点；
2. 否则 A 期：map → costmap → pose/waypoints 回退。

## 10. 文件清单

| 路径 | 动作 |
|------|------|
| `frontend/src/renderer/map2d/staticSlam.ts` | 解析 sidecar、角点、canvas 构建 |
| `frontend/src/renderer/map2d/staticSlam.test.ts` | 解析与角点单测 |
| `frontend/src/renderer/map2d/pgmDecode.ts` |（若需要）PGM→ImageData |
| `frontend/src/store/staticSlamStore.ts` | basemap 状态 + persist |
| `frontend/src/store/layoutStore.ts` | `LayerKey` + `basemap` |
| `frontend/src/renderer/map2d/drawScene.ts` | 绘静态层 |
| `frontend/src/renderer/view3d/layers/basemap.ts` | 3D plane |
| `frontend/src/renderer/view3d/createScene.ts` / `syncScene.ts` | 挂 plane |
| `frontend/src/components/Map/MapFloatToolbar.tsx` | 加载/清除入口 |
| `frontend/src/components/Map/StaticSlamLoadPop.tsx` | 表单 / URL popover |
| `frontend/src/components/Map2D/Map2DPanel.tsx` | 接线 + fit |
| `frontend/src/components/View3D/View3DPanel.tsx` | 接线 |
| `frontend/src/components/Channels/mapDisplayBinding.ts` | `effectiveMapLayers` 透传 `basemap`（若需要） |

## 11. 测试与验收

**自动：**

- sidecar：Nav2 yaml `origin`/`resolution`；BICMap 别名 JSON；
- 角点：`origin + size * res`；
- PGM 小样例解码（若实现 pgmDecode）。

**手动：**

1. 加载 `turtlebot3_world.pgm` + yaml → 底图与尺度合理；
2. 叠 mock 实时 map/costmap，开关 `basemap` / `map` 互不影响；
3. URL 加载一张 PNG；
4. Fit 框住静态图；
5. 2D↔3D 底图位置一致；测距仍可用。

## 12. 风险

| 风险 | 缓解 |
|------|------|
| 浏览器不显示 PGM | 自研轻量解码 |
| Object URL 泄漏 | `revokeObjectURL` 在替换/清除时 |
| persist 恢复失效 blob | 仅恢复参数，提示重新选文件 |
| 与 Channels「map」概念混淆 | UI 文案用「底图 / Basemap」，layer key `basemap` |

## 13. 后续

- **P2** 建图过程可视化（可复用静态 canvas 更新通路）
- **P3** 语义区 / 多楼层
- **P4** POI / 绘制工具
