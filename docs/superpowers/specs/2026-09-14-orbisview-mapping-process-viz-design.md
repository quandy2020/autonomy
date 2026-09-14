# OrbisView 建图过程可视化（P2）

**日期：** 2026-09-14

**状态：** 已确认（路径 **B + 模式 C + UI C**；总路线 P1→P2→P3→P4 之 **P2**）

**范围：** 在 P1 静态底图通路上增加**建图过程可视化**：**演示回放**（本地/URL 帧序列）与 **实时建图**（`Mapping` + 实时 OccupancyGrid）；Map 精简 HUD + Mapping 面板完整控制；不引入 MapLibre / BICMap npm。

**关联：**

- `docs/superpowers/specs/2026-09-14-orbisview-static-slam-basemap-design.md`（P1 底图）
- `docs/superpowers/specs/2026-09-14-orbisview-indoor-occupancy-display-design.md`（实时 Occupancy 纹理）
- BICMap 参考：`src/examples/indoor/slam/buildMap.vue`（帧序列、`applyStage`、进度与状态条）
- 数据：`orbisview.render.Mapping`（`render.proto`）、mock `/orbisview/mock/mapping`

## 1. 背景与动机

P1 已支持单次静态 SLAM 底图加载。BICMap `buildMap` 演示通过**逐帧更新同一 canvas 源**展示建图过程，并附带播放控制与状态 HUD。OrbisView 需在笛卡尔 Map2D/View3D 上提供等价体验，同时对接 mock/Autolink 的 **Mapping** 状态与**实时 map** 增量，而不把演示帧伪装成 OccupancyGrid channel。

## 2. 目标

1. **双模式**：`demo`（帧回放）与 `live`（通道驱动），可切换；
2. **Demo**：选择多帧图像（或 URL 列表）；播放/暂停/重置；可调间隔；进度条 + 当前帧/总帧；每帧刷新 P1 底图 canvas（几何沿用已加载 basemap meta 或 sidecar）；
3. **Live**：订阅 `Mapping`（`status`、`keyframes`）；可选将**启用中的 map OccupancyGrid** 同步到底图 canvas（或仅 HUD + 保留 live map 层）；状态与 P1 叠层不冲突；
4. **UI**：Map 上精简 HUD（模式、进度/keyframes、播放态）；**Mapping 面板**完整控制（模式切换、demo 加载、播放、live 说明）；
5. `npm test` / `npm run build` 通过；单测覆盖 demo 调度逻辑（帧索引、间隔、边界）。

## 3. 非目标（本 P2 不做）

- BICMap 建图机器人 GPS 轨迹动画、`robotController` 复刻（**P4** 可扩展）；
- 语义区 / 多楼层（**P3**）；
- 后端推送 PNG 帧流新 channel（demo 仅前端资源；live 用现有 Occupancy + Mapping）；
- 改 `Mapping` proto 字段（沿用 `status` + `keyframes`）；
- MapLibre、Turf、`@x-humanoid-cloud/bic-map`。

## 4. 方案选择

| 方案 | 结论 |
|------|------|
| ① `mappingStore` 驱动 P1 底图 canvas 更新 | **采用** |
| ② Demo 独立 DOM 层，live 走 map 层 | 拒绝（双通路） |
| ③ Demo 帧注入假 OccupancyGrid | 拒绝（抢 map 通道） |

用户锁定：

- **数据**：演示回放 + 实时通道（**C**）；
- **UI**：Map HUD + Mapping 面板（**C**）。

## 5. 数据契约

### 5.1 现有 Mapping（不变）

```protobuf
message Mapping {
  string status = 1;
  uint32 keyframes = 2;
}
```

Wire JSON 示例（mock）：`{"mode":"SLAM","keyframes":12,"status":"mapping"}` — 前端解析时以 `status`、`keyframes` 为主；多余字段忽略。

### 5.2 mappingStore（新增）

```ts
type MappingVizMode = 'demo' | 'live';

interface MappingDemoState {
  frameUrls: string[];       // blob: or http(s)
  frameIndex: number;          // 0-based
  playing: boolean;
  intervalMs: number;
  /** 与 P1 一致，demo 开始前需有或随 sidecar 设定 */
  basemapMeta: Pick<StaticSlamBasemap, 'originX' | 'originY' | 'resolution' | 'widthPx' | 'heightPx'> | null;
}

interface MappingLiveState {
  /** 为 true 时，map Occupancy 变化会刷新 basemap canvas（见 §6.2） */
  mirrorMapToBasemap: boolean;
}

interface MappingVizState {
  mode: MappingVizMode;
  demo: MappingDemoState;
  live: MappingLiveState;
  elapsedMs: number;         // demo 播放累计（可选 HUD）
  // actions: setMode, loadDemoFrames, play/pause/reset, setInterval, tick, ...
}
```

Persist（建议 `orbisview-mapping-viz-v1`）：仅 `mode`、`intervalMs`、`mirrorMapToBasemap`；**不**持久化 blob URL 列表。

### 5.3 Demo 帧来源

- 多选文件：按文件名排序（`stage_01.png` … 或 BICMap `map-data` 命名习惯）；
- 可选：粘贴/导入 URL 列表（JSON 数组）；
- 首帧加载时：若尚无 basemap，用 `staticSlamStore.formDraft` + `probeImageSize` 构造 `StaticSlamBasemap` 并 `setBasemap`；后续帧只替换 canvas 像素（`sharedStaticSlamCanvasCache.clear()` + 新 `imageSrc` 或原地 `putImageData`/drawImage）。

## 6. 架构与数据流

```
                    ┌─────────────────┐
                    │  MappingPanel   │  完整控制
                    └────────┬────────┘
                             │
                    ┌────────▼────────┐
                    │  mappingStore   │
                    └────────┬────────┘
              demo │          │ live
                   │          │
     frame timer   │          │ envelopes[Mapping] + map Occupancy
                   ▼          ▼
         updateBasemapCanvas()  ──► sharedStaticSlamCanvasCache
                   │          │      + staticSlamStore.setBasemap (meta 不变时仅 invalidate)
                   └────┬─────┘
                        ▼
              Map2D drawScene / View3D basemapPlane（P1 已有）
                        │
              MapMappingHud（精简只读 + 可选暂停）
```

### 6.1 Demo 模式

1. 用户加载帧序列 +（可选）yaml meta；
2. `play` → `setInterval` / RAF 调度：`frameIndex++` → 对当前帧 URL 重建 canvas → `cache.clear()` → Map/3D 重绘；
3. `pause` / `reset`：停止定时器；reset 回到帧 0 并刷新底图；
4. 播放结束：`playing=false`，HUD 显示「完成」。

参考 BICMap：`applyStage(stage)` 换图 + `preloadAhead` 可 P2 简化为按需 fetch（不预加载 8 帧亦可）。

### 6.2 Live 模式

1. **HUD**：从 `envelopes` 读 `Mapping` → 显示 `status`、`keyframes`；
2. **地图**：
   - 默认：P1 basemap（若有）+ 上层 **live map** Occupancy（Channels 驱动），与现行为一致；
   - 可选 **`mirrorMapToBasemap`**：当 `layers.map` 开启且 map grid 更新时，将 Occupancy 纹理**烘焙**到底图 canvas（同一 world 几何），便于「单底图 + 关闭 map 层仍见建图结果」；实现为 `occupancyTexture` → 临时 canvas → 替换 basemap 或 alpha 合成（P2 采用 **整图替换 basemap 像素、保留 origin/resolution** 的最简路径）。
3. 不强制关闭 live map 层；用户可同时开 basemap + map。

### 6.3 与 P1 工具栏关系

- 「静态底图」仍负责**单次**加载；Mapping 面板 demo 可**自动**建立 basemap（若未加载）；
- Demo 播放中禁止切换 basemap 文件（或先 stop demo）— UI 提示即可。

## 7. UI

### 7.1 Map 精简 HUD（`MapMappingHud.tsx`）

- 位置：Map 视口左下或状态条上方，不挡 `MapInstrumentCluster`；
- 内容：`demo` → `帧 12/50 · 播放中`；`live` → `SLAM · keyframes 42 · mapping`；
- Demo 播放中：可选单行「暂停」快捷按钮（完整控制在 Mapping 面板）。

### 7.2 Mapping 面板（替换纯 JSON `TaskPanel`）

- 模式切换：`演示回放` | `实时建图`；
- Demo：选帧、间隔 ms、播放/暂停/重置、进度条；
- Live：`mirrorMapToBasemap` 勾选 + Mapping JSON 摘要；
- 链接提示：「Map 上 basemap / map 图层可在 Channels 或底图工具栏切换」。

## 8. 文件清单

| 路径 | 动作 |
|------|------|
| `frontend/src/store/mappingVizStore.ts` | 状态 + demo 调度 |
| `frontend/src/store/mappingVizStore.test.ts` | 帧索引/边界/interval |
| `frontend/src/renderer/map2d/mappingDemo.ts` | 帧排序、applyFrame 辅助 |
| `frontend/src/components/Map/MapMappingHud.tsx` | Map HUD |
| `frontend/src/components/Tasks/MappingPanel.tsx`（或扩 `TaskPanels.tsx`） | 完整 UI |
| `frontend/src/components/Map2D/Map2DPanel.tsx` | 挂 HUD；live mirror hook |
| `frontend/src/components/View3D/View3DPanel.tsx` | 同上 |
| `frontend/src/components/Map/MapFloatToolbar.tsx` | 可选：demo 播放时禁用底图加载 |

## 9. 测试与验收

**自动：**

- demo：`nextFrame` 在末帧停止；reset 回 0；interval 不重叠 tick。

**手动：**

1. Demo：选 5+ 张 PNG/PGM + yaml，播放可见底图逐帧变化，暂停/重置正常；
2. Live：mock 运行，Mapping 面板 keyframes 递增，HUD 更新；
3. `mirrorMapToBasemap` 开：map 更新时底图跟随（关 map 层仍可见 basemap 烘焙结果）；
4. 2D↔3D 一致；测距/导航不受影响。

## 10. 风险

| 风险 | 缓解 |
|------|------|
| 每帧 clear cache + 解码卡顿 | 限制预加载；大图降采样沿用 P1 maxEdge |
| live mirror 与 live map 双重绘制 | 默认 mirror 关；开 mirror 时提示可关 map 层 |
| blob URL 泄漏 | demo stop/clear 时 revoke |

## 11. 后续

- **P3** 语义区 / 多楼层
- **P4** POI / 绘制 / 可选建图轨迹 overlay
