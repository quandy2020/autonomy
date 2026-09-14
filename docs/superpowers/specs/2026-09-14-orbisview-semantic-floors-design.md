# OrbisView 语义区与多楼层（P3）

**日期：** 2026-09-14

**状态：** 已确认（路径 **B + 数据 C + UI C + 绘制 B**；总路线 P1→P2→P3→P4 之 **P3**）

**范围：** 在 P1 底图与现有 Map2D/View3D 上增加 **语义区多边形** 与 **多楼层切换**（切换时更新 P1 basemap）；静态 fixture + live channel；Map 楼层条 + 面板完整控制；不引入 MapLibre / BICMap npm。

**关联：**

- `docs/superpowers/specs/2026-09-14-orbisview-static-slam-basemap-design.md`（P1 底图）
- `docs/superpowers/specs/2026-09-14-orbisview-mapping-process-viz-design.md`（P2；楼层切 basemap 与 demo 播放互斥提示）
- Proto：`automsgs/.../strata_msgs/semantic_zone.proto`、`floor_info.proto`
- Autoviz 参考：`strata_semantic_zone_display`、`strata_floor_panel`；BICMap 示例 channel `/strata/semantic_zones`、`/strata/floors`

## 1. 背景与动机

室内可视化需要在 SLAM 底图上叠加功能分区（禁行、可通行、房间等），并在多楼层建筑中切换当前层及其对应底图。OrbisView 已有 `vectormap` keepouts，但缺少 `zone_type` / 颜色 / label，且无楼层模型。Strata proto 与 Autoviz 已定义契约；P3 在 Canvas2D + Three 路径上移植只读展示与楼层切换，不复刻 BICMap 编辑器。

## 2. 目标

1. **语义区**：解析并绘制 `SemanticZone` 多边形（填充、描边、可选 label）；图层 `semantic` 可开关；
2. **多楼层**：维护 `FloorInfo[]` + `active_floor_id`；Map 条与面板可切换；切换时按该层 `slam` 几何加载/更新 P1 basemap（有 `slam_image_path` / 可解析 URL 时）；
3. **数据双源**：静态 JSON fixture（文件/粘贴）+ live channel（mock / Autolink 转换）；
4. **绘制**：Map2D + View3D（贴地）；不做跨楼层过滤 pose/path；
5. **UI**：Map 精简楼层切换条；**Semantic / Floors 面板**（或合并 `IndoorMapPanel`）含列表、图例、静态加载；
6. `npm test` / `npm run build` 通过；单测覆盖 zone 归一化、楼层切换与 basemap meta 映射。

## 3. 非目标（本 P3 不做）

- POI / 绘制工具 / 语义区在线编辑（**P4**）；
- 跨楼层隐藏 robot/path/obstacles；
- 修改 strata proto 字段；
- MapLibre、Turf、`@x-humanoid-cloud/bic-map`；
- 强制与 `vectormap` 合并为同一图层（二者并存；语义优先用 `semantic`）。

## 4. 方案选择

| 方案 | 结论 |
|------|------|
| ① 独立 `semantic` 层 + `indoorMapStore`（floors/zones）驱动 P1 basemap | **采用** |
| ② 语义塞进 `vectormap` keepouts | 拒绝（丢 zone_type/颜色契约） |
| ③ 仅 2D | 拒绝（用户锁定绘制 **B**） |
| ④ 切楼层过滤全部 overlay | 拒绝（留给后续） |

用户锁定：

- **数据**：静态 + live（**C**）；
- **UI**：Map 楼层条 + 面板（**C**）；
- **绘制**：2D + 3D（**B**）。

## 5. 数据契约

### 5.1 Strata 对齐（字段）

**SemanticZone**（wire JSON 可用 camel/snake，前端归一化）：

| 字段 | 含义 |
|------|------|
| `id` | 稳定 id |
| `zone_type` | 类型字符串（room / keepout / passable / …） |
| `polygon` | `{x,y}` 或 `[x,y]` 点列（世界坐标） |
| `fill_color` / `fill_opacity` | RGBA 或 0–1 opacity |
| `outline_color` / `outline_width` | 描边 |
| `label` | 可选文字 |

**FloorInfo：**

| 字段 | 含义 |
|------|------|
| `id` / `name` / `level` | 标识与排序 |
| `slam_image_path` | 底图 URL 或相对路径（静态/可 fetch） |
| `start_x` / `start_y` | origin（同 BICMap / P1） |
| `x_grid_count` / `y_grid_count` | 像素宽高 |
| `resolution` | m/px |

**FloorInfoArray：** `floors[]` + `active_floor_id`。

### 5.2 OrbisView schema

新增（与现有 `orbisview.render.*` 一致）：

```text
orbisview.render.SemanticZoneArray
orbisview.render.FloorInfoArray
```

- Frontend：`SCHEMAS.SemanticZoneArray` / `SCHEMAS.FloorInfoArray`；
- Channels `displayTypes`：匹配上述 schema + `strata_msgs/SemanticZoneArray` / `FloorInfoArray` 别名；
- Backend（P3 范围）：
  - mock：`/orbisview/mock/semantic_zones`、`/orbisview/mock/floors`；
  - Automsgs converter：`strata_msgs.SemanticZoneArray` / `FloorInfoArray` → 上述 schema JSON（若本迭代时间紧，可先 mock + 静态，converter 列为 plan 必做或 follow-up；**spec 要求至少 mock + 前端解析**，converter **应做**以对齐 live）。

### 5.3 静态 fixture JSON

```json
{
  "floors": [ { "id": "F1", "name": "1F", "level": 1, "slam_image_path": "...", "start_x": -10, "start_y": -10, "x_grid_count": 384, "y_grid_count": 384, "resolution": 0.05 } ],
  "active_floor_id": "F1",
  "zones": [ { "id": "z1", "zone_type": "keepout", "polygon": [[0,0],[2,0],[2,1],[0,1]], "fill_color": { "r": 1, "g": 0.2, "b": 0.2, "a": 1 }, "fill_opacity": 0.35, "label": "禁行" } ]
}
```

也可分文件加载 zones / floors。

### 5.4 indoorMapStore

```ts
interface IndoorMapState {
  source: 'none' | 'static' | 'live';
  floors: FloorInfo[];
  activeFloorId: string | null;
  zones: SemanticZone[];
  // live 覆盖：有 envelope 时 source='live'；静态加载 source='static'
  setFromFixture: (f: IndoorFixture) => void;
  setActiveFloor: (id: string) => void; // 触发 basemap 更新
  clearStatic: () => void;
}
```

Persist（`orbisview-indoor-map-v1`）：仅 `activeFloorId`（若仍存在于 floors）；**不**持久化大图 path blob。

切楼层副作用：

1. 更新 `activeFloorId`；
2. 若该层有可用 `slam_image_path`（http(s)/绝对 public/blob）→ `staticSlamStore.setBasemap`（meta 来自 floor 字段）；
3. 若无图像：仅切换高亮，status 提示「无底图」；
4. 若 P2 demo `playing`：拒绝切层或先 pause 并提示（与 P2 工具栏互斥同型）。

## 6. 架构与数据流

```
 Channels / mock / fixture
        │
        ▼
 indoorMapStore (floors, zones, active)
        │
        ├─ setActiveFloor ──► staticSlamStore.setBasemap (P1)
        │
        ▼
 Map2D drawScene (layer semantic) + View3D semantic meshes
        │
 MapFloorBar  +  IndoorMapPanel
```

### 6.1 叠层顺序（2D）

grid → basemap → map → costmap → **semantic** → vectormap → path → robot / …  

（semantic 在 occupancy 之上、矢量/路径之下，便于看见分区又不挡导航线。）

### 6.2 View3D

与 2D 同世界坐标；`Shape`/`ShapeGeometry` 或 `Line`+半透明 `Mesh` 贴地（yLift 略高于 basemap、低于 robot）；颜色取 zone 字段，缺省按 `zone_type` 调色板。

### 6.3 与 vectormap

并存；不自动把 keepouts 导入 semantic。文档注明差异。

## 7. UI

### 7.1 MapFloorBar

- 位置：Map 视口顶部居中或左上（不挡 `MapInstrumentCluster` / Mapping HUD）；
- 控件：‹ · **当前楼层名** · ›；无 floors 时隐藏；
- 点击也可打开面板（可选）。

### 7.2 IndoorMapPanel（或 SemanticPanel）

- 楼层列表（高亮 active；点击 = `setActiveFloor`）；
- 语义图例：按 `zone_type` 聚合色块 + 数量；
- 静态：选择 JSON / 粘贴；清除静态（live 仍可覆盖）；
- 提示：basemap / semantic 图层开关位置。

### 7.3 Layers

- `LayerKey` 增加 `semantic: true` 默认；
- Persist bump **`orbisview-layers-v7`**，migrate 缺省 `semantic: true`。

## 8. 文件清单（预期）

| 路径 | 动作 |
|------|------|
| `frontend/src/store/indoorMapStore.ts` | floors/zones/active |
| `frontend/src/renderer/map2d/semanticZones.ts` | 归一化、调色板、corners |
| `frontend/src/renderer/map2d/drawSemantic.ts` | 2D 绘制 |
| `frontend/src/renderer/view3d/layers/semantic.ts` | 3D |
| `frontend/src/components/Map/MapFloorBar.tsx` | 楼层条 |
| `frontend/src/components/Tasks/IndoorMapPanel.tsx` | 面板 |
| `frontend/src/store/layoutStore.ts` | `semantic` + v7 |
| `frontend/src/store/websocket/types.ts` + `displayTypes` + binding | schema / role |
| `backend` mock +（应做）automsgs converter | live |
| `schemas.test.ts` | schema 常量 |

## 9. 测试与验收

**自动：**

- polygon 点列归一化；缺色时 fallback palette；
- `setActiveFloor` 在有 slam 字段时产生正确 `StaticSlamBasemap` meta；
- layer persist migrate。

**手动：**

1. 加载 fixture → 语义可见；切楼层 → basemap 变（有图时）；
2. mock floors/zones → live 更新；关 `semantic` 层多边形消失；
3. 2D↔3D 分区对齐；测距/导航不受影响；
4. P2 demo 播放中切楼层有提示或被阻止。

## 10. 风险

| 风险 | 缓解 |
|------|------|
| `slam_image_path` 不可 fetch | 仅切列表；status 提示 |
| zone 过多卡顿 | 上限绘制 N（如 200）+ 简化 label |
| live 与 static 争用 | live envelope 到达覆盖 static zones/floors；clearStatic 仅清静态 |
| converter 工作量大 | mock 先通；converter 同 PR 或紧随 |

## 11. 后续

- **P4** POI / 绘制 / 语义编辑
- 跨楼层 overlay 过滤、电梯连通可视化
