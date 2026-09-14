# OrbisView POI 与绘制标注（P4）

**日期：** 2026-09-14

**状态：** 已确认（范围 **C + 交互 B + 数据/UI C**；总路线 P1→P2→P3→P4 之 **P4**）

**范围：** 在 Map2D/View3D 上增加 **POI 落点** 与 **多边形/折线绘制**；本地 persist + JSON 导入导出；工具栏新工具 + Annotations 面板；不引入 MapLibre / BICMap npm；不接 live 写回。

**关联：**

- `docs/superpowers/specs/2026-09-14-orbisview-semantic-floors-design.md`（P3；语义只读，本阶段不在线编辑）
- 现有：`MapTool`（pan/measure/nav/pick）、`waypointStore`、测距落点交互
- BICMap 参考：室内 POI / PassableArea 编辑（逻辑移植，不引 npm）

## 1. 背景与动机

室内运维需要在 SLAM 底图上标记充电桩、电梯等兴趣点，并手绘禁行/通道示意。OrbisView 已有取点与导航航点，但缺少可持久化的标注层与专用绘制工具。P4 在本地完成编辑闭环，为后续服务端同步留接口形状，而不在本期做写回。

## 2. 目标

1. **POI 工具**：单击落点；类型（`charger` / `elevator` / `custom`）+ 可选 label / yaw；可选拖移与删除；
2. **绘制工具**：左键加点，双击或右击结束；`polygon` | `polyline`；撤销上一点；
3. **图层**：`poi`、`draw` 可独立开关；2D + 3D 一致显示；
4. **数据**：`annotationStore` 本地 persist；JSON 导入/导出；
5. **UI**：工具栏入口 + **Annotations 面板**（列表、筛选、清除、导入导出）；
6. `npm test` / `npm run build` 通过；单测覆盖 JSON 归一化与 store 增删边界。

## 3. 非目标（本 P4 不做）

- 语义区（`SemanticZone`）在线编辑（仍 P3 只读；后续可把 draw 导出为 zone）；
- 建图机器人 GPS 轨迹动画；
- 服务端持久化 / Autolink 写回 / live POI channel 订阅；
- MapLibre、Turf、`@x-humanoid-cloud/bic-map`；
- 跨楼层过滤标注（可随 `activeFloorId` 扩展字段，本期全部全局显示）。

## 4. 方案选择

| 方案 | 结论 |
|------|------|
| ① 独立 `annotationStore` + `MapTool` `poi`/`draw` | **采用** |
| ② 复用 `waypointStore` 塞 POI | 拒绝（语义与导航航点混淆） |
| ③ 仅 2D 交互 | 拒绝（用户锁定交互 **B**） |
| ④ 本期 live 写回 | 拒绝（用户锁定数据 **C**） |

用户锁定：范围 **C**、交互 **B**、数据/UI **C**。

## 5. 数据契约

### 5.1 POI

```ts
type PoiKind = 'charger' | 'elevator' | 'custom';

interface MapPoi {
  id: string;          // uuid
  x: number;
  y: number;
  yaw?: number;
  kind: PoiKind;
  label?: string;
  color?: string;      // optional override
}
```

### 5.2 形状

```ts
type DrawShapeKind = 'polygon' | 'polyline';

interface MapDrawShape {
  id: string;
  kind: DrawShapeKind;
  points: [number, number][];  // polygon ≥3, polyline ≥2
  label?: string;
  stroke?: string;
  fill?: string;               // polygon only
}
```

### 5.3 Fixture / 导出 JSON

```json
{
  "version": 1,
  "pois": [ { "id": "...", "x": 1, "y": 2, "kind": "charger", "label": "充电1" } ],
  "shapes": [ { "id": "...", "kind": "polygon", "points": [[0,0],[1,0],[1,1]], "label": "禁行示意" } ]
}
```

### 5.4 annotationStore

```ts
interface AnnotationState {
  pois: MapPoi[];
  shapes: MapDrawShape[];
  draft: null | { tool: 'draw'; kind: DrawShapeKind; points: [number, number][] };
  poiDefaultKind: PoiKind;
  drawDefaultKind: DrawShapeKind;
  addPoi / updatePoi / removePoi
  addShape / updateShape / removeShape
  setDraft / appendDraftPoint / undoDraftPoint / commitDraft / cancelDraft
  importJson / exportJson / clearAll
}
```

Persist（`orbisview-annotations-v1`）：`pois`、`shapes`、`poiDefaultKind`、`drawDefaultKind`；**不**持久化 `draft`。

## 6. 架构与数据流

```
 MapTool poi|draw  (Map2D / View3D)
        │
        ▼
 annotationStore  ◄── AnnotationsPanel (list / import / export)
        │
        ▼
 drawAnnotations (2D) + view3d/layers/annotations (3D)
```

### 6.1 工具行为

**poi**

1. 左键：在世界坐标添加 POI（`poiDefaultKind`）；
2. 已有 POI 附近拖移（命中半径随 scale，约 0.35 m 或 12 px）；
3. Delete/Backspace 或面板删除选中项；
4. 可选：第二次点击空白取消选中。

**draw**

1. 左键追加 draft 点；橡皮筋预览到光标；
2. 双击或右击：若点数足够则 `commitDraft`，否则 `cancelDraft`；
3. Esc：取消 draft；
4. 工具栏或面板切换 polygon/polyline（`drawDefaultKind`）。

与 measure/nav 互斥：同一时间仅一个 `MapTool`。

### 6.2 叠层（2D）

… → semantic → vectormap → **draw** → **poi** → path → robot → …

（标注靠近矢量层，不挡路径与机器人。）

### 6.3 View3D

- POI：贴地小锥/圆柱 + 可选 label sprite（或简化为彩色点 + 状态栏名称）；
- polygon：半透明 Shape；polyline：Line；
- 交互：与现有 pick/nav 相同的地面射线拾取。

## 7. UI

### 7.1 工具栏

- 新增 `poi`、`draw` 按钮（图标可复用 `pick`/`waypoint` 变体或新增 `poi`/`draw` Icon）；
- draw 激活时：短提示「左键加点 · 双击/右击结束」。

### 7.2 Annotations 面板

- Tab 或分段：POI | 形状；
- 列表：选中、改 label/kind、删除；
- 默认类型选择；
- 导入 JSON / 导出下载 / 全部清除（确认）；
- hint：数据仅保存在本机浏览器。

### 7.3 Layers

- `LayerKey`：`poi`、`draw`，默认 `true`；
- Persist bump **`orbisview-layers-v8`**，migrate 缺省 true。

## 8. 文件清单（预期）

| 路径 | 动作 |
|------|------|
| `frontend/src/store/annotationStore.ts` | 状态 + persist |
| `frontend/src/store/annotationStore.test.ts` | 增删、commit draft、import |
| `frontend/src/renderer/map2d/annotations.ts` | 归一化 JSON |
| `frontend/src/renderer/map2d/drawAnnotations.ts` | 2D 绘制 |
| `frontend/src/renderer/view3d/layers/annotations.ts` | 3D |
| `frontend/src/store/mapViewStore.ts` | MapTool 扩展 |
| `frontend/src/store/layoutStore.ts` | layers v8 |
| `frontend/src/components/Map/MapFloatToolbar.tsx` | 工具按钮 |
| `frontend/src/components/Tasks/AnnotationsPanel.tsx` | 面板 |
| `frontend/src/components/Map2D/Map2DPanel.tsx` | 交互 + 绘制 |
| `frontend/src/components/View3D/View3DPanel.tsx` | 同上 |
| `frontend/src/styles/main.css` | 样式 |

## 9. 测试与验收

**自动：**

- import 非法 JSON 失败不破坏原数据；
- commit polygon 点数 &lt;3 拒绝；polyline &lt;2 拒绝；
- clearAll 清空 pois/shapes。

**手动：**

1. poi 落点 → 刷新页面仍在；导出再导入一致；
2. draw 多边形/折线在 2D/3D 对齐；撤销一点与 Esc 正常；
3. 关 `poi`/`draw` 层隐藏；测距/导航工具不受影响；
4. 与 P2 demo 播放、P3 切楼层可并存（标注不随楼层过滤）。

## 10. 风险

| 风险 | 缓解 |
|------|------|
| 工具过多挤工具栏 | 分组分隔；图标+title |
| 拖移与 pan 冲突 | poi 工具下禁用 pan 拖图，或按住空格临时 pan（可选；P4 默认工具内不 pan） |
| localStorage 体积 | 点数上限（如每 shape 200 点；pois 500） |

## 11. 后续

- 标注绑定 `floorId`；导出为 `SemanticZone`；
- live channel 订阅/写回；
- 充电桩对接导航特殊行为
