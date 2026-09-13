# OrbisView 地面机 Map2D P1 设计

**日期：** 2026-09-13

**状态：** 已确认（路径 B：mock + Autolink 双路径）

**范围：** 室内/地面移动机器人主视口能力：`renderer/` 抽层、可配置 footprint、local costmap 叠层、差速/阿克曼 HUD；mock 与 Autolink 共用同一 render schema。

**关联：** `docs/superpowers/specs/2026-09-13-orbisview-layered-migration-design.md`

## 1. 背景与动机

OrbisView 前端已有地面机布局 preset 与 Map2D（occupancy、激光、TF、固定矩形 footprint、teleop），但：

- 绘制逻辑堆在 `Map2DPanel.tsx`，`renderer/` 为空壳；
- footprint 写死常量，无法对接真机 footprint topic；
- 无 local costmap 叠层；
- HUD 仅显示 twist，无运动模型语义。

Apollo Dreamview 以车规 3D/HD Map 为主，不作为本 P1 的功能对标；本设计只借鉴「面板订阅 + 共享渲染层」的拆分方式。

## 2. 目标

P1 完成后应满足：

1. Map2D 绘制逻辑落在 `frontend/src/renderer/map2d/`，Panel 只负责订阅与交互；
2. 可渲染 **全局 map** 与 **local costmap**（同一 `OccupancyGrid` schema，靠 channel 区分）；
3. footprint 来自 `RobotFootprint` envelope；缺失时回退 `config/parameters.js` 默认矩形；
4. HUD 显示运动模型（`DIFF` / `ACKERMANN`）与 `vx`/`wz`（阿克曼可显示 steering）；
5. mock 发布 `/orbisview/mock/costmap`、`/orbisview/mock/footprint`；
6. Autolink：`OccupancyGrid` 继续服务 map/costmap；新增 `Polygon` / `PolygonStamped` → `RobotFootprint` converter + 单测。

## 3. 非目标（P1 不做）

- 新建专用 Costmap proto（0–255 lethal/inscribed 枚举）；
- 多楼层 / 多地图切换、充电桩 / POI；
- 定位协方差椭圆、建图质量条、地图保存加载 UX；
- 真机 bag 联调或改 Autolink 订阅配置 UI；
- View3D / 车模资源对齐 Dreamview。

## 4. 方案选择

| 方案 | 结论 |
|------|------|
| ① OccupancyGrid 复用 + 新 Footprint | **采用** |
| ② 新建 Costmap schema | 推迟到后续 |
| ③ 嵌入 WorldState | 拒绝（破坏 channel 订阅） |

数据源路径：**B** — mock 与 Autolink 双路径同步落地（converter 单测可无真机验收）。

## 5. 数据契约

### 5.1 Proto（`autonomy/orbisview/proto/render.proto`）

新增：

```protobuf
enum FootprintShape {
  FOOTPRINT_RECT = 0;      // 由 points 四角或 length/width 表达
  FOOTPRINT_POLYGON = 1;
  FOOTPRINT_CIRCLE = 2;
}

message RobotFootprint {
  FootprintShape shape = 1;
  // 机体系坐标，单位 m；POLYGON/RECT 使用；CIRCLE 可空
  repeated Pose2D points = 2;
  double radius = 3;       // CIRCLE
  double padding = 4;      // 可选膨胀
  double length = 5;       // RECT 便捷字段（无 points 时）
  double width = 6;
}

enum MotionModel {
  MOTION_DIFF = 0;
  MOTION_ACKERMANN = 1;
}
```

扩展 `ChassisState`（保持向后兼容，新字段默认 0 / 空）：

```protobuf
// 在既有 ChassisState 末尾追加：
MotionModel motion_model = 8;
```

不新增独立 `MotionModelHud` message：HUD 读 `ChassisState` + `Twist2D` 即可。

### 5.2 Schema ID

- `orbisview.render.RobotFootprint`
- 既有：`orbisview.render.OccupancyGrid`、`ChassisState`、`Twist2D`

同步：`backend/common/render_schemas.h`、`frontend/src/store/websocket/types.ts` `SCHEMAS`、`schemas.test.ts`。

### 5.3 Channel 约定

| 角色 | mock channel | schema | Autolink 输入 |
|------|--------------|--------|---------------|
| 全局图 | `/orbisview/mock/map` | OccupancyGrid | `map_msgs.OccupancyGrid`（已有） |
| 局部代价 | `/orbisview/mock/costmap` | OccupancyGrid | 同类型，**任意 channel 名**；FE 按 schema + 名称启发式或显式订阅列表识别 costmap |
| Footprint | `/orbisview/mock/footprint` | RobotFootprint | `geometry_msgs.Polygon` / `PolygonStamped` |
| 底盘/速度 | 既有 chassis / twist | ChassisState / Twist2D | 既有 |

**FE 识别 costmap：** 默认订阅列表含 `…/costmap`；Autolink 场景允许用户订阅任意 OccupancyGrid channel，并通过 layer 绑定「map 主层 / costmap 叠层」——P1 简化为：

- 第一个匹配 `OccupancyGrid` 且 channel 不含 `costmap` 的 → 全局 map；
- channel 名包含 `costmap`（大小写不敏感）→ costmap 叠层；
- 若仅有一个 OccupancyGrid，只画 map，costmap 层空。

### 5.4 JSON payload 形状（与现有 mock / converter 对齐）

OccupancyGrid（**以现有 JSON 为准**，非 proto field 名直出）：

```json
{
  "resolution": 0.05,
  "width": 100,
  "height": 100,
  "origin": { "x": -2.5, "y": -2.5, "yaw": 0 },
  "data": [0, -1, 100, ...]
}
```

绘制层只认上述 JSON；不强制改为 `origin_x`。

RobotFootprint 示例：

```json
{
  "shape": "POLYGON",
  "points": [
    {"x": 0.45, "y": 0.28},
    {"x": 0.45, "y": -0.28},
    {"x": -0.45, "y": -0.28},
    {"x": -0.45, "y": 0.28}
  ],
  "padding": 0.0
}
```

`shape` 在 JSON 中用字符串枚举名（去掉前缀）：`RECT` | `POLYGON` | `CIRCLE`，与现有 chassis `gear` 等字符串风格一致。Converter / mock 输出字符串；FE 容错数字枚举。

### 5.5 Polygon → Footprint 映射

- `Polygon` / `PolygonStamped.polygon.points`（Point32 x,y）→ `points`（z 忽略）；
- `shape` 固定为 `POLYGON`；
- `PolygonStamped` 时 `frame_id` 写入 envelope；点仍按机体系约定（与 Nav footprint 一致：base_link）；若 frame 非 base，P1 仍原样绘制并在 HUD 旁注 `frame=…`（不做 TF 变换）。

## 6. 模块边界

```text
MockSource / Autolink+automsgs_converter
        │ StreamEnvelope
        ▼
WebsocketHandler → FE dataStore.envelopes
        │
        ▼
Map2DPanel（订阅、layer、goal/measure/follow）
        │
        ▼
renderer/map2d（纯 canvas 绘制，无 React / 无 WS）
```

| 区域 | 路径 | 动作 |
|------|------|------|
| Proto | `proto/render.proto` | +`RobotFootprint`、`FootprintShape`、`MotionModel`；扩 `ChassisState` |
| Schema | `render_schemas.h` + FE `SCHEMAS` | + Footprint |
| Mock | `mock_source.*` | + costmap / footprint 通道与周期发布；chassis 带 `motion_model` |
| Converter | `automsgs_converter.*` + test | Polygon(Stamped)→Footprint |
| Renderer | `frontend/src/renderer/map2d/*` | coords、drawGrid、drawCostmap、drawLaser、drawFootprint、drawHud、types |
| Panel | `Map2DPanel.tsx` | 改用 renderer；订 footprint/costmap |
| Config | `frontend/config/parameters.js` | `DEFAULT_FOOTPRINT`、`DEFAULT_MOTION_MODEL` |
| Layers | `layoutStore` layer flags | +`costmap`，ground 默认开 |
| 默认订阅 | `Orbisview.tsx` | + mock costmap / footprint |

## 7. 前端渲染细节

### 7.1 绘制顺序（底→顶）

1. 全局 occupancy（灰阶）；
2. costmap 半透明热力（occupied 偏红/橙，unknown 跳过或极淡）；
3. vector map / keepouts（既有）；
4. path / planning / prediction / obstacles（既有）；
5. laser；
6. TF；
7. footprint 描边；
8. robot 位姿点 + 航向；
9. goal / measure；
10. HUD。

### 7.2 HUD

固定左上角半透明框：

- pose x/y/yaw；
- `model=DIFF|ACKERMANN` + `vx` / `wz`（ACKERMANN 时加 `steer` 若 chassis 有）；
- goal；
- 可选一行 `footprint=stream|default`。

### 7.3 默认 footprint（parameters）

```js
export const DEFAULT_MOTION_MODEL = 'DIFF';
export const DEFAULT_FOOTPRINT = {
  shape: 'RECT',
  length: 0.9,
  width: 0.56,
};
```

与当前硬编码半长 0.45 / 半宽 0.28 等价。

## 8. 错误处理与降级

| 情况 | 行为 |
|------|------|
| 无 costmap envelope | costmap 层不绘制，不报错 |
| 无 footprint envelope | 使用 `DEFAULT_FOOTPRINT` |
| OccupancyGrid `data` 长度与 w×h 不符 | 截断/填充 0，devtools 可 console.warn 一次 |
| Polygon 少于 3 点 | converter 返回 false；FE 回退默认 |
| Autolink 未开 | 仅 mock；行为与现网一致 |

## 9. 测试与验收

1. **单元：** `SuggestedRenderSchema` / `ConvertAutomsgsRaw` 对 Polygon、PolygonStamped；`schemas.test.ts` 含 Footprint ID；
2. **mock 手测：** `launch/dev.sh` → Map2D 可见 costmap 叠色 + 多边形 footprint + HUD `model=DIFF`；
3. **layer：** 关闭 `costmap` / `footprint` 后对应层消失；
4. **无 footprint 通道：** 临时从默认订阅去掉 footprint，仍画默认矩形。

## 10. 实现顺序建议

1. Proto + schema 字符串（BE/FE）；
2. Converter + 单测；
3. Mock 双通道；
4. `renderer/map2d` 抽取并接 Map2DPanel；
5. parameters / layers / 默认订阅；
6. 手测与文档一行更新（`orbisview/frontend/README.md`）。

## 11. 后续（P2，本文不实施）

定位协方差、建图质量、多地图/楼层、POI/充电桩、专用 Costmap schema、Autolink channel 绑定 UI。
