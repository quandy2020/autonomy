# OrbisView Ground Robot Map2D P1 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Ship indoor ground-robot Map2D P1: shared `renderer/map2d`, streamable footprint, OccupancyGrid costmap overlay, DIFF/ACKERMANN HUD, with mock + Autolink dual path on the same schemas.

**Architecture:** Keep `OccupancyGrid` for both global map and local costmap (channel name distinguishes). Add `RobotFootprint` (+ `MotionModel` on `ChassisState`). Mock publishes new channels; `automsgs_converter` maps `Polygon`/`PolygonStamped` → Footprint. FE extracts canvas draws into `renderer/map2d`; `Map2DPanel` only subscribes and handles tools.

**Tech Stack:** C++17 (orbisview backend), automsgs protobuf, Vite/React/TS/zustand frontend, gtest + frontend `schemas.test.ts`.

**Spec:** `docs/superpowers/specs/2026-09-13-orbisview-ground-robot-map2d-p1-design.md`

## Global Constraints

- Path **B**: mock and Autolink dual path in the same P1.
- Do **not** add a dedicated Costmap proto; reuse OccupancyGrid.
- OccupancyGrid **JSON** shape stays `{resolution,width,height,origin:{x,y,yaw},data:[…]}` (mock/converter today), not proto field names `origin_x`.
- Footprint JSON `shape` strings: `RECT` | `POLYGON` | `CIRCLE`.
- P1 non-goals: multi-floor, localization cov, map save/load UX, POI, Dreamview 3D parity.
- Prefer small focused files under `frontend/src/renderer/map2d/`.
- Follow existing mock/converter patterns (EmitJson, SetJsonPayload, TypeIs).

## File map

| File | Responsibility |
|------|----------------|
| `autonomy/orbisview/proto/render.proto` | `RobotFootprint`, enums, `ChassisState.motion_model` |
| `backend/common/render_schemas.h` | `kSchemaFootprint` |
| `backend/adapters/automsgs/automsgs_converter.{h,cc}` | Polygon → Footprint |
| `backend/adapters/automsgs/automsgs_converter_test.cc` | Converter tests |
| `backend/adapters/mock/mock_source.{h,cc}` | `/costmap`, `/footprint`; chassis `motion_model` |
| `frontend/src/store/websocket/types.ts` | `SCHEMAS.RobotFootprint` |
| `frontend/src/schemas.test.ts` | Schema string assert |
| `frontend/config/parameters.js` | `DEFAULT_FOOTPRINT`, `DEFAULT_MOTION_MODEL` |
| `frontend/src/renderer/map2d/*` | Pure draw helpers |
| `frontend/src/renderer/index.ts` | Re-exports |
| `frontend/src/components/Map2D/Map2DPanel.tsx` | Wire renderer + new layers |
| `frontend/src/store/layoutStore.ts` | `costmap` layer flag |
| `frontend/src/components/Orbisview.tsx` | Default subscribe list |
| `frontend/README.md` | One-line P1 note |

---

### Task 1: Proto + schema ID strings

**Files:**
- Modify: `autonomy/orbisview/proto/render.proto`
- Modify: `autonomy/orbisview/backend/common/render_schemas.h`
- Modify: `autonomy/orbisview/frontend/src/store/websocket/types.ts`
- Modify: `autonomy/orbisview/frontend/src/schemas.test.ts`

**Interfaces:**
- Produces: schema id `orbisview.render.RobotFootprint`; C++ `rendering::kSchemaFootprint`; FE `SCHEMAS.RobotFootprint`

- [ ] **Step 1: Extend `render.proto`**

Append before `HmiStatus` (or after `ChassisState` block) exactly:

```protobuf
enum FootprintShape {
  FOOTPRINT_RECT = 0;
  FOOTPRINT_POLYGON = 1;
  FOOTPRINT_CIRCLE = 2;
}

message RobotFootprint {
  FootprintShape shape = 1;
  repeated Pose2D points = 2;
  double radius = 3;
  double padding = 4;
  double length = 5;
  double width = 6;
}

enum MotionModel {
  MOTION_DIFF = 0;
  MOTION_ACKERMANN = 1;
}
```

In `ChassisState`, append:

```protobuf
  MotionModel motion_model = 8;
```

(Keep existing fields 1–7 unchanged.)

- [ ] **Step 2: Add C++ schema constant**

In `render_schemas.h` after `kSchemaOccupancyGrid`:

```cpp
inline constexpr char kSchemaFootprint[] = "orbisview.render.RobotFootprint";
```

- [ ] **Step 3: Add FE SCHEMAS + failing test first**

In `types.ts` `SCHEMAS` object add:

```ts
  RobotFootprint: 'orbisview.render.RobotFootprint',
```

In `schemas.test.ts` add:

```ts
expect(SCHEMAS.RobotFootprint).toBe('orbisview.render.RobotFootprint');
```

- [ ] **Step 4: Run FE schema test**

Run: `cd autonomy/orbisview/frontend && npm test -- --run src/schemas.test.ts`

Expected: PASS

- [ ] **Step 5: Commit**

```bash
git add autonomy/orbisview/proto/render.proto \
  autonomy/orbisview/backend/common/render_schemas.h \
  autonomy/orbisview/frontend/src/store/websocket/types.ts \
  autonomy/orbisview/frontend/src/schemas.test.ts
git commit -m "feat(orbisview): add RobotFootprint render schema"
```

---

### Task 2: Automsgs Polygon → Footprint (TDD)

**Files:**
- Modify: `autonomy/orbisview/backend/adapters/automsgs/automsgs_converter.cc`
- Modify: `autonomy/orbisview/backend/adapters/automsgs/automsgs_converter_test.cc`
- Test: same test file

**Interfaces:**
- Consumes: `kSchemaFootprint`; `automsgs.msgs.geometry_msgs.Polygon` / `PolygonStamped`
- Produces: `ConvertAutomsgsRaw` success with JSON `{"shape":"POLYGON","points":[...],"padding":0}`

- [ ] **Step 1: Write failing tests**

Add includes:

```cpp
#include <automsgs/msgs/geometry_msgs/polygon.pb.h>
#include <automsgs/msgs/geometry_msgs/polygon_stamped.pb.h>
```

Add tests:

```cpp
TEST(AutomsgsConverterTest, SuggestedSchemaPolygon) {
  EXPECT_EQ(SuggestedRenderSchema("automsgs.msgs.geometry_msgs.Polygon"),
            autonomy::orbisview::rendering::kSchemaFootprint);
  EXPECT_EQ(SuggestedRenderSchema("automsgs.msgs.geometry_msgs.PolygonStamped"),
            autonomy::orbisview::rendering::kSchemaFootprint);
}

TEST(AutomsgsConverterTest, PolygonToFootprintJson) {
  automsgs::msgs::geometry_msgs::Polygon poly;
  auto* p0 = poly.add_points();
  p0->set_x(0.45f);
  p0->set_y(0.28f);
  auto* p1 = poly.add_points();
  p1->set_x(0.45f);
  p1->set_y(-0.28f);
  auto* p2 = poly.add_points();
  p2->set_x(-0.45f);
  p2->set_y(-0.28f);
  std::string bytes;
  ASSERT_TRUE(poly.SerializeToString(&bytes));
  StreamEnvelope env;
  ASSERT_TRUE(ConvertAutomsgsRaw("/footprint",
      "automsgs.msgs.geometry_msgs.Polygon", bytes, 7, &env));
  EXPECT_EQ(env.schema, autonomy::orbisview::rendering::kSchemaFootprint);
  const std::string json(env.payload.begin(), env.payload.end());
  EXPECT_NE(json.find("\"shape\":\"POLYGON\""), std::string::npos);
  EXPECT_NE(json.find("\"x\":0.45"), std::string::npos);
}

TEST(AutomsgsConverterTest, PolygonTooFewPointsFails) {
  automsgs::msgs::geometry_msgs::Polygon poly;
  poly.add_points()->set_x(0.f);
  std::string bytes;
  ASSERT_TRUE(poly.SerializeToString(&bytes));
  StreamEnvelope env;
  EXPECT_FALSE(ConvertAutomsgsRaw("/footprint",
      "automsgs.msgs.geometry_msgs.Polygon", bytes, 1, &env));
}
```

Also add a `PolygonStamped` happy-path asserting `env.frame_id == "base_link"`.

- [ ] **Step 2: Run tests — expect FAIL**

Run (adjust build dir if needed):

```bash
cmake --build build-orbisview --target orbisview_automsgs_converter_test -j$(nproc) 2>/dev/null \
  || cmake --build build-orbisview --target orbisview_tests -j8
# then run the gtest binary that contains AutomsgsConverterTest
ctest --test-dir build-orbisview -R AutomsgsConverter -V
```

If target names differ, locate with `rg AutomsgsConverterTest autonomy/orbisview/CMakeLists.txt` and use that binary. Expected: link/compile OK but SuggestedSchemaPolygon / Convert FAIL (empty schema / false).

- [ ] **Step 3: Implement converter**

In anonymous namespace of `automsgs_converter.cc`, add includes for polygon protos. Implement:

```cpp
bool ConvertPolygonPoints(const google::protobuf::RepeatedPtrField<
                              geometry_msgs::Point32>& points,
                          core::StreamEnvelope* out) {
  if (points.size() < 3) return false;
  std::ostringstream oss;
  oss << "{\"shape\":\"POLYGON\",\"padding\":0,\"points\":[";
  for (int i = 0; i < points.size(); ++i) {
    if (i) oss << ',';
    oss << "{\"x\":" << points.Get(i).x() << ",\"y\":" << points.Get(i).y()
        << ",\"yaw\":0}";
  }
  oss << "]}";
  out->schema = rendering::kSchemaFootprint;
  SetJsonPayload(out, oss.str());
  return true;
}

bool ConvertPolygon(const std::string& bytes, core::StreamEnvelope* out) {
  geometry_msgs::Polygon msg;
  if (!msg.ParseFromString(bytes)) return false;
  return ConvertPolygonPoints(msg.points(), out);
}

bool ConvertPolygonStamped(const std::string& bytes, core::StreamEnvelope* out) {
  geometry_msgs::PolygonStamped msg;
  if (!msg.ParseFromString(bytes)) return false;
  out->frame_id = msg.header().frame_id();
  return ConvertPolygonPoints(msg.polygon().points(), out);
}
```

Wire in `SuggestedRenderSchema` and `ConvertAutomsgsRaw` for both type strings.

- [ ] **Step 4: Run tests — expect PASS**

Same `ctest` / binary as Step 2. Expected: all AutomsgsConverterTest PASS.

- [ ] **Step 5: Commit**

```bash
git add autonomy/orbisview/backend/adapters/automsgs/
git commit -m "feat(orbisview): convert Polygon to RobotFootprint"
```

---

### Task 3: Mock costmap + footprint channels

**Files:**
- Modify: `autonomy/orbisview/backend/adapters/mock/mock_source.h`
- Modify: `autonomy/orbisview/backend/adapters/mock/mock_source.cc`

**Interfaces:**
- Produces: channels `/orbisview/mock/costmap` (`kSchemaOccupancyGrid`), `/orbisview/mock/footprint` (`kSchemaFootprint`); chassis JSON includes `"motion_model":"DIFF"`

- [ ] **Step 1: Add channel constants + seq + Channels() entries**

```cpp
constexpr char kCostmapChannel[] = "/orbisview/mock/costmap";
constexpr char kFootprintChannel[] = "/orbisview/mock/footprint";
```

Header: `uint64_t seq_costmap_{0}; uint64_t seq_footprint_{0};`

In `Channels()` append two `ChannelInfo` rows mirroring map (OccupancyGrid) and footprint (`kSchemaFootprint` for schema + msg_type).

- [ ] **Step 2: BuildCostmapJson + BuildFootprintJson helpers**

`BuildCostmapJson`: smaller grid (e.g. 30×30, resolution 0.1, origin near robot or fixed `-1.5,-1.5`) with a lethal blob ahead of +x in map frame (static is OK for P1).

`BuildFootprintJson`:

```cpp
return R"({"shape":"POLYGON","padding":0,"points":[
  {"x":0.45,"y":0.28,"yaw":0},{"x":0.45,"y":-0.28,"yaw":0},
  {"x":-0.45,"y":-0.28,"yaw":0},{"x":-0.45,"y":0.28,"yaw":0}]})";
```

- [ ] **Step 3: Emit in Loop**

Near existing map emit (same slow cadence is fine):

```cpp
EmitJson(kCostmapChannel, rendering::kSchemaOccupancyGrid, "map",
         &seq_costmap_, BuildCostmapJson());
EmitJson(kFootprintChannel, rendering::kSchemaFootprint, "base_link",
         &seq_footprint_, BuildFootprintJson());
```

Extend chassis payload with `,"motion_model":"DIFF"` before closing `}`.

- [ ] **Step 4: Smoke rebuild backend**

```bash
cmake -S autonomy/orbisview -B build-orbisview && cmake --build build-orbisview -j8
```

Expected: build succeeds. Optional: run binary, WS list_channels shows new names (manual).

- [ ] **Step 5: Commit**

```bash
git add autonomy/orbisview/backend/adapters/mock/
git commit -m "feat(orbisview): mock costmap and footprint streams"
```

---

### Task 4: `renderer/map2d` extract + wire Map2DPanel

**Files:**
- Create: `frontend/src/renderer/map2d/types.ts`
- Create: `frontend/src/renderer/map2d/coords.ts`
- Create: `frontend/src/renderer/map2d/drawOccupancy.ts`
- Create: `frontend/src/renderer/map2d/drawFootprint.ts`
- Create: `frontend/src/renderer/map2d/drawHud.ts`
- Create: `frontend/src/renderer/map2d/drawScene.ts`
- Modify: `frontend/src/renderer/index.ts`
- Modify: `frontend/src/components/Map2D/Map2DPanel.tsx`
- Modify: `frontend/config/parameters.js`
- Modify: `frontend/src/store/layoutStore.ts` (`LayerKey` + `costmap: true`, bump persist name to `orbisview-layers-v5`)
- Modify: `frontend/src/components/Orbisview.tsx` default channels
- Modify: `frontend/README.md` (one bullet: Map2D P1 costmap/footprint)

**Interfaces:**
- Consumes: envelopes for map/costmap/footprint/chassis/twist; `DEFAULT_FOOTPRINT`
- Produces:

```ts
// coords.ts
export type WorldToScreen = (x: number, y: number) => readonly [number, number];
export function makeWorldToScreen(
  canvasW: number, canvasH: number, offsetX: number, offsetY: number, scale: number
): WorldToScreen;

// types.ts — Pose2D, OccupancyGridJson, RobotFootprintJson, ChassisJson, LayerFlags

// drawOccupancy.ts
export function drawOccupancyGrid(
  ctx: CanvasRenderingContext2D,
  grid: OccupancyGridJson,
  toScreen: WorldToScreen,
  scale: number,
  mode: 'map' | 'costmap',
): void;

// drawFootprint.ts
export function resolveFootprintPoints(
  fp: RobotFootprintJson | null,
  fallback: { shape: string; length: number; width: number },
): { x: number; y: number }[];
export function drawFootprint(
  ctx: CanvasRenderingContext2D,
  pose: Pose2D,
  pointsBody: { x: number; y: number }[],
  toScreen: WorldToScreen,
): void;

// drawHud.ts
export function drawMapHud(
  ctx: CanvasRenderingContext2D,
  args: {
    pose: Pose2D | null;
    twist: { vx: number; wz: number } | null;
    chassis: { motion_model?: string; steering?: number } | null;
    goal: Pose2D | null;
    footprintSource: 'stream' | 'default';
  },
): void;

// drawScene.ts
export function paintMap2DScene(ctx: CanvasRenderingContext2D, input: Map2DSceneInput): void;
```

Costmap mode: occupied cells fill `rgba(255,112,67,0.45)`; skip free (0); unknown optional skip.

Map/costmap selection in Panel:

```ts
function pickMapAndCostmap(envelopes: Record<string, StreamEnvelope>) {
  const grids = Object.values(envelopes).filter((e) => e.schema === SCHEMAS.OccupancyGrid);
  const costmapEnv = grids.find((e) => /costmap/i.test(e.channel));
  const mapEnv =
    grids.find((e) => e !== costmapEnv && !/costmap/i.test(e.channel)) ??
    grids.find((e) => e !== costmapEnv);
  return { map: asPayload(mapEnv), costmap: asPayload(costmapEnv) };
}
```

`parameters.js`:

```js
export const DEFAULT_MOTION_MODEL = 'DIFF';
export const DEFAULT_FOOTPRINT = { shape: 'RECT', length: 0.9, width: 0.56 };
```

Default subscribe in `Orbisview.tsx`:

```ts
'/orbisview/mock/costmap',
'/orbisview/mock/footprint',
```

- [ ] **Step 1: Add parameters + layer key + default channels** (no draw yet)

- [ ] **Step 2: Implement `renderer/map2d/*` and export from `renderer/index.ts`**

Move existing Map2D draw loops into `paintMap2DScene` / helpers; keep interaction (goal/measure) in Panel.

- [ ] **Step 3: Refactor `Map2DPanel` to call `paintMap2DScene`**

Subscribe footprint (`SCHEMAS.RobotFootprint`) and chassis; pass `layers.costmap`; HUD uses `motion_model` / footprint source.

- [ ] **Step 4: Verify FE**

```bash
cd autonomy/orbisview/frontend && npm test && npm run build
```

Expected: tests PASS, build OK.

- [ ] **Step 5: Manual smoke (optional if stack running)**

`npm run dev:all` or `launch/dev.sh` → Connect → Map2D shows orange costmap overlay + teal footprint polygon + HUD `model=DIFF` and `footprint=stream`. Toggle costmap/footprint layers off → layers hide.

- [ ] **Step 6: Commit**

```bash
git add autonomy/orbisview/frontend/
git commit -m "feat(orbisview): Map2D renderer with costmap and footprint"
```

---

### Task 5: End-to-end checklist (no new features)

**Files:** none required unless README missed

- [ ] **Step 1: Confirm checklist against spec §9**

- [x] Schema strings aligned BE/FE  
- [x] Polygon converter tests  
- [x] Mock channels listed  
- [x] Map2D costmap + footprint + HUD  
- [x] Default footprint fallback (briefly unsubscribe footprint and confirm RECT still draws)

- [ ] **Step 2: Final commit only if README/docs tweaks remain**

```bash
git add autonomy/orbisview/frontend/README.md
git commit -m "docs(orbisview): note Map2D P1 costmap footprint"
```

(Skip empty commit if already included in Task 4.)

---

## Spec coverage self-check

| Spec section | Task |
|--------------|------|
| §5 Proto / schema / JSON | Task 1 |
| §5.5 Polygon mapping | Task 2 |
| §5.3 mock channels + chassis model | Task 3 |
| §6–7 renderer + panel + layers + parameters | Task 4 |
| §8 degradations | Task 4 (`resolveFootprintPoints`, empty costmap) |
| §9 acceptance | Task 2 tests + Task 4 build/smoke + Task 5 |
| §3 / §11 non-goals | Not implemented (explicit) |

## Placeholder / consistency check

- Schema id consistently `orbisview.render.RobotFootprint` / `kSchemaFootprint` / `SCHEMAS.RobotFootprint`.
- Channel names `/orbisview/mock/costmap` and `/orbisview/mock/footprint` match FE default subscribe.
- `drawOccupancy` modes `'map' | 'costmap'` only.
