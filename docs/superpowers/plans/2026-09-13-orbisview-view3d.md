# OrbisView Ground Robot View3D Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Ship ground-robot View3D: extract Three.js into `renderer/view3d/`, follow/free camera, layers for robot/path/goal/pointcloud/footprint/occupancy/costmap/laser, shared layer toggles + 3D opts.

**Architecture:** `View3DPanel` subscribes to envelopes and drives a pure `renderer/view3d` context (`createScene` → `syncScene` → rAF). Camera mode is driven by existing `useLayerStore.followRobot`. Coordinate map→Three is centralized in `coords.ts`. No R3F/Babylon; reuse wire JSON and Map2D footprint helpers.

**Tech Stack:** Vite/React/TypeScript, `three` ^0.170, zustand persist, vitest.

**Spec:** `docs/superpowers/specs/2026-09-13-orbisview-view3d-design.md`

## Global Constraints

- Keep dependency `three` only — do **not** add `@react-three/fiber` or Babylon.
- Point cloud wire JSON stays `{"points":[{"x","y","z","i"?},…]}` (mock + converter); do not switch FE to proto `xyz` packed floats.
- Reuse `useLayerStore` LayerKeys; 3D-only opts live in a separate persist store `orbisview-view3d-opts-v1`.
- Camera follow uses existing `followRobot` / `setFollowRobot` (single source of truth).
- Occupancy texture longest edge ≤ **512**; build textures only when layer is on.
- Map↔Three: `(x_t,y_t,z_t) = (x, z||0, −y)` via `toThree`.
- Prefer small files under `frontend/src/renderer/view3d/`.
- Non-goals: GLTF car model, point picking, new Costmap proto, raising 500-point backend cap.

## File map

| File | Responsibility |
|------|----------------|
| `frontend/src/renderer/view3d/coords.ts` | `toThree` / helpers |
| `frontend/src/renderer/view3d/types.ts` | `View3DSceneInput`, cloud/laser/occupancy types |
| `frontend/src/renderer/view3d/coords.test.ts` | Unit tests for coords |
| `frontend/src/renderer/view3d/createScene.ts` | WebGL scene lifecycle → `View3DContext` |
| `frontend/src/renderer/view3d/cameraController.ts` | Follow/free orbit + reset |
| `frontend/src/renderer/view3d/syncScene.ts` | Apply input + layers + opts |
| `frontend/src/renderer/view3d/layers/robot.ts` | Pose mesh |
| `frontend/src/renderer/view3d/layers/path.ts` | Path + goal line |
| `frontend/src/renderer/view3d/layers/cloud.ts` | Point cloud buffer update |
| `frontend/src/renderer/view3d/layers/footprint.ts` | Footprint loop/extrude |
| `frontend/src/renderer/view3d/layers/occupancy.ts` | Map/costmap planes + texture |
| `frontend/src/renderer/view3d/layers/laser.ts` | Laser points/lines |
| `frontend/src/renderer/view3d/index.ts` | Public exports |
| `frontend/src/renderer/index.ts` | Re-export view3d public API |
| `frontend/src/store/view3dStore.ts` | `view3dOpts` persist |
| `frontend/src/components/View3D/View3DPanel.tsx` | Thin panel |
| `frontend/src/styles/main.css` | View3D toolbar styles |
| `frontend/src/schemas.test.ts` | Persist key assert (optional) |
| `proto/render.proto` | PointCloud2 comment ↔ wire |
| `frontend/README.md` | View3D acceptance note |

---

### Task 1: coords + types + unit test

**Files:**
- Create: `autonomy/orbisview/frontend/src/renderer/view3d/coords.ts`
- Create: `autonomy/orbisview/frontend/src/renderer/view3d/types.ts`
- Create: `autonomy/orbisview/frontend/src/renderer/view3d/coords.test.ts`

**Interfaces:**
- Produces:
  - `toThree(x: number, y: number, z?: number): { x: number; y: number; z: number }`
  - Types: `View3DCloudPoint`, `View3DSceneInput`, `View3DOpts`, `View3DLayerFlags`

- [ ] **Step 1: Write failing coords test**

```ts
import { describe, expect, it } from 'vitest';
import { toThree } from './coords';

describe('toThree', () => {
  it('maps map XY to Three X/−Z with optional height on Y', () => {
    expect(toThree(1, 2)).toEqual({ x: 1, y: 0, z: -2 });
    expect(toThree(1, 2, 0.5)).toEqual({ x: 1, y: 0.5, z: -2 });
  });
});
```

- [ ] **Step 2: Run test — expect FAIL**

Run: `cd autonomy/orbisview/frontend && npm test -- src/renderer/view3d/coords.test.ts`

Expected: FAIL (module not found / `toThree` undefined).

- [ ] **Step 3: Implement coords + types**

`coords.ts`:

```ts
export function toThree(x: number, y: number, z = 0): { x: number; y: number; z: number } {
  return { x, y: z, z: -y };
}
```

`types.ts` (exact shapes later tasks consume):

```ts
import type { OccupancyGridJson, Pose2D, RobotFootprintJson } from '../map2d/types';

export type CloudColorMode = 'height' | 'intensity';

export interface View3DOpts {
  cloudColor: CloudColorMode;
  laserHeight: number;
  mapOpacity: number;
}

export interface View3DLayerFlags {
  grid: boolean;
  robot: boolean;
  path: boolean;
  pointcloud: boolean;
  footprint: boolean;
  map: boolean;
  costmap: boolean;
  laser: boolean;
}

export interface View3DCloudPoint {
  x: number;
  y: number;
  z: number;
  i?: number;
}

export interface View3DLaserScan {
  angle_min: number;
  angle_increment: number;
  ranges: number[];
}

export interface View3DNavGoal {
  x: number;
  y: number;
}

export interface View3DSceneInput {
  pose: Pose2D | null;
  path: Pose2D[] | null;
  goal: View3DNavGoal | null;
  cloud: View3DCloudPoint[] | null;
  footprint: RobotFootprintJson | null;
  map: OccupancyGridJson | null;
  costmap: OccupancyGridJson | null;
  laser: View3DLaserScan | null;
  layers: View3DLayerFlags;
  opts: View3DOpts;
  followRobot: boolean;
}
```

- [ ] **Step 4: Run test — expect PASS**

Run: `cd autonomy/orbisview/frontend && npm test -- src/renderer/view3d/coords.test.ts`

Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add autonomy/orbisview/frontend/src/renderer/view3d/coords.ts \
  autonomy/orbisview/frontend/src/renderer/view3d/types.ts \
  autonomy/orbisview/frontend/src/renderer/view3d/coords.test.ts
git commit -m "feat(orbisview): add View3D coords and scene input types"
```

---

### Task 2: createScene skeleton + Panel mount/resize

**Files:**
- Create: `autonomy/orbisview/frontend/src/renderer/view3d/createScene.ts`
- Create: `autonomy/orbisview/frontend/src/renderer/view3d/index.ts`
- Modify: `autonomy/orbisview/frontend/src/renderer/index.ts`
- Modify: `autonomy/orbisview/frontend/src/components/View3D/View3DPanel.tsx` (replace monolithic Three setup with createScene; keep temporary inline sync OR empty sync until Task 4)

**Interfaces:**
- Consumes: `three`
- Produces:
  - `View3DContext` with `renderer`, `scene`, `camera`, `grid`, `robot`, `goal`, `goalLine`, `path`, `cloud`, `footprint`, `mapPlane`, `costmapPlane`, `laser`, `dispose()`, `setSize(w,h)`
  - `createView3DScene(mount: HTMLElement): View3DContext`

- [ ] **Step 1: Implement `createView3DScene`**

Create meshes/groups for all layer objects (even if unused until later tasks). Initial camera FOV 55, near 0.1, far 200. Background `#0f1419`. GridHelper(20,20). Directional + ambient lights. Robot cone (green), goal cone (orange, hidden), dashed goal line, path line, empty Points cloud, empty footprint LineLoop, two Planes for map/costmap (hidden), empty laser Points. Append `renderer.domElement` to `mount`.

`setSize(w,h)`: `renderer.setSize(w,h,false)`; update `camera.aspect`; `camera.updateProjectionMatrix()`; set canvas CSS `width/height: 100%`.

`dispose()`: cancel nothing here (Panel owns rAF); dispose geometries/materials/renderer; remove canvas from mount.

- [ ] **Step 2: Export from `view3d/index.ts` and `renderer/index.ts`**

```ts
// view3d/index.ts
export { toThree } from './coords';
export { createView3DScene } from './createScene';
export type { View3DContext } from './createScene';
export type { View3DSceneInput, View3DOpts } from './types';
```

Re-export the same from `renderer/index.ts`.

- [ ] **Step 3: Slim Panel mount**

In `View3DPanel.tsx` mount effect: call `createView3DScene(mount)`, `ResizeObserver` → `setSize`, rAF render loop, cleanup `dispose`. Temporarily keep the existing envelope→geometry logic against `ctx` fields (robot/path/cloud/…) so the panel still works until Task 4 extracts `syncScene`. Fix fixed 720×420: size from `mount.clientWidth/Height` (min height 240).

- [ ] **Step 4: Verify build**

Run: `cd autonomy/orbisview/frontend && npm run build`

Expected: PASS (tsc + vite).

- [ ] **Step 5: Commit**

```bash
git add autonomy/orbisview/frontend/src/renderer/view3d \
  autonomy/orbisview/frontend/src/renderer/index.ts \
  autonomy/orbisview/frontend/src/components/View3D/View3DPanel.tsx
git commit -m "feat(orbisview): extract View3D createScene and responsive mount"
```

---

### Task 3: cameraController (follow / free / reset)

**Files:**
- Create: `autonomy/orbisview/frontend/src/renderer/view3d/cameraController.ts`
- Modify: `autonomy/orbisview/frontend/src/components/View3D/View3DPanel.tsx`
- Modify: `autonomy/orbisview/frontend/src/styles/main.css` (toolbar row)

**Interfaces:**
- Consumes: `THREE.PerspectiveCamera`, `toThree`
- Produces:
  - `createCameraController(camera): CameraController`
  - `CameraController`: `{ yaw, pitch, distance, target, setFollow(v), setTarget(x,y), onPointerDown/Move/Up, onWheel, reset(), update() }`
  - Defaults: `yaw=0.8`, `pitch=0.6`, `distance=8`

- [ ] **Step 1: Implement controller**

```ts
// Pseudocode behavior (implement fully in TS):
update():
  const t = target // Vector3 from setTarget / follow pose
  const cp = Math.cos(pitch)
  camera.position.set(
    t.x + distance * cp * Math.sin(yaw),
    t.y + distance * Math.sin(pitch),
    t.z + distance * cp * Math.cos(yaw),
  )
  camera.lookAt(t)

setFollow(true): orbit around target only (no pan)
setFollow(false): drag with shift or middle button pans target in XZ; else orbit
reset(): yaw=0.8; pitch=0.6; distance=8; if follow keep target else target→(0,0,0)
```

Pointer handlers attach to `renderer.domElement` inside Panel (or controller.attach(el)).

- [ ] **Step 2: Wire Panel**

Each sync frame: if `followRobot && pose`, `setTarget(toThree(pose.x, pose.y).x, 0, toThree(...).z)` then `update()`. Toolbar buttons: Follow (toggles `setFollowRobot`), Free, Reset.

Use existing:

```ts
const followRobot = useLayerStore((s) => s.followRobot);
const setFollowRobot = useLayerStore((s) => s.setFollowRobot);
```

- [ ] **Step 3: CSS**

```css
.view3d-host { position: relative; width: 100%; min-height: 240px; flex: 1; }
.view3d-host canvas { display: block; width: 100%; height: 100%; }
.view3d-toolbar { display: flex; gap: 0.35rem; flex-wrap: wrap; margin-bottom: 0.35rem; }
```

- [ ] **Step 4: Manual smoke**

Connect mock → drag orbit → toggle Follow/Free → Reset. Robot motion should pull lookAt when Follow on.

- [ ] **Step 5: Commit**

```bash
git add autonomy/orbisview/frontend/src/renderer/view3d/cameraController.ts \
  autonomy/orbisview/frontend/src/components/View3D/View3DPanel.tsx \
  autonomy/orbisview/frontend/src/styles/main.css
git commit -m "feat(orbisview): View3D follow/free camera controller"
```

---

### Task 4: syncScene + migrate robot/path/goal/cloud/grid

**Files:**
- Create: `autonomy/orbisview/frontend/src/renderer/view3d/syncScene.ts`
- Create: `autonomy/orbisview/frontend/src/renderer/view3d/layers/robot.ts`
- Create: `autonomy/orbisview/frontend/src/renderer/view3d/layers/path.ts`
- Create: `autonomy/orbisview/frontend/src/renderer/view3d/layers/cloud.ts`
- Modify: `autonomy/orbisview/frontend/src/components/View3D/View3DPanel.tsx` (Panel only builds `View3DSceneInput` + calls `syncView3DScene`)
- Modify: `autonomy/orbisview/frontend/src/renderer/view3d/index.ts`

**Interfaces:**
- Produces: `syncView3DScene(ctx: View3DContext, input: View3DSceneInput): void`
- Layer helpers: `updateRobot`, `updatePathAndGoal`, `updateCloud` (visibility + geometry)

- [ ] **Step 1: Implement layer helpers using `toThree`**

Robot: position from pose; `rotation.z = -(yaw)`; visible ↔ `layers.robot`.

Path: `setFromPoints` of `toThree(p.x,p.y,0.05)`; goal cone + dashed line as today.

Cloud: build Float32 positions/colors; `cloudColor==='intensity'` use `i`, else normalize z height to RGB; visible ↔ `layers.pointcloud`.

Grid: `ctx.grid.visible = layers.grid`.

- [ ] **Step 2: `syncView3DScene` orchestrates helpers**

Order: visibility/grid → robot → path/goal → cloud. (Footprint/occupancy/laser no-ops until later — leave objects hidden.)

- [ ] **Step 3: Refactor Panel**

Remove inline geometry mutation; build input from envelopes (same schema picks as today + layers from store). Pass placeholder `opts: { cloudColor:'intensity', laserHeight:0.1, mapOpacity:0.55 }` until Task 8.

- [ ] **Step 4: Build + test**

Run: `cd autonomy/orbisview/frontend && npm test && npm run build`

Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add autonomy/orbisview/frontend/src/renderer/view3d \
  autonomy/orbisview/frontend/src/components/View3D/View3DPanel.tsx
git commit -m "refactor(orbisview): syncView3DScene for robot path goal cloud"
```

---

### Task 5: footprint layer

**Files:**
- Create: `autonomy/orbisview/frontend/src/renderer/view3d/layers/footprint.ts`
- Modify: `autonomy/orbisview/frontend/src/renderer/view3d/syncScene.ts`
- Modify: `autonomy/orbisview/frontend/src/components/View3D/View3DPanel.tsx` (pick `SCHEMAS.RobotFootprint`)

**Interfaces:**
- Consumes: `resolveFootprintPoints` from `renderer/map2d/drawFootprint`, `DEFAULT_FOOTPRINT` from `config/parameters`
- Produces: `updateFootprint(ctx, pose, footprintJson, visible)`

- [ ] **Step 1: Implement updateFootprint**

Resolve body points → transform by pose yaw into map frame → `toThree` → `LineLoop` positions (close loop). Color `#80cbc4`. If no pose, hide.

- [ ] **Step 2: Wire sync + Panel envelope pick**

`layers.footprint` controls visibility.

- [ ] **Step 3: Manual check**

Mock footprint channel already subscribed in Orbisview defaults — toggle Layers → footprint.

- [ ] **Step 4: Commit**

```bash
git add autonomy/orbisview/frontend/src/renderer/view3d \
  autonomy/orbisview/frontend/src/components/View3D/View3DPanel.tsx
git commit -m "feat(orbisview): View3D footprint layer"
```

---

### Task 6: occupancy map + costmap planes

**Files:**
- Create: `autonomy/orbisview/frontend/src/renderer/view3d/layers/occupancy.ts`
- Modify: `autonomy/orbisview/frontend/src/renderer/view3d/syncScene.ts`
- Modify: `autonomy/orbisview/frontend/src/components/View3D/View3DPanel.tsx` (reuse Map2D `pickMapAndCostmap` logic — extract shared helper **or** duplicate the small finder to avoid large Map2D refactor; prefer copy 15-line helper into `view3d/pickGrids.ts` or shared `renderer/pickOccupancy.ts`)

**Interfaces:**
- Produces: `updateOccupancyPlane(plane, grid, opts: { opacity, yLift, maxEdge: 512, tint })`
- sync: map `yLift=0.01`, costmap `yLift=0.02`, costmap opacity `mapOpacity * 0.85`

- [ ] **Step 1: Texture builder**

Paint occupancy cells to offscreen canvas (unknown/free/occupied colors similar to Map2D). Scale so `max(width,height) <= 512`. Create/update `CanvasTexture` (`needsUpdate=true`). Plane size = `width*res` × `height*res`, position at origin + grid origin offset via `toThree(origin.x + w/2*res, origin.y + h/2*res, yLift)`. Rotate plane to XZ (`rotation.x = -π/2`).

- [ ] **Step 2: Skip work when layer off**

If `!layers.map`, hide map plane and skip texture rebuild; same for costmap.

- [ ] **Step 3: Manual check**

Toggle map/costmap in Layers; both panels share switches.

- [ ] **Step 4: Commit**

```bash
git add autonomy/orbisview/frontend/src/renderer/view3d \
  autonomy/orbisview/frontend/src/components/View3D/View3DPanel.tsx
git commit -m "feat(orbisview): View3D occupancy and costmap planes"
```

---

### Task 7: LaserScan layer

**Files:**
- Create: `autonomy/orbisview/frontend/src/renderer/view3d/layers/laser.ts`
- Modify: `autonomy/orbisview/frontend/src/renderer/view3d/syncScene.ts`
- Modify: `autonomy/orbisview/frontend/src/components/View3D/View3DPanel.tsx`

**Interfaces:**
- Produces: `updateLaser(ctx, pose, scan, height, visible)`
- Points in map frame: `x = pose.x + r*cos(yaw+angle)`, `y = pose.y + r*sin(...)`, then `toThree(..., height)`.

- [ ] **Step 1: Implement updateLaser**

Skip `!finite` / out-of-range. Color `#ffcc80`. Use `opts.laserHeight` (default 0.1).

- [ ] **Step 2: Wire `SCHEMAS.LaserScan` + `layers.laser`

- [ ] **Step 3: Manual check**

Toggle laser; confirm ring near robot with mock laser channel.

- [ ] **Step 4: Commit**

```bash
git add autonomy/orbisview/frontend/src/renderer/view3d \
  autonomy/orbisview/frontend/src/components/View3D/View3DPanel.tsx
git commit -m "feat(orbisview): View3D laser scan layer"
```

---

### Task 8: view3dOpts store + UI

**Files:**
- Create: `autonomy/orbisview/frontend/src/store/view3dStore.ts`
- Modify: `autonomy/orbisview/frontend/src/components/View3D/View3DPanel.tsx`
- Modify: `autonomy/orbisview/frontend/src/schemas.test.ts` (persist key pattern)
- Modify: `autonomy/orbisview/frontend/src/styles/main.css`

**Interfaces:**
- Produces zustand store:

```ts
interface View3DStore {
  cloudColor: 'height' | 'intensity';
  laserHeight: number;
  mapOpacity: number;
  setCloudColor: (v: 'height' | 'intensity') => void;
  setLaserHeight: (v: number) => void;
  setMapOpacity: (v: number) => void;
}
// persist name: 'orbisview-view3d-opts-v1'
// defaults: intensity, 0.1, 0.55
```

- [ ] **Step 1: Add failing persist-key test**

```ts
it('uses versioned view3d opts key', () => {
  expect('orbisview-view3d-opts-v1').toMatch(/^orbisview-view3d-opts-v\d+$/);
});
```

- [ ] **Step 2: Implement store + Panel controls**

Toolbar: select cloudColor; range inputs for laserHeight (0–1 step 0.05) and mapOpacity (0.1–1 step 0.05). Pass opts into `View3DSceneInput`.

- [ ] **Step 3: Run tests + build**

Run: `cd autonomy/orbisview/frontend && npm test && npm run build`

Expected: PASS.

- [ ] **Step 4: Commit**

```bash
git add autonomy/orbisview/frontend/src/store/view3dStore.ts \
  autonomy/orbisview/frontend/src/components/View3D/View3DPanel.tsx \
  autonomy/orbisview/frontend/src/schemas.test.ts \
  autonomy/orbisview/frontend/src/styles/main.css
git commit -m "feat(orbisview): View3D opts store and toolbar"
```

---

### Task 9: Proto comment alignment + README + acceptance

**Files:**
- Modify: `autonomy/orbisview/proto/render.proto` (`PointCloud2` comment)
- Modify: `autonomy/orbisview/frontend/README.md`
- Modify: `autonomy/orbisview/README.md` (View3D bullets if present)

**Interfaces:** none new.

- [ ] **Step 1: Update PointCloud2 proto comment**

```protobuf
// Wire JSON (StreamEnvelope.payload) is authoritative for the web UI:
//   {"points":[{"x":float,"y":float,"z":float,"i":float?}, ...]}
// Field `xyz` documents a packed XYZ layout for potential binary paths;
// mock and Autolink converters currently emit the points[] JSON shape.
message PointCloud2 {
  repeated float xyz = 1 [packed = true];
}
```

Do **not** change converter or mock payloads.

- [ ] **Step 2: README acceptance checklist**

Document: Connect → open View3D → Follow/Free → layer toggles → cloudColor / mapOpacity / laserHeight → stale badge.

- [ ] **Step 3: Full verify**

Run: `cd autonomy/orbisview/frontend && npm test && npm run build`

Manual: walk Spec §7 checklist against mock backend.

- [ ] **Step 4: Commit**

```bash
git add autonomy/orbisview/proto/render.proto \
  autonomy/orbisview/frontend/README.md \
  autonomy/orbisview/README.md
git commit -m "docs(orbisview): align PointCloud2 wire notes and View3D acceptance"
```

---

## Spec coverage check

| Spec requirement | Task |
|------------------|------|
| `renderer/view3d/` extract | 2–4 |
| Layers 1–6 (grid/robot/path/goal/cloud/footprint/map/costmap/laser) | 4–7 |
| Follow + free camera + reset | 3 |
| Shared LayerKeys + 3D opts | 4, 8 |
| Three only, no R3F | Global + all tasks |
| Wire `points[]` + proto comment | 4, 9 |
| Texture ≤512 / layer-gated | 6 |
| `npm test` / `npm run build` + manual §7 | 1,4,8,9 |

## Placeholder / consistency scan

- Names locked: `toThree`, `createView3DScene`, `View3DContext`, `syncView3DScene`, `View3DSceneInput`, `orbisview-view3d-opts-v1`.
- No TBD steps; Task 4 placeholder opts replaced by Task 8 store.
