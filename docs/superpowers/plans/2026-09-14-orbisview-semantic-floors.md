# OrbisView Semantic Zones & Multi-Floor (P3) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.
>
> **Note:** If subagent billing fails, implement **inline** in the parent session (same task order).

**Goal:** Display Strata-aligned semantic zone polygons on Map2D/View3D and switch floors (updating P1 basemap), fed by static JSON fixtures and live channels.

**Architecture:** Pure parsers in `renderer/map2d/semanticZones.ts` + `floorBasemap.ts`; zustand `indoorMapStore` holds floors/zones/activeFloorId; live envelopes merged via `IndoorMapEffects` (mounted once under Orbisview); 2D `drawSemantic` between costmap and vectormap; View3D ground meshes; floor switch calls `staticSlamStore.setBasemap`. No MapLibre/BICMap npm.

**Tech Stack:** Vite/React/TypeScript, Canvas2D, `three`, zustand persist, vitest; backend mock C++ + optional Automsgs converter.

**Spec:** `docs/superpowers/specs/2026-09-14-orbisview-semantic-floors-design.md`

## Global Constraints

- No new deps: `@x-humanoid-cloud/bic-map`, `maplibre-gl`, `@turf/turf`.
- Layer key exactly `semantic`; persist bump `orbisview-layers-v6` → **`orbisview-layers-v7`** with `semantic: true` default + migrate missing key.
- Schemas: `orbisview.render.SemanticZoneArray`, `orbisview.render.FloorInfoArray`.
- Draw cap: max **200** zones per frame (slice); labels only if `scale` large enough or zone count ≤ 40.
- Floor switch blocked when `mappingVizStore.demo.playing` (pause-first message via `setStatusMsg`).
- Persist `orbisview-indoor-map-v1`: only `activeFloorId` (validated against current floors on hydrate).
- Working directory frontend: `autonomy/orbisview/frontend`.
- Non-goals: P4 POI/edit, cross-floor pose/path filtering, strata proto changes.

## File map

| File | Responsibility |
|------|----------------|
| `frontend/src/renderer/map2d/semanticZones.ts` | Normalize wire JSON → `SemanticZoneNorm`; palette; rgba CSS |
| `frontend/src/renderer/map2d/semanticZones.test.ts` | Polygon/color tests |
| `frontend/src/renderer/map2d/floorBasemap.ts` | `floorToBasemapMeta` / `basemapFromFloor` |
| `frontend/src/renderer/map2d/floorBasemap.test.ts` | Meta mapping tests |
| `frontend/src/renderer/map2d/drawSemantic.ts` | Canvas2D fill/stroke/label |
| `frontend/src/store/indoorMapStore.ts` | floors, zones, active, fixture/live actions |
| `frontend/src/store/indoorMapStore.test.ts` | setActiveFloor / demo block |
| `frontend/src/store/layoutStore.ts` | `semantic` + v7 |
| `frontend/src/renderer/map2d/types.ts` | `semantic?` on `LayerFlags` |
| `frontend/src/store/websocket/types.ts` | SCHEMAS entries |
| `frontend/src/schemas.test.ts` | Assert new schemas |
| `frontend/src/components/Channels/displayTypes.ts` | Display type defs |
| `frontend/src/components/Channels/mapDisplayBinding.ts` | roles `semantic` / `floors` |
| `frontend/src/hooks/useIndoorMapLiveSync.ts` | envelopes → store |
| `frontend/src/components/IndoorMapEffects.tsx` | Mount sync once |
| `frontend/src/renderer/map2d/drawScene.ts` | Call drawSemantic |
| `frontend/src/renderer/view3d/layers/semantic.ts` | 3D meshes |
| `frontend/src/renderer/view3d/createScene.ts` / `syncScene.ts` / `types.ts` | Wire |
| `frontend/src/components/Map/MapFloorBar.tsx` | ‹ name › |
| `frontend/src/components/Tasks/IndoorMapPanel.tsx` | Full UI |
| `frontend/src/components/index.ts` | Register panel |
| `frontend/src/components/Map2D/Map2DPanel.tsx` | FloorBar + zones into draw |
| `frontend/src/components/View3D/View3DPanel.tsx` | Pass zones |
| `frontend/src/components/Orbisview.tsx` | Effects + DEFAULT_CHANNELS |
| `frontend/src/styles/main.css` | Floor bar + panel |
| `backend/common/render_schemas.h` | Schema constants |
| `backend/proto/render.proto` (or JSON-only) | Optional messages; **lock:** mock may emit JSON without new proto if Mapping-style already in render.proto — **add** `SemanticZoneArray` / `FloorInfoArray` messages mirroring strata fields for consistency |
| `backend/adapters/mock/mock_source.cc` | Channels + emit |
| `backend/adapters/automsgs/automsgs_converter.cc` | strata → orbisview JSON |

---

### Task 1: Normalize semantic zones (TDD)

**Files:**
- Create: `autonomy/orbisview/frontend/src/renderer/map2d/semanticZones.ts`
- Create: `autonomy/orbisview/frontend/src/renderer/map2d/semanticZones.test.ts`

**Interfaces:**

```ts
export interface SemanticZoneNorm {
  id: string;
  zoneType: string;
  polygon: [number, number][]; // ≥3 points
  fill: string;                 // css rgba()
  stroke: string;
  strokeWidth: number;
  label?: string;
}

export function normalizePolygon(raw: unknown): [number, number][] | null;
export function normalizeSemanticZone(raw: unknown): SemanticZoneNorm | null;
export function normalizeSemanticZoneArray(payload: unknown): SemanticZoneNorm[];
export function zoneTypePalette(zoneType: string): { fill: string; stroke: string };
export function colorRgbaToCss(
  c: { r?: number; g?: number; b?: number; a?: number } | undefined,
  opacity?: number,
): string | null;
```

**Rules:**
- Accept `{x,y}` or `[x,y]`; drop invalid points; need ≥3.
- Prefer wire colors; else `zoneTypePalette` (`keepout` red, `passable` green, `room` blue, default gray).
- `fill_opacity` multiplies alpha when present (0–1).

- [ ] **Step 1: Write failing tests** (array vs object points; palette fallback; opacity)

- [ ] **Step 2: Run FAIL**

```bash
cd autonomy/orbisview/frontend && npx vitest run src/renderer/map2d/semanticZones.test.ts
```

- [ ] **Step 3: Implement**

- [ ] **Step 4: PASS + Commit**

```bash
git commit -m "$(cat <<'EOF'
feat(orbisview): normalize semantic zone polygons and colors

Accept strata-like wire JSON with palette fallback by zone_type.
EOF
)"
```

---

### Task 2: Floor → basemap mapping (TDD)

**Files:**
- Create: `floorBasemap.ts` + `.test.ts`

**Interfaces:**

```ts
export interface FloorInfoNorm {
  id: string;
  name: string;
  level: number;
  slamImagePath?: string;
  originX: number;
  originY: number;
  widthPx: number;
  heightPx: number;
  resolution: number;
}

export function normalizeFloorInfo(raw: unknown): FloorInfoNorm | null;
export function normalizeFloorInfoArray(payload: unknown): {
  floors: FloorInfoNorm[];
  activeFloorId: string | null;
};
/** Sort by level then name */
export function sortFloors(floors: FloorInfoNorm[]): FloorInfoNorm[];

export function floorHasFetchableSlam(f: FloorInfoNorm): boolean;
/** Builds StaticSlamBasemap when path fetchable; else null */
export function basemapFromFloor(f: FloorInfoNorm): import('./staticSlam').StaticSlamBasemap | null;
```

`floorHasFetchableSlam`: `slamImagePath` non-empty and (`http(s):`, `blob:`, `/`, or relative without `file:`). Relative paths resolve against `window.location.origin` when applying (document in helper comment).

- [ ] **Step 1–4: TDD + Commit**

```bash
git commit -m "$(cat <<'EOF'
feat(orbisview): map FloorInfo fields onto StaticSlamBasemap meta

Normalize floors array and detect fetchable slam_image_path.
EOF
)"
```

---

### Task 3: indoorMapStore

**Files:**
- Create: `indoorMapStore.ts` + `.test.ts`

**Behavior lock:**

```ts
setFromFixture(f): source='static'; replace floors+zones; set active from fixture or first floor; applyBasemapForActive()
applyLiveFloors(arr): source='live'; merge floors/active (keep zones unless also live)
applyLiveZones(zones): source='live'; replace zones
setActiveFloor(id):
  if mappingViz.demo.playing → setStatusMsg('请先暂停建图演示'); return false
  else set active; applyBasemapForActive(); return true
applyBasemapForActive():
  floor = find; if basemapFromFloor → setBasemap + layer basemap on; else setStatusMsg('该楼层无底图')
clearStatic(): if source==='static' clear floors/zones/active (live untouched until next envelope)
stepFloor(delta: -1|1): neighbor in sortFloors order
```

Persist partialize: `{ activeFloorId }` only.

- [ ] **Step 1: Tests** — setActiveFloor builds basemap when path http; blocked when playing

- [ ] **Step 2: Implement + PASS + Commit**

```bash
git commit -m "$(cat <<'EOF'
feat(orbisview): add indoorMapStore for floors and semantic zones

Floor switch updates P1 basemap; blocks while mapping demo plays.
EOF
)"
```

---

### Task 4: Layer `semantic` + v7 persist

**Files:** `layoutStore.ts`, `types.ts` (`LayerFlags`), any Layers UI / `panelIcons` / `LAYER_ICONS` if present

- Add `semantic: true` default
- Persist name `orbisview-layers-v7`
- merge: `semantic: p.semantic ?? true` (and keep `basemap` migrate)

- [ ] **Step 1: Implement**

- [ ] **Step 2: Commit**

```bash
git commit -m "$(cat <<'EOF'
feat(orbisview): add semantic layer key with layers-v7 persist

Default on; migrate missing key like basemap.
EOF
)"
```

---

### Task 5: Schemas + Channels binding

**Files:** `types.ts` (SCHEMAS), `schemas.test.ts`, `displayTypes.ts`, `mapDisplayBinding.ts`

- SCHEMAS + roles: `semantic` → layer `semantic`; `floors` → no layer toggle (store-driven) OR soft-bind without LayerKey
- `ROLE_FALLBACK_SCHEMAS` for both
- Display types: `strata_msgs/SemanticZoneArray`, `FloorInfoArray` aliases

- [ ] **Step 1: Implement + schemas.test PASS**

- [ ] **Step 2: Commit**

```bash
git commit -m "$(cat <<'EOF'
feat(orbisview): register SemanticZoneArray and FloorInfoArray schemas

Channels display types match strata aliases for live binding.
EOF
)"
```

---

### Task 6: Map2D drawSemantic

**Files:** `drawSemantic.ts`, `drawScene.ts`, `Map2DPanel.tsx`

Insert after costmap, before vectormap:

```ts
if (layers.semantic && semanticZones?.length) {
  drawSemanticZones(ctx, semanticZones, toScreen, scale);
}
```

Pass `semanticZones` from `useIndoorMapStore(s => s.zones)` (already normalized).

- [ ] **Step 1: Implement draw + wire**

- [ ] **Step 2: Commit**

```bash
git commit -m "$(cat <<'EOF'
feat(orbisview): draw semantic zones on Map2D under vectormap

Fill/stroke/label with draw cap and scale-aware labels.
EOF
)"
```

---

### Task 7: View3D semantic layer

**Files:** `view3d/layers/semantic.ts`, `createScene.ts`, `syncScene.ts`, `types.ts`, `View3DPanel.tsx`

- Group `semanticGroup`; clear/rebuild meshes when zones identity changes (hash of ids+point counts)
- Mesh: THREE.Shape from polygon; material transparent; yLift ~0.01 above basemap
- Edges: LineLoop optional

- [ ] **Step 1: Implement + wire visible=layers.semantic**

- [ ] **Step 2: Commit**

```bash
git commit -m "$(cat <<'EOF'
feat(orbisview): render semantic zone meshes in View3D

Ground-aligned translucent polygons shared with Map2D data.
EOF
)"
```

---

### Task 8: Live sync + UI (FloorBar + Panel)

**Files:**
- `useIndoorMapLiveSync.ts` — pick envelopes for semantic/floors roles (or schema scan); normalize; `applyLive*`
- `IndoorMapEffects.tsx` — call hook; mount in Orbisview
- `MapFloorBar.tsx` — prev/next/name; hide if floors.length===0
- `IndoorMapPanel.tsx` — list, legend, file/paste fixture, clear
- `Orbisview.tsx` — DEFAULT_CHANNELS add mock paths
- `main.css` — styles
- `components/index.ts` — register `indoor_map` panel (category planning)

Fixture load: `JSON.parse` → normalize floors+zones → `setFromFixture`.

- [ ] **Step 1: Effects + FloorBar on Map2D**

- [ ] **Step 2: IndoorMapPanel + register**

- [ ] **Step 3: Commit**

```bash
git commit -m "$(cat <<'EOF'
feat(orbisview): indoor map panel, floor bar, and live sync

Static fixture load plus envelope-driven floors/zones updates.
EOF
)"
```

---

### Task 9: Backend mock + Automsgs converter

**Files:**
- `render.proto` — add messages mirroring strata (zones + floors arrays)
- `render_schemas.h` — `kSchemaSemanticZones`, `kSchemaFloors`
- `mock_source.cc` — channels `/orbisview/mock/semantic_zones`, `/orbisview/mock/floors`; emit 2 floors + 2–3 zones periodically (or static JSON once per tick)
- `automsgs_converter.cc` — map `automsgs.msgs.strata_msgs.SemanticZoneArray` / `FloorInfoArray` → JSON payloads

Fixture geometry: simple rectangles around origin matching existing mock map scale (~10–20 m).

- [ ] **Step 1: Schemas + mock emit**

- [ ] **Step 2: Converter**

- [ ] **Step 3: Commit**

```bash
git commit -m "$(cat <<'EOF'
feat(orbisview): mock and convert semantic zones and floors channels

Publish orbisview.render.SemanticZoneArray / FloorInfoArray JSON.
EOF
)"
```

---

### Task 10: Full verification

- [ ] **Step 1:**

```bash
cd autonomy/orbisview/frontend && npm test && npm run build
```

- [ ] **Step 2: Manual checklist**

1. IndoorMapPanel paste fixture → zones visible; floor switch changes basemap when path set
2. Mock on → live floors/zones; toggle `semantic` layer
3. 2D↔3D alignment; measure/nav OK
4. Mapping demo playing → floor switch blocked with status message

- [ ] **Step 3: Fixups only if needed**

---

## Spec coverage

| Spec § | Task |
|--------|------|
| Zone normalize / palette | T1 |
| Floor → basemap | T2 |
| indoorMapStore | T3 |
| Layer semantic v7 | T4 |
| Schemas / Channels | T5 |
| Map2D draw order | T6 |
| View3D | T7 |
| UI + live sync | T8 |
| Mock + converter | T9 |
| Tests + build | T1–T3, T10 |

## Type consistency

- Store: `useIndoorMapStore` / `indoorMapStore.ts`
- Persist: `orbisview-indoor-map-v1`, `orbisview-layers-v7`
- Layer key: `semantic`
- Schemas: `orbisview.render.SemanticZoneArray`, `orbisview.render.FloorInfoArray`
- Effects once via `IndoorMapEffects`; FloorBar only on Map2D
