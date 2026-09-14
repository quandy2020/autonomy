# OrbisView POI & Draw Annotations (P4) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.
>
> **Note:** If subagent billing fails, implement **inline** in the parent session (same task order).

**Goal:** Add local POI placement and polygon/polyline drawing on Map2D/View3D with persist, import/export, toolbar tools, and an Annotations panel.

**Architecture:** Pure JSON normalize in `renderer/map2d/annotations.ts`; zustand `annotationStore` owns pois/shapes/draft; extend `MapTool` with `poi` | `draw`; 2D `drawAnnotations` after vectormap; View3D `layers/annotations.ts`; panel for list CRUD. No MapLibre/BICMap npm; no live write-back.

**Tech Stack:** Vite/React/TypeScript, Canvas2D, `three`, zustand persist, vitest.

**Spec:** `docs/superpowers/specs/2026-09-14-orbisview-poi-draw-annotations-design.md`

## Global Constraints

- No new deps: `@x-humanoid-cloud/bic-map`, `maplibre-gl`, `@turf/turf`.
- `MapTool` exactly: `'pan' | 'measure' | 'nav' | 'pick' | 'poi' | 'draw'`.
- Layer keys: `poi`, `draw`; persist bump **`orbisview-layers-v8`** (migrate both default `true`).
- Annotation persist: `orbisview-annotations-v1` — pois, shapes, defaults; **never** draft.
- Caps: max **500** pois; max **200** points per shape; reject commit below min points.
- `poi` tool: no map pan drag (click places / drag moves POI only).
- Working directory: `autonomy/orbisview/frontend`.
- Non-goals: semantic edit, live write-back, floor-filtered annotations, GPS trail.

## File map

| File | Responsibility |
|------|----------------|
| `frontend/src/renderer/map2d/annotations.ts` | Types + normalize import JSON + id helper |
| `frontend/src/renderer/map2d/annotations.test.ts` | Import validation |
| `frontend/src/store/annotationStore.ts` | CRUD, draft, import/export, persist |
| `frontend/src/store/annotationStore.test.ts` | commit/clear/import boundaries |
| `frontend/src/store/mapViewStore.ts` | MapTool + hints |
| `frontend/src/store/layoutStore.ts` | `poi`/`draw` + v8 |
| `frontend/src/renderer/map2d/types.ts` | LayerFlags |
| `frontend/src/renderer/map2d/drawAnnotations.ts` | 2D paint + draft rubber-band |
| `frontend/src/renderer/map2d/drawScene.ts` | Call after vectormap |
| `frontend/src/renderer/view3d/layers/annotations.ts` | 3D meshes |
| `frontend/src/renderer/view3d/createScene.ts` / `syncScene.ts` / `types.ts` | Wire group |
| `frontend/src/components/Map/MapFloatToolbar.tsx` | Tool buttons |
| `frontend/src/components/Tasks/AnnotationsPanel.tsx` | Panel UI |
| `frontend/src/components/index.ts` | Register panel |
| `frontend/src/components/icons/Icon.tsx` / `panelIcons.ts` | Icons |
| `frontend/src/components/Map2D/Map2DPanel.tsx` | Interaction |
| `frontend/src/components/View3D/View3DPanel.tsx` | Interaction |
| `frontend/src/styles/main.css` | Panel styles |
| `frontend/src/schemas.test.ts` | layers-v8 string smoke |

**Hit-test helper (lock):** put `hitTestPoi(pois, worldX, worldY, radiusM)` in `annotations.ts` for shared 2D/3D use (3D converts ray hit to map x/y first).

---

### Task 1: Annotation JSON normalize (TDD)

**Files:**
- Create: `annotations.ts` + `.test.ts`

**Interfaces:**

```ts
export type PoiKind = 'charger' | 'elevator' | 'custom';
export type DrawShapeKind = 'polygon' | 'polyline';

export interface MapPoi { id: string; x: number; y: number; yaw?: number; kind: PoiKind; label?: string; color?: string; }
export interface MapDrawShape { id: string; kind: DrawShapeKind; points: [number, number][]; label?: string; stroke?: string; fill?: string; }

export function newAnnotationId(): string; // crypto.randomUUID fallback
export function normalizePoi(raw: unknown): MapPoi | null;
export function normalizeShape(raw: unknown): MapDrawShape | null;
export function parseAnnotationFixture(text: string): { pois: MapPoi[]; shapes: MapDrawShape[] };
export function serializeAnnotations(pois: MapPoi[], shapes: MapDrawShape[]): string;
export function hitTestPoi(pois: MapPoi[], x: number, y: number, radiusM: number): MapPoi | null;
export function poiKindColor(kind: PoiKind): string;
```

`parseAnnotationFixture`: throw on invalid JSON; skip bad entries; return valid subset (or throw if `version` present and ≠1 — lock: **warn via skip**, accept missing version).

- [ ] **Step 1: Failing tests** (valid fixture, reject short polygon, hitTest)

- [ ] **Step 2: Implement + PASS**

- [ ] **Step 3: Commit**

```bash
git commit -m "$(cat <<'EOF'
feat(orbisview): normalize POI and draw annotation JSON

Parse fixture v1, hit-test helpers, and kind color defaults.
EOF
)"
```

---

### Task 2: annotationStore (TDD)

**Files:** `annotationStore.ts` + `.test.ts`

**Behavior lock:**

- `addPoi({x,y,...})` — generate id; enforce max 500 (no-op + return false if full)
- `commitDraft()` — polygon need ≥3, polyline ≥2; else return false; clear draft on success
- `undoDraftPoint()` — pop last; if empty keep draft with []
- `importJson(text)` — parse; **replace** pois/shapes on success; on throw leave state unchanged
- `exportJson()` — `serializeAnnotations`
- `clearAll()` — empty arrays + cancel draft
- `selectedPoiId` / `selectedShapeId` optional for panel sync — **add** `selectedId: string | null` + `setSelected`

Persist partialize: `{ pois, shapes, poiDefaultKind, drawDefaultKind }`.

- [ ] **Step 1–4: TDD + Commit**

```bash
git commit -m "$(cat <<'EOF'
feat(orbisview): add annotationStore for local POI and shapes

Persist annotations; draft commit enforces point minimums.
EOF
)"
```

---

### Task 3: MapTool + layers v8

**Files:** `mapViewStore.ts`, `layoutStore.ts`, `types.ts` LayerFlags, `panelIcons.ts` LAYER_ICONS, `schemas.test.ts`

- Extend `MapTool` + `TOOL_HINT` + `normalizeTool`
- Bump map-view persist if needed (tool enum) — keep `orbisview-map-view-v3` but normalize unknown → `nav`
- Layers: `poi: true`, `draw: true`; name `orbisview-layers-v8`; merge defaults

- [ ] **Step 1: Implement**

- [ ] **Step 2: Commit**

```bash
git commit -m "$(cat <<'EOF'
feat(orbisview): add poi/draw map tools and layers-v8

Toolbar tools and independent poi/draw layer toggles.
EOF
)"
```

---

### Task 4: Map2D drawAnnotations + drawScene

**Files:** `drawAnnotations.ts`, `drawScene.ts`

```ts
drawAnnotations(ctx, {
  pois, shapes, draft, draftPreview?, selectedId?,
  layers: { poi, draw },
  toScreen, scale,
})
```

- Shapes: fill polygon, stroke poly/line; draft dashed + rubber-band to preview
- POIs: filled circle by kind color + label if scale large
- Selected: thicker ring

Insert in `paintMap2DScene` after vectormap, before obstacles (or before path — **lock: after vectormap, before obstacles**).

- [ ] **Step 1: Implement + wire `semanticZones`-style input fields**

- [ ] **Step 2: Commit**

```bash
git commit -m "$(cat <<'EOF'
feat(orbisview): draw POI and annotation shapes on Map2D

Include draft rubber-band preview and selection highlight.
EOF
)"
```

---

### Task 5: View3D annotations layer

**Files:** `view3d/layers/annotations.ts`, create/sync/types, View3DPanel input

- Group `annotationGroup`; rebuild on key change (ids + point counts + draft len)
- POI: small cone at `toThree(x,y,0.02)` colored by kind
- polygon/polyline: same approach as semantic layer
- Draft: Line with dashed material if available, else solid accent

- [ ] **Step 1: Implement + sync `layers.poi` / `layers.draw`**

- [ ] **Step 2: Commit**

```bash
git commit -m "$(cat <<'EOF'
feat(orbisview): render annotation POIs and shapes in View3D

Ground-aligned markers shared with Map2D annotation store.
EOF
)"
```

---

### Task 6: Map2D / View3D interaction

**Files:** `Map2DPanel.tsx`, `View3DPanel.tsx`

**poi tool:**

- pointerdown on empty → `addPoi`
- pointerdown on hit POI → start drag; pointermove updates `updatePoi`; pointerup end
- Delete key → remove selected

**draw tool:**

- click → `appendDraftPoint` (ensure draft exists with `drawDefaultKind`)
- dblclick / contextmenu → `commitDraft` or cancel
- Esc → `cancelDraft`
- optional: Backspace → `undoDraftPoint`

When leaving tool (`selectTool` other): `cancelDraft`.

Status messages via existing `setStatusMsg`.

- [ ] **Step 1: Map2D**

- [ ] **Step 2: View3D** (reuse ground hit from pick/nav)

- [ ] **Step 3: Commit**

```bash
git commit -m "$(cat <<'EOF'
feat(orbisview): wire poi and draw tool interactions on Map 2D/3D

Place, drag, commit drafts with the same gestures as measure/nav.
EOF
)"
```

---

### Task 7: Toolbar + Annotations panel

**Files:** `MapFloatToolbar.tsx`, `AnnotationsPanel.tsx`, `index.ts`, `Icon.tsx`, `main.css`

- TOOL_META entries for poi/draw
- Panel: lists, kind selects, import file/paste, export download blob, clear confirm
- Register `annotations` panel under planning

- [ ] **Step 1: Icons + toolbar**

- [ ] **Step 2: Panel + register**

- [ ] **Step 3: Commit**

```bash
git commit -m "$(cat <<'EOF'
feat(orbisview): Annotations panel and toolbar for POI/draw

List, import/export JSON, and default kind controls.
EOF
)"
```

---

### Task 8: Full verification

- [ ] **Step 1:**

```bash
cd autonomy/orbisview/frontend && npm test && npm run build
```

- [ ] **Step 2: Manual checklist**

1. Place POIs → reload → still present; export/import round-trip
2. Draw polygon + polyline; Esc cancels; 2D↔3D align
3. Toggle layers; measure/nav still work
4. Caps / short commit rejected without wiping data

- [ ] **Step 3: Fixups only if needed**

---

## Spec coverage

| Spec § | Task |
|--------|------|
| JSON / types | T1 |
| Store | T2 |
| MapTool + layers | T3 |
| Map2D draw | T4 |
| View3D | T5 |
| Interaction | T6 |
| UI | T7 |
| Tests + build | T1–T2, T8 |

## Type consistency

- Store: `useAnnotationStore` / `annotationStore.ts`
- Persist: `orbisview-annotations-v1`, `orbisview-layers-v8`
- Tools: `'poi' | 'draw'`
- Layer keys: `poi`, `draw`
- Panel id: `annotations`
