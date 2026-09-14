# OrbisView Indoor Occupancy Display Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace Map2D per-cell OccupancyGrid painting with a shared BICMap-style canvas texture pipeline (2D `drawImage` + 3D `CanvasTexture`), unify palette, improve fit-to-map, and lightly restyle the map viewport — without MapLibre/Turf/BICMap npm.

**Architecture:** Pure `occupancyTexture.ts` builds RGBA buffers + optional cached `HTMLCanvasElement` keyed by grid identity; `drawOccupancy.ts` and `view3d/layers/occupancy.ts` both consume it. Fit prefers map/costmap corners. CSS adds a faint grid under the canvas.

**Tech Stack:** Vite / React / TypeScript, Canvas2D, `three` ^0.170, vitest.

**Spec:** `docs/superpowers/specs/2026-09-14-orbisview-indoor-occupancy-display-design.md`

## Global Constraints

- Do **not** add dependencies: `@x-humanoid-cloud/bic-map`, `maplibre-gl`, `@turf/turf`, `pcl.js`, `urdf-loader`.
- Do **not** change OccupancyGrid wire JSON or backend mock schema.
- Do **not** add static PNG/BMP SLAM load UI or buildMap frame playback.
- Keep measure / nav / pick / Channels layer toggles behavior unchanged.
- Texture longest edge ≤ **1024** (NearestFilter); map vs costmap each have their own cache entry.
- Palette for 2D and 3D **must** come from the same `occupancyCellRgba` function.
- Working directory for npm: `autonomy/orbisview/frontend`.

## File map

| File | Responsibility |
|------|----------------|
| `frontend/src/renderer/map2d/occupancyTexture.ts` | Palette, size, RGBA fill, texture cache |
| `frontend/src/renderer/map2d/occupancyTexture.test.ts` | Unit tests (pure RGBA + size + cache key) |
| `frontend/src/renderer/map2d/drawOccupancy.ts` | 2D `drawImage` of cached canvas into world rect |
| `frontend/src/renderer/view3d/layers/occupancy.ts` | 3D plane uses shared builder |
| `frontend/src/renderer/index.ts` | Re-export public helpers if useful |
| `frontend/src/components/Map2D/Map2DPanel.tsx` | `fitView` prefers map then costmap corners |
| `frontend/src/styles/main.css` | Map viewport base + faint CSS grid |

---

### Task 1: occupancyTexture palette + size + RGBA fill (TDD)

**Files:**
- Create: `autonomy/orbisview/frontend/src/renderer/map2d/occupancyTexture.ts`
- Create: `autonomy/orbisview/frontend/src/renderer/map2d/occupancyTexture.test.ts`

**Interfaces:**
- Produces:
  - `export type OccupancyPaintMode = 'map' | 'costmap'`
  - `export const OCCUPANCY_MAX_EDGE = 1024`
  - `export function occupancyCellRgba(v: number, mode: OccupancyPaintMode): [number, number, number, number]`
  - `export function computeTextureSize(width: number, height: number, maxEdge?: number): { tw: number; th: number; scale: number }`
  - `export function fillOccupancyRgba(out: Uint8ClampedArray, grid: OccupancyGridJson, mode: OccupancyPaintMode, tw: number, th: number, scale: number): void`

- [ ] **Step 1: Write failing tests**

```ts
import { describe, expect, it } from 'vitest';
import {
  computeTextureSize,
  fillOccupancyRgba,
  occupancyCellRgba,
  OCCUPANCY_MAX_EDGE,
} from './occupancyTexture';
import type { OccupancyGridJson } from './types';

describe('occupancyCellRgba map', () => {
  it('unknown is translucent gray', () => {
    const [r, g, b, a] = occupancyCellRgba(-1, 'map');
    expect(a).toBeGreaterThan(0);
    expect(a).toBeLessThan(255);
    expect(Math.abs(r - g)).toBeLessThan(20);
  });

  it('free is fully transparent', () => {
    expect(occupancyCellRgba(0, 'map')[3]).toBe(0);
  });

  it('occupied is dark opaque-ish', () => {
    const [r, g, b, a] = occupancyCellRgba(100, 'map');
    expect(a).toBeGreaterThan(180);
    expect(r).toBeLessThan(80);
    expect(g).toBeLessThan(80);
    expect(b).toBeLessThan(80);
  });
});

describe('occupancyCellRgba costmap', () => {
  it('skips unknown (alpha 0)', () => {
    expect(occupancyCellRgba(-1, 'costmap')[3]).toBe(0);
  });

  it('lethal is warm', () => {
    const [r, g, , a] = occupancyCellRgba(100, 'costmap');
    expect(a).toBeGreaterThan(80);
    expect(r).toBeGreaterThan(g);
  });
});

describe('computeTextureSize', () => {
  it('keeps small grids 1:1', () => {
    expect(computeTextureSize(10, 20)).toEqual({ tw: 10, th: 20, scale: 1 });
  });

  it('downsamples when over max edge', () => {
    const { tw, th, scale } = computeTextureSize(2048, 1024, OCCUPANCY_MAX_EDGE);
    expect(Math.max(tw, th)).toBeLessThanOrEqual(OCCUPANCY_MAX_EDGE);
    expect(scale).toBeLessThan(1);
  });
});

describe('fillOccupancyRgba', () => {
  it('writes free cell as transparent', () => {
    const grid: OccupancyGridJson = {
      resolution: 0.05,
      width: 2,
      height: 1,
      origin: { x: 0, y: 0 },
      data: [0, 100],
    };
    const out = new Uint8ClampedArray(2 * 1 * 4);
    fillOccupancyRgba(out, grid, 'map', 2, 1, 1);
    expect(out[3]).toBe(0);
    expect(out[7]).toBeGreaterThan(180);
  });
});
```

- [ ] **Step 2: Run tests — expect FAIL**

Run: `cd autonomy/orbisview/frontend && npm test -- src/renderer/map2d/occupancyTexture.test.ts`

Expected: FAIL (module not found / export missing).

- [ ] **Step 3: Implement `occupancyTexture.ts` (palette + size + fill only)**

```ts
import type { OccupancyGridJson } from './types';

export type OccupancyPaintMode = 'map' | 'costmap';
export const OCCUPANCY_MAX_EDGE = 1024;

export function occupancyCellRgba(
  v: number,
  mode: OccupancyPaintMode,
): [number, number, number, number] {
  if (mode === 'costmap') {
    if (v < 0) return [0, 0, 0, 0];
    if (v >= 100) return [255, 112, 67, 140];
    if (v === 0) return [0, 0, 0, 0];
    return [255, 167, 38, 90];
  }
  // map
  if (v < 0) return [160, 168, 176, 110];
  if (v === 0) return [0, 0, 0, 0];
  if (v >= 100) return [40, 44, 52, 230];
  const t = v / 100;
  return [
    Math.round(40 * t),
    Math.round(44 * t),
    Math.round(52 * t),
    Math.round(40 + 190 * t),
  ];
}

export function computeTextureSize(
  width: number,
  height: number,
  maxEdge = OCCUPANCY_MAX_EDGE,
): { tw: number; th: number; scale: number } {
  if (width <= 0 || height <= 0) return { tw: 1, th: 1, scale: 1 };
  const scale = Math.min(1, maxEdge / Math.max(width, height));
  return {
    tw: Math.max(1, Math.round(width * scale)),
    th: Math.max(1, Math.round(height * scale)),
    scale,
  };
}

export function fillOccupancyRgba(
  out: Uint8ClampedArray,
  grid: OccupancyGridJson,
  mode: OccupancyPaintMode,
  tw: number,
  th: number,
  scale: number,
): void {
  const { width, height, data } = grid;
  for (let j = 0; j < th; j++) {
    for (let i = 0; i < tw; i++) {
      const sx = Math.min(width - 1, Math.floor(i / scale));
      // Image row 0 = grid y = height-1 if we flip; match existing 3D: j maps with floor(j/scale) as sy
      const sy = Math.min(height - 1, Math.floor(j / scale));
      const v = data[sy * width + sx] ?? -1;
      const [r, g, b, a] = occupancyCellRgba(v, mode);
      const o = (j * tw + i) * 4;
      out[o] = r;
      out[o + 1] = g;
      out[o + 2] = b;
      out[o + 3] = a;
    }
  }
}
```

Note: keep Y orientation consistent with current `view3d/layers/occupancy.ts` (sy = floor(j/scale)). Do not invent a flip unless 2D drawImage placement already accounted for screen Y.

- [ ] **Step 4: Run tests — expect PASS**

Run: `cd autonomy/orbisview/frontend && npm test -- src/renderer/map2d/occupancyTexture.test.ts`

Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add autonomy/orbisview/frontend/src/renderer/map2d/occupancyTexture.ts \
        autonomy/orbisview/frontend/src/renderer/map2d/occupancyTexture.test.ts
git commit -m "$(cat <<'EOF'
feat(orbisview): add shared occupancy RGBA palette and fill

BICMap-aligned map/costmap colors and downsampling helpers for texture blit.
EOF
)"
```

---

### Task 2: OccupancyTextureCache

**Files:**
- Modify: `autonomy/orbisview/frontend/src/renderer/map2d/occupancyTexture.ts`
- Modify: `autonomy/orbisview/frontend/src/renderer/map2d/occupancyTexture.test.ts`

**Interfaces:**
- Consumes: `fillOccupancyRgba`, `computeTextureSize`, `OccupancyPaintMode`
- Produces:
  - `export interface OccupancyTextureHandle { canvas: HTMLCanvasElement; texW: number; texH: number; worldW: number; worldH: number; originX: number; originY: number }`
  - `export class OccupancyTextureCache { get(grid: OccupancyGridJson, mode: OccupancyPaintMode): OccupancyTextureHandle | null; clear(): void }`
  - Module-level helpers for tests that avoid DOM:
    - `export function occupancyCacheKey(grid: OccupancyGridJson, mode: OccupancyPaintMode): string`

Cache key must include: `mode`, `width`, `height`, `resolution`, `origin.x`, `origin.y`, and **data array identity** (`data` reference). When the same object reference is passed, `get` returns the same handle without rebuilding.

If `document` / `HTMLCanvasElement` is unavailable (Node vitest), `get` may return `null` after still validating key logic — but prefer testing `occupancyCacheKey` + a package-private rebuild path. Practical approach for vitest without canvas:

```ts
export function occupancyCacheKey(grid: OccupancyGridJson, mode: OccupancyPaintMode): string {
  return [
    mode,
    grid.width,
    grid.height,
    grid.resolution,
    grid.origin?.x ?? 0,
    grid.origin?.y ?? 0,
    // Weak identity: length + first/mid/last samples to catch in-place mutations of same array
    grid.data?.length ?? 0,
    grid.data?.[0] ?? '',
    grid.data?.[Math.floor((grid.data?.length ?? 1) / 2)] ?? '',
    grid.data?.[(grid.data?.length ?? 1) - 1] ?? '',
  ].join('|');
}
```

Spec allows dataRef **or** generation/fingerprint — use the fingerprint above so in-place mock updates still refresh.

- [ ] **Step 1: Extend tests for `occupancyCacheKey`**

```ts
describe('occupancyCacheKey', () => {
  const base = (): OccupancyGridJson => ({
    resolution: 0.05,
    width: 2,
    height: 1,
    origin: { x: 1, y: 2 },
    data: [0, 100],
  });

  it('changes when mode changes', () => {
    const g = base();
    expect(occupancyCacheKey(g, 'map')).not.toBe(occupancyCacheKey(g, 'costmap'));
  });

  it('changes when a cell value changes', () => {
    const g = base();
    const a = occupancyCacheKey(g, 'map');
    g.data[1] = 50;
    expect(occupancyCacheKey(g, 'map')).not.toBe(a);
  });
});
```

- [ ] **Step 2: Run — expect FAIL** then implement `occupancyCacheKey` + `OccupancyTextureCache`

```ts
export interface OccupancyTextureHandle {
  canvas: HTMLCanvasElement;
  texW: number;
  texH: number;
  worldW: number;
  worldH: number;
  originX: number;
  originY: number;
}

export class OccupancyTextureCache {
  private key = '';
  private handle: OccupancyTextureHandle | null = null;

  get(grid: OccupancyGridJson | null | undefined, mode: OccupancyPaintMode): OccupancyTextureHandle | null {
    if (!grid || grid.width <= 0 || grid.height <= 0 || !(grid.resolution > 0)) {
      return null;
    }
    const next = occupancyCacheKey(grid, mode);
    if (this.handle && this.key === next) return this.handle;

    const { tw, th, scale } = computeTextureSize(grid.width, grid.height);
    const canvas = document.createElement('canvas');
    canvas.width = tw;
    canvas.height = th;
    const ctx = canvas.getContext('2d');
    if (!ctx) return null;
    const img = ctx.createImageData(tw, th);
    fillOccupancyRgba(img.data, grid, mode, tw, th, scale);
    ctx.putImageData(img, 0, 0);

    const originX = grid.origin?.x ?? 0;
    const originY = grid.origin?.y ?? 0;
    this.handle = {
      canvas,
      texW: tw,
      texH: th,
      worldW: grid.width * grid.resolution,
      worldH: grid.height * grid.resolution,
      originX,
      originY,
    };
    this.key = next;
    return this.handle;
  }

  clear(): void {
    this.key = '';
    this.handle = null;
  }
}
```

Use **two** cache instances in callers (mapCache, costmapCache) — one class instance per mode stream — so keys need not collide across modes in one object (mode is still in the key).

- [ ] **Step 3: Run key tests PASS**

Run: `cd autonomy/orbisview/frontend && npm test -- src/renderer/map2d/occupancyTexture.test.ts`

- [ ] **Step 4: Commit**

```bash
git add autonomy/orbisview/frontend/src/renderer/map2d/occupancyTexture.ts \
        autonomy/orbisview/frontend/src/renderer/map2d/occupancyTexture.test.ts
git commit -m "$(cat <<'EOF'
feat(orbisview): cache occupancy canvas textures by grid fingerprint

Rebuild ImageData only when map/costmap content or geometry changes.
EOF
)"
```

---

### Task 3: Map2D drawOccupancy uses texture blit

**Files:**
- Modify: `autonomy/orbisview/frontend/src/renderer/map2d/drawOccupancy.ts`
- Modify: `autonomy/orbisview/frontend/src/renderer/map2d/drawScene.ts` (pass caches or use module singletons)

**Interfaces:**
- Consumes: `OccupancyTextureCache`, `OccupancyPaintMode`
- Produces: updated `drawOccupancyGrid(ctx, grid, toScreen, scale, mode, cache?)`

Preferred signature (backward compatible default caches):

```ts
import type { OccupancyGridJson } from './types';
import type { WorldToScreen } from './coords';
import { OccupancyTextureCache, type OccupancyPaintMode } from './occupancyTexture';

const defaultMapCache = new OccupancyTextureCache();
const defaultCostmapCache = new OccupancyTextureCache();

export function drawOccupancyGrid(
  ctx: CanvasRenderingContext2D,
  grid: OccupancyGridJson,
  toScreen: WorldToScreen,
  _scale: number,
  mode: OccupancyPaintMode,
  cache?: OccupancyTextureCache,
): void {
  const c =
    cache ?? (mode === 'costmap' ? defaultCostmapCache : defaultMapCache);
  const tex = c.get(grid, mode);
  if (!tex) return;

  const x0 = tex.originX;
  const y0 = tex.originY;
  const x1 = x0 + tex.worldW;
  const y1 = y0 + tex.worldH;
  // Corners in screen space: bottom-left and top-right depend on Y-up world
  const [sx0, sy0] = toScreen(x0, y0);
  const [sx1, sy1] = toScreen(x1, y1);
  const left = Math.min(sx0, sx1);
  const top = Math.min(sy0, sy1);
  const w = Math.abs(sx1 - sx0);
  const h = Math.abs(sy1 - sy0);
  ctx.imageSmoothingEnabled = false;
  ctx.drawImage(tex.canvas, left, top, w, h);
}
```

Remove the old per-cell `fillRect` loop entirely.

`drawScene.ts` can keep calling `drawOccupancyGrid(ctx, map, toScreen, scale, 'map')` unchanged.

- [ ] **Step 1: Replace `drawOccupancy.ts` body as above**

- [ ] **Step 2: Typecheck**

Run: `cd autonomy/orbisview/frontend && npm run lint`

Expected: no errors.

- [ ] **Step 3: Commit**

```bash
git add autonomy/orbisview/frontend/src/renderer/map2d/drawOccupancy.ts
git commit -m "$(cat <<'EOF'
feat(orbisview): blit occupancy textures on Map2D instead of per-cell fills

Aligns indoor map display with BICMap canvas-layer approach on Cartesian grids.
EOF
)"
```

---

### Task 4: View3D occupancy uses shared builder

**Files:**
- Modify: `autonomy/orbisview/frontend/src/renderer/view3d/layers/occupancy.ts`

**Interfaces:**
- Consumes: `OccupancyTextureCache` (or `fillOccupancyRgba` + `computeTextureSize`)
- Produces: same `updateOccupancyPlane(plane, grid, opts)` signature as today

Replace local `cellColor` / MAX_EDGE=512 with:

```ts
import { OccupancyTextureCache } from '../../map2d/occupancyTexture';

const mapCache = new OccupancyTextureCache();
const costmapCache = new OccupancyTextureCache();

export function updateOccupancyPlane(
  plane: THREE.Mesh,
  grid: OccupancyGridJson | null,
  opts: { opacity: number; yLift: number; visible: boolean; mode?: 'map' | 'costmap' },
): void {
  const mode = opts.mode ?? 'map';
  plane.visible = opts.visible && !!grid;
  if (!opts.visible || !grid) return;

  const cache = mode === 'costmap' ? costmapCache : mapCache;
  const tex = cache.get(grid, mode);
  if (!tex) {
    plane.visible = false;
    return;
  }

  const prev = plane.material as THREE.MeshBasicMaterial;
  if (prev.map) prev.map.dispose();
  const texture = new THREE.CanvasTexture(tex.canvas);
  texture.magFilter = THREE.NearestFilter;
  texture.minFilter = THREE.NearestFilter;
  texture.needsUpdate = true;
  prev.map = texture;
  prev.opacity = opts.opacity;
  prev.transparent = true;
  prev.needsUpdate = true;

  plane.geometry.dispose();
  plane.geometry = new THREE.PlaneGeometry(tex.worldW, tex.worldH);
  plane.rotation.x = -Math.PI / 2;
  const center = toThree(
    tex.originX + tex.worldW / 2,
    tex.originY + tex.worldH / 2,
    opts.yLift,
  );
  plane.position.set(center.x, center.y, center.z);
}
```

Update `syncScene.ts` to pass `mode` (keep existing opacity fields exactly):

```ts
updateOccupancyPlane(ctx.mapPlane, input.map, {
  opacity: opts.mapOpacity,
  yLift: 0.01,
  visible: layers.map,
  mode: 'map',
});
updateOccupancyPlane(ctx.costmapPlane, input.costmap, {
  opacity: opts.mapOpacity * 0.85,
  yLift: 0.02,
  visible: layers.costmap,
  mode: 'costmap',
});
```

- [ ] **Step 1: Read current `syncScene.ts` occupancy calls and align field names**

- [ ] **Step 2: Implement occupancy.ts + syncScene mode passthrough**

- [ ] **Step 3: Lint + unit tests**

Run:

```bash
cd autonomy/orbisview/frontend && npm run lint && npm test
```

Expected: PASS.

- [ ] **Step 4: Commit**

```bash
git add autonomy/orbisview/frontend/src/renderer/view3d/layers/occupancy.ts \
        autonomy/orbisview/frontend/src/renderer/view3d/syncScene.ts
git commit -m "$(cat <<'EOF'
feat(orbisview): share occupancy texture builder with View3D planes

Unifies 2D/3D indoor map colors and upscales max texture edge to 1024.
EOF
)"
```

---

### Task 5: fitView prefers map / costmap corners

**Files:**
- Modify: `autonomy/orbisview/frontend/src/components/Map2D/Map2DPanel.tsx` (`fitView`, ~767–807)

**Interfaces:**
- Consumes: existing `map`, `costmap`, `layers` / `paintLayers` from panel
- Produces: unchanged `fitView` callback wired to toolbar

Logic (spec §9):

1. If map exists and map layer effectively on → use only map corners `(ox,oy)` and `(ox+W, oy+H)`.
2. Else if costmap exists and costmap layer on → use costmap corners.
3. Else fall back to current pose/goal/waypoints/(optional map) union.

Use `effectiveMapLayers` / whatever the panel already uses for `layers.map` visibility — read the paint path variable (often `paintLayers` or `layers`). Prefer the same flags used when drawing.

Padding: keep `pad = 1.2` (spec allows ~1.08–1.2).

```ts
const fitView = () => {
  const canvas = canvasRef.current;
  const w = canvas?.width ?? canvasSize.w;
  const h = canvas?.height ?? canvasSize.h;
  const pts: { x: number; y: number }[] = [];

  const pushGrid = (g: OccupancyGridJson) => {
    const ox = g.origin?.x ?? 0;
    const oy = g.origin?.y ?? 0;
    const res = g.resolution ?? 0.05;
    pts.push({ x: ox, y: oy });
    pts.push({ x: ox + g.width * res, y: oy + g.height * res });
  };

  // paintLayers / layers — use the same object drawScene receives
  if (map && paintLayers.map) {
    pushGrid(map);
  } else if (costmap && paintLayers.costmap) {
    pushGrid(costmap);
  } else {
    if (pose) pts.push(pose);
    if (goal) pts.push(goal);
    waypoints.forEach((wp) => pts.push(wp));
    if (map) pushGrid(map);
  }

  // ... existing AABB → scale / viewOffset ...
  setStatusMsg(`自适应 · scale ${nextScale.toFixed(0)}`);
};
```

- [ ] **Step 1: Patch `fitView` as above (resolve `paintLayers` name from file)**

- [ ] **Step 2: Lint**

Run: `cd autonomy/orbisview/frontend && npm run lint`

- [ ] **Step 3: Commit**

```bash
git add autonomy/orbisview/frontend/src/components/Map2D/Map2DPanel.tsx
git commit -m "$(cat <<'EOF'
fix(orbisview): fit Map2D view to occupancy bounds like BICMap

Prefer map then costmap corners before pose/waypoint union.
EOF
)"
```

---

### Task 6: Map viewport CSS (faint grid + slightly lifted base)

**Files:**
- Modify: `autonomy/orbisview/frontend/src/styles/main.css` (`.map-viewport` / `.map-canvas-fill`)

**Interfaces:** none (visual only)

- [ ] **Step 1: Add viewport styling**

Near `.map-viewport` / `.map-canvas-fill`:

```css
.map-viewport.map-primary,
.map-viewport.view3d-panel {
  background-color: #121820;
  background-image:
    linear-gradient(rgba(110, 198, 234, 0.04) 1px, transparent 1px),
    linear-gradient(90deg, rgba(110, 198, 234, 0.04) 1px, transparent 1px);
  background-size: 40px 40px;
}
.map-canvas-fill {
  background: transparent; /* let viewport grid show through free cells */
}
```

Also lighten Map2D clear color in `drawScene.ts` from `#0f1419` to `#141b22` so free/transparent cells show the CSS grid faintly:

```ts
ctx.fillStyle = '#141b22';
ctx.fillRect(0, 0, w, h);
```

(World `layers.grid` strokes stay as-is.)

- [ ] **Step 2: Commit**

```bash
git add autonomy/orbisview/frontend/src/styles/main.css \
        autonomy/orbisview/frontend/src/renderer/map2d/drawScene.ts
git commit -m "$(cat <<'EOF'
style(orbisview): lighten map viewport with BICMap-like faint grid

Transparent free cells reveal the underlay; keep OrbisView dark shell.
EOF
)"
```

---

### Task 7: Full verification + acceptance checklist

**Files:** none required (docs optional one-liner in frontend README — skip unless already documenting Map2D).

- [ ] **Step 1: Run full frontend checks**

```bash
cd autonomy/orbisview/frontend && npm test && npm run build
```

Expected: all tests green; `tsc --noEmit` + vite build OK.

- [ ] **Step 2: Manual mock checklist (executor records results in PR/chat)**

1. Ground layout + mock map: unknown gray, free transparent, walls dark — not red blocks.
2. Pan/zoom large map feels smooth (no per-cell stutter).
3. Costmap overlay warm translucent on top of map.
4. Fit button frames the occupancy rectangle.
5. 2D↔3D colors match; measure/nav still work.

- [ ] **Step 3: Final commit only if tiny fixups were needed; otherwise stop**

---

## Spec coverage self-check

| Spec item | Task |
|-----------|------|
| Shared RGBA builder 2D/3D | T1–T4 |
| Map2D `drawImage` | T3 |
| View3D shared builder | T4 |
| Fit map/costmap corners | T5 |
| Viewport faint grid + lift | T6 |
| No MapLibre/BICMap deps | Global + all tasks |
| Unit tests + npm test/build | T1, T2, T7 |
| Non-goals (static PNG, buildMap, …) | Not scheduled |

## Placeholder / type consistency self-check

- `OccupancyPaintMode`, `OccupancyTextureCache`, `occupancyCellRgba`, `fillOccupancyRgba`, `computeTextureSize`, `occupancyCacheKey`, `OccupancyTextureHandle` names are stable across tasks.
- `drawOccupancyGrid` keeps `(ctx, grid, toScreen, scale, mode)` call sites in `drawScene.ts`.
- `updateOccupancyPlane` gains optional `mode`; `syncScene` must pass it.
