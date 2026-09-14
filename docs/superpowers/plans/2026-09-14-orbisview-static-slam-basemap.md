# OrbisView Static SLAM Basemap (P1) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Load a static SLAM/occupancy image (file / URL / asset) with Nav2 or BICMap-style metadata and draw it as a Map2D/View3D basemap under live map/costmap layers.

**Architecture:** Pure parsers + canvas builder in `renderer/map2d/`; zustand `staticSlamStore` holds the active `StaticSlamBasemap`; `drawScene` blits basemap after world grid and before live occupancy; View3D adds `basemapPlane` below map plane. No MapLibre/Turf/BICMap npm.

**Tech Stack:** Vite/React/TypeScript, Canvas2D, `three`, zustand persist, vitest. **Do not** add `js-yaml` — use a minimal map-server YAML subset parser.

**Spec:** `docs/superpowers/specs/2026-09-14-orbisview-static-slam-basemap-design.md`

## Global Constraints

- No new deps: `@x-humanoid-cloud/bic-map`, `maplibre-gl`, `@turf/turf`, `js-yaml`, `pcl.js`.
- Basemap is **under** live `map` then `costmap`; layers independently toggleable via `LayerKey` `basemap`.
- Static image is painted **as-is** (no −1/0/100 remap).
- Object URLs from file picks must be `revokeObjectURL`'d on replace/clear.
- Persist key `orbisview-static-slam-v1` must **not** restore blob: Object URLs; restore params only for `source==='file'`.
- Layer persist bump to `orbisview-layers-v6` with `basemap: true` default + migrate missing key.
- Working directory for npm: `autonomy/orbisview/frontend`.
- Non-goals: P2 buildMap frames, P3 semantic/floors, P4 POI/draw.

## File map

| File | Responsibility |
|------|----------------|
| `frontend/src/renderer/map2d/staticSlam.ts` | Types helpers, sidecar parse, corners, canvas from ImageBitmap/canvas |
| `frontend/src/renderer/map2d/staticSlam.test.ts` | Parser + corners tests |
| `frontend/src/renderer/map2d/pgmDecode.ts` | P5 PGM → ImageData |
| `frontend/src/renderer/map2d/pgmDecode.test.ts` | Tiny PGM fixture |
| `frontend/src/store/staticSlamStore.ts` | Basemap state + persist + revoke |
| `frontend/src/store/layoutStore.ts` | `basemap` LayerKey + v6 persist |
| `frontend/src/renderer/map2d/types.ts` | `basemap?` on `LayerFlags` |
| `frontend/src/renderer/map2d/drawScene.ts` | Draw basemap blit |
| `frontend/src/renderer/view3d/layers/basemap.ts` | 3D plane update |
| `frontend/src/renderer/view3d/createScene.ts` | `basemapPlane` |
| `frontend/src/renderer/view3d/syncScene.ts` | Sync basemap |
| `frontend/src/renderer/view3d/types.ts` | Input field for basemap |
| `frontend/src/components/Map/StaticSlamLoadPop.tsx` | Form / URL / file UX |
| `frontend/src/components/Map/MapFloatToolbar.tsx` | Entry buttons |
| `frontend/src/components/Map2D/Map2DPanel.tsx` | Wire + fit |
| `frontend/src/components/View3D/View3DPanel.tsx` | Wire |
| `frontend/src/styles/main.css` | Popover styles |

---

### Task 1: Sidecar parse + corners (TDD)

**Files:**
- Create: `autonomy/orbisview/frontend/src/renderer/map2d/staticSlam.ts`
- Create: `autonomy/orbisview/frontend/src/renderer/map2d/staticSlam.test.ts`

**Interfaces:**
- Produces:
  - `export interface StaticSlamMeta { originX: number; originY: number; resolution: number; widthPx?: number; heightPx?: number; image?: string }`
  - `export function parseStaticSlamSidecar(text: string, format: 'yaml' | 'json' | 'auto'): StaticSlamMeta`
  - `export function staticSlamWorldSize(meta: { widthPx: number; heightPx: number; resolution: number }): { worldW: number; worldH: number }`
  - `export function staticSlamCorners(meta: { originX: number; originY: number; widthPx: number; heightPx: number; resolution: number }): { x0: number; y0: number; x1: number; y1: number }`

- [ ] **Step 1: Write failing tests**

```ts
import { describe, expect, it } from 'vitest';
import {
  parseStaticSlamSidecar,
  staticSlamCorners,
  staticSlamWorldSize,
} from './staticSlam';

describe('parseStaticSlamSidecar', () => {
  it('parses Nav2-style YAML', () => {
    const text = `
image: turtlebot3_world.pgm
resolution: 0.050000
origin: [-10.000000, -10.000000, 0.000000]
negate: 0
`;
    const m = parseStaticSlamSidecar(text, 'yaml');
    expect(m.image).toBe('turtlebot3_world.pgm');
    expect(m.resolution).toBeCloseTo(0.05);
    expect(m.originX).toBeCloseTo(-10);
    expect(m.originY).toBeCloseTo(-10);
  });

  it('parses BICMap-style JSON aliases', () => {
    const m = parseStaticSlamSidecar(
      JSON.stringify({
        startX: -58.99,
        startY: -21.35,
        resolution: 0.05,
        xGridCount: 100,
        yGridCount: 50,
        image: 'slam.png',
      }),
      'json',
    );
    expect(m.originX).toBeCloseTo(-58.99);
    expect(m.originY).toBeCloseTo(-21.35);
    expect(m.widthPx).toBe(100);
    expect(m.heightPx).toBe(50);
    expect(m.image).toBe('slam.png');
  });
});

describe('staticSlamCorners', () => {
  it('builds AABB from origin and pixel size', () => {
    expect(
      staticSlamCorners({
        originX: -10,
        originY: -10,
        widthPx: 200,
        heightPx: 200,
        resolution: 0.05,
      }),
    ).toEqual({ x0: -10, y0: -10, x1: 0, y1: 0 });
    expect(staticSlamWorldSize({ widthPx: 200, heightPx: 200, resolution: 0.05 })).toEqual({
      worldW: 10,
      worldH: 10,
    });
  });
});
```

- [ ] **Step 2: Run — expect FAIL**

Run: `cd autonomy/orbisview/frontend && npm test -- src/renderer/map2d/staticSlam.test.ts`

- [ ] **Step 3: Implement parser (minimal YAML subset — no js-yaml)**

```ts
export interface StaticSlamMeta {
  originX: number;
  originY: number;
  resolution: number;
  widthPx?: number;
  heightPx?: number;
  image?: string;
}

function parseOriginList(raw: string): { x: number; y: number } | null {
  const m = raw.match(/\[\s*([-\d.eE+]+)\s*,\s*([-\d.eE+]+)/);
  if (!m) return null;
  return { x: Number(m[1]), y: Number(m[2]) };
}

export function parseStaticSlamSidecar(
  text: string,
  format: 'yaml' | 'json' | 'auto' = 'auto',
): StaticSlamMeta {
  const trimmed = text.trim();
  const asJson =
    format === 'json' || (format === 'auto' && (trimmed.startsWith('{') || trimmed.startsWith('[')));
  if (asJson) {
    const j = JSON.parse(trimmed) as Record<string, unknown>;
    const origin = Array.isArray(j.origin) ? j.origin : null;
    const originX =
      typeof j.startX === 'number'
        ? j.startX
        : origin
          ? Number(origin[0])
          : 0;
    const originY =
      typeof j.startY === 'number'
        ? j.startY
        : origin
          ? Number(origin[1])
          : 0;
    return {
      originX,
      originY,
      resolution: Number(j.resolution ?? 0.05),
      widthPx: typeof j.xGridCount === 'number' ? j.xGridCount : undefined,
      heightPx: typeof j.yGridCount === 'number' ? j.yGridCount : undefined,
      image: typeof j.image === 'string' ? j.image : undefined,
    };
  }
  // YAML subset: key: value lines + origin: [x, y, yaw]
  const lines = trimmed.split(/\r?\n/);
  let image: string | undefined;
  let resolution = 0.05;
  let originX = 0;
  let originY = 0;
  let widthPx: number | undefined;
  let heightPx: number | undefined;
  for (const line of lines) {
    const t = line.trim();
    if (!t || t.startsWith('#')) continue;
    const kv = t.match(/^([A-Za-z0-9_]+)\s*:\s*(.*)$/);
    if (!kv) continue;
    const key = kv[1];
    const val = kv[2].trim();
    if (key === 'image') image = val.replace(/^["']|["']$/g, '');
    else if (key === 'resolution') resolution = Number(val);
    else if (key === 'origin') {
      const o = parseOriginList(val);
      if (o) {
        originX = o.x;
        originY = o.y;
      }
    } else if (key === 'startX') originX = Number(val);
    else if (key === 'startY') originY = Number(val);
    else if (key === 'xGridCount') widthPx = Number(val);
    else if (key === 'yGridCount') heightPx = Number(val);
  }
  return { originX, originY, resolution, widthPx, heightPx, image };
}

export function staticSlamWorldSize(meta: {
  widthPx: number;
  heightPx: number;
  resolution: number;
}): { worldW: number; worldH: number } {
  return {
    worldW: meta.widthPx * meta.resolution,
    worldH: meta.heightPx * meta.resolution,
  };
}

export function staticSlamCorners(meta: {
  originX: number;
  originY: number;
  widthPx: number;
  heightPx: number;
  resolution: number;
}): { x0: number; y0: number; x1: number; y1: number } {
  const { worldW, worldH } = staticSlamWorldSize(meta);
  return {
    x0: meta.originX,
    y0: meta.originY,
    x1: meta.originX + worldW,
    y1: meta.originY + worldH,
  };
}
```

- [ ] **Step 4: Run — expect PASS**

- [ ] **Step 5: Commit**

```bash
git add autonomy/orbisview/frontend/src/renderer/map2d/staticSlam.ts \
        autonomy/orbisview/frontend/src/renderer/map2d/staticSlam.test.ts
git commit -m "$(cat <<'EOF'
feat(orbisview): parse static SLAM sidecar metadata

Support Nav2 YAML origin/resolution and BICMap JSON aliases.
EOF
)"
```

---

### Task 2: PGM decode (TDD)

**Files:**
- Create: `autonomy/orbisview/frontend/src/renderer/map2d/pgmDecode.ts`
- Create: `autonomy/orbisview/frontend/src/renderer/map2d/pgmDecode.test.ts`

**Interfaces:**
- Produces: `export function decodePgm(buffer: ArrayBuffer): ImageData`

Support **P5 binary** and **P2 ASCII** grayscale. Output RGBA (gray → RGB, alpha 255; optionally map pure white to alpha 0 only if needed — **P1: keep opaque** per “paint as-is”).

- [ ] **Step 1: Failing test with tiny P2**

```ts
import { describe, expect, it } from 'vitest';
import { decodePgm } from './pgmDecode';

describe('decodePgm', () => {
  it('decodes ASCII P2 2x1', () => {
    const text = 'P2\n2 1\n255\n0 255\n';
    const img = decodePgm(new TextEncoder().encode(text).buffer);
    expect(img.width).toBe(2);
    expect(img.height).toBe(1);
    expect(img.data[0]).toBe(0);
    expect(img.data[4]).toBe(255);
    expect(img.data[7]).toBe(255);
  });
});
```

- [ ] **Step 2: Implement `decodePgm`** (skip comments starting with `#` after magic)

- [ ] **Step 3: Tests PASS + commit**

```bash
git commit -m "$(cat <<'EOF'
feat(orbisview): decode PGM maps for static basemap

Browser-native Image cannot load PGM; add P2/P5 → ImageData.
EOF
)"
```

---

### Task 3: Canvas builder from image source

**Files:**
- Modify: `autonomy/orbisview/frontend/src/renderer/map2d/staticSlam.ts`

**Interfaces:**
- Produces:
  - `export interface StaticSlamBasemap { imageSrc: string; originX: number; originY: number; resolution: number; widthPx: number; heightPx: number; label?: string; source: 'file' | 'url' | 'asset' }`
  - `export async function loadStaticSlamCanvas(basemap: StaticSlamBasemap, maxEdge = 1024): Promise<{ canvas: HTMLCanvasElement; worldW: number; worldH: number }>`

Behavior:

1. If `imageSrc` ends with `.pgm` / fetch Content-Type suggests pgm → `decodePgm` → putImageData on canvas.
2. Else `createImageBitmap` or `Image()` load → drawImage.
3. Downsample longest edge to `maxEdge` for texture pixels; `worldW/H` still use full `widthPx * resolution` from basemap (caller must set widthPx/heightPx from natural size if missing).

Also export:

```ts
export async function probeImageSize(src: string): Promise<{ w: number; h: number }>
```

- [ ] **Step 1: Implement + a unit test that uses decodePgm path via data URL is optional; at minimum export compiles**

- [ ] **Step 2: `npm run lint` PASS**

- [ ] **Step 3: Commit**

```bash
git commit -m "$(cat <<'EOF'
feat(orbisview): build static SLAM canvas from image or PGM

Downsample texture edge while preserving world meters from meta.
EOF
)"
```

---

### Task 4: `staticSlamStore` + `basemap` layer key

**Files:**
- Create: `autonomy/orbisview/frontend/src/store/staticSlamStore.ts`
- Modify: `autonomy/orbisview/frontend/src/store/layoutStore.ts`
- Modify: `autonomy/orbisview/frontend/src/renderer/map2d/types.ts` (`LayerFlags.basemap?: boolean`)
- Modify: `autonomy/orbisview/frontend/src/schemas.test.ts` if it asserts layers persist name — bump expect to `orbisview-layers-v6`

**Interfaces:**
- Produces store:

```ts
interface StaticSlamState {
  basemap: StaticSlamBasemap | null;
  formDraft: { originX: number; originY: number; resolution: number };
  setBasemap: (b: StaticSlamBasemap | null) => void;
  setFormDraft: (p: Partial<StaticSlamState['formDraft']>) => void;
  clearBasemap: () => void;
}
```

`setBasemap` / `clearBasemap`: if previous `imageSrc` was `blob:` → `URL.revokeObjectURL`.

Persist partializes: never persist `blob:` imageSrc; persist `formDraft` always; persist basemap meta when `source !== 'file'` OR persist meta without imageSrc for file.

Layer store: add `basemap: true`; persist `{ name: 'orbisview-layers-v6', merge: (p,c) => ({ ...c, ...p, basemap: p.basemap ?? true }) }` — use zustand `merge` option.

- [ ] **Step 1: Implement store + layer key**

- [ ] **Step 2: `npm test` (schemas + existing) PASS**

- [ ] **Step 3: Commit**

```bash
git commit -m "$(cat <<'EOF'
feat(orbisview): add staticSlamStore and basemap layer toggle

Persist URL/asset basemaps; revoke blob URLs; bump layers persist v6.
EOF
)"
```

---

### Task 5: Map2D draw basemap + fitView

**Files:**
- Modify: `autonomy/orbisview/frontend/src/renderer/map2d/drawScene.ts`
- Modify: `autonomy/orbisview/frontend/src/components/Map2D/Map2DPanel.tsx`

**Interfaces:**
- Extend `Map2DSceneInput`:

```ts
basemap?: {
  canvas: HTMLCanvasElement;
  originX: number;
  originY: number;
  worldW: number;
  worldH: number;
} | null;
```

Draw after grid, before live map — same Y-flip blit as `drawOccupancy.ts`.

Panel: subscribe to `useStaticSlamStore`; `useEffect` rebuild canvas when basemap changes; pass into `paintMap2DScene`.

`fitView`: if `basemap && paintLayers.basemap` → corners first (spec §9).

- [ ] **Step 1: Implement draw + panel wire + fit**

- [ ] **Step 2: lint PASS**

- [ ] **Step 3: Commit**

```bash
git commit -m "$(cat <<'EOF'
feat(orbisview): render static SLAM basemap under live occupancy (2D)

Fit prefers basemap bounds when the basemap layer is enabled.
EOF
)"
```

---

### Task 6: View3D basemap plane

**Files:**
- Create: `autonomy/orbisview/frontend/src/renderer/view3d/layers/basemap.ts`
- Modify: `createScene.ts`, `syncScene.ts`, `types.ts`, `View3DPanel.tsx`

**Interfaces:**
- `basemapPlane` via `makePlane(0.005)`
- `updateBasemapPlane(plane, handle | null, visible)` — CanvasTexture NearestFilter, geometry from worldW/H, center via `toThree`

Reuse the same canvas handle the panel builds (share via store ref or pass through scene input). Prefer: store keeps `StaticSlamBasemap`; each panel builds/caches canvas locally OR put canvas on a module-level cache keyed by `imageSrc+meta` in `staticSlam.ts`:

```ts
export class StaticSlamCanvasCache {
  get(basemap: StaticSlamBasemap): Promise<…>
  clear(): void
}
```

Use one shared async cache so 2D/3D don’t double-fetch.

- [ ] **Step 1: Add cache + 3D layer + sync**

- [ ] **Step 2: lint + tests PASS**

- [ ] **Step 3: Commit**

```bash
git commit -m "$(cat <<'EOF'
feat(orbisview): show static SLAM basemap plane in View3D

Share canvas cache with Map2D; yLift below live map plane.
EOF
)"
```

---

### Task 7: Load UI (toolbar + popover)

**Files:**
- Create: `autonomy/orbisview/frontend/src/components/Map/StaticSlamLoadPop.tsx`
- Modify: `MapFloatToolbar.tsx`, `main.css`
- Optionally add icon `basemap` in `Icon.tsx` or reuse `map` / `layers`

**UX (spec §7):**

1. Toolbar button「底图」opens popover.
2. Actions: Choose files (multiple), Load URL, Clear, Apply form.
3. On files: partition image vs yaml/json; if sidecar present parse and set form; set basemap with `URL.createObjectURL`.
4. URL: fetch optional sibling `.yaml` by replacing extension; else use formDraft.
5. Asset shortcut (optional): button「示例 turtlebot3」only if a public URL is configured in `parameters.ts` as `STATIC_SLAM_ASSETS?: { label, imageUrl, metaUrl }[]` — if empty array, hide section.

Wire `onBasemapChange` status via `useMapViewStore.setStatusMsg`.

- [ ] **Step 1: Implement popover + toolbar**

- [ ] **Step 2: Manual smoke not required in CI; lint PASS**

- [ ] **Step 3: Commit**

```bash
git commit -m "$(cat <<'EOF'
feat(orbisview): UI to load static SLAM basemap from file or URL

Popover supports sidecar auto-fill, form draft, and clear/revoke.
EOF
)"
```

---

### Task 8: Full verification

- [ ] **Step 1:**

```bash
cd autonomy/orbisview/frontend && npm test && npm run build
```

Expected: all green.

- [ ] **Step 2: Manual checklist (record in PR/chat)**

1. Load `turtlebot3_world.pgm` + yaml from `autonomy/map/conf/`
2. Toggle basemap vs map layers
3. URL PNG
4. Fit frames basemap
5. 2D↔3D alignment; measure still works

- [ ] **Step 3: Fixups commit only if needed**

---

## Spec coverage

| Spec § | Task |
|--------|------|
| Sidecar Nav2 + BICMap | T1 |
| PGM | T2 |
| Canvas / downsample | T3 |
| Store + persist + basemap layer | T4 |
| 2D draw + fit | T5 |
| 3D plane | T6 |
| File/URL/asset UI | T7 |
| Tests + build | T1–T2, T8 |

## Type consistency

- `StaticSlamBasemap` / `StaticSlamMeta` names stable across tasks.
- Layer key string is exactly `basemap`.
- Persist names: `orbisview-static-slam-v1`, `orbisview-layers-v6`.
