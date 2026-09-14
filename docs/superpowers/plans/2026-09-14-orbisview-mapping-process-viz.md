# OrbisView Mapping Process Visualization (P2) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.
>
> **Note:** If subagent billing fails, implement **inline** in the parent session (same task order).

**Goal:** Add demo frame-replay and live Mapping/Occupancy visualization that drives the P1 static basemap canvas, with a Map HUD and a full Mapping panel.

**Architecture:** Pure helpers in `renderer/map2d/mappingDemo.ts` (frame sort + applyFrame); zustand `mappingVizStore` owns demo/live mode and playback state; applying a demo frame or live mirror calls `staticSlamStore.setBasemap` / cache clear so Map2D + View3D already redraw via P1 paths. No MapLibre/BICMap npm; do not inject fake OccupancyGrid into the map channel.

**Tech Stack:** Vite/React/TypeScript, Canvas2D, zustand persist, vitest. Reuse `loadStaticSlamCanvas`, `sharedStaticSlamCanvasCache`, `occupancyTexture` / `fillOccupancyRgba`.

**Spec:** `docs/superpowers/specs/2026-09-14-orbisview-mapping-process-viz-design.md`

## Global Constraints

- No new deps: `@x-humanoid-cloud/bic-map`, `maplibre-gl`, `@turf/turf`, `js-yaml`.
- Demo frames update **basemap** only (P1 path); never write fake Occupancy into live map channel.
- Persist key `orbisview-mapping-viz-v1`: only `mode`, `demo.intervalMs`, `live.mirrorMapToBasemap` — **never** persist blob URL lists.
- Object URLs from demo file picks must be `revokeObjectURL`'d on clear/replace/unmount.
- `mirrorMapToBasemap` default **false**.
- Working directory for npm: `autonomy/orbisview/frontend`.
- Non-goals: P3 semantic/floors, P4 POI/draw/robot GPS trail, Mapping proto changes.

## File map

| File | Responsibility |
|------|----------------|
| `frontend/src/renderer/map2d/mappingDemo.ts` | Sort frame files; `applyDemoFrame` → setBasemap + cache clear |
| `frontend/src/renderer/map2d/mappingDemo.test.ts` | Sort + frame-index helpers |
| `frontend/src/store/mappingVizStore.ts` | Mode, demo playback state, live mirror flag, actions |
| `frontend/src/store/mappingVizStore.test.ts` | nextFrame / reset / play boundary |
| `frontend/src/hooks/useMappingDemoPlayer.ts` | Interval timer that advances frames while `playing` |
| `frontend/src/hooks/useMappingLiveMirror.ts` | When live+mirror: bake map Occupancy → basemap |
| `frontend/src/components/Map/MapMappingHud.tsx` | Compact Map overlay |
| `frontend/src/components/Tasks/MappingPanel.tsx` | Full controls (replace JSON-only MappingPanel) |
| `frontend/src/components/Tasks/TaskPanels.tsx` | Re-export / remove stub MappingPanel |
| `frontend/src/components/index.ts` | Keep `MappingPanel` registration |
| `frontend/src/components/Map2D/Map2DPanel.tsx` | Mount HUD + run player/mirror hooks |
| `frontend/src/components/View3D/View3DPanel.tsx` | Same hooks (or hoist once in Orbisview — prefer **Map2DPanel + View3DPanel both call hooks**; hooks must be idempotent via store) |
| `frontend/src/components/Map/MapFloatToolbar.tsx` | Disable basemap pop while demo `playing` |
| `frontend/src/styles/main.css` | HUD + Mapping panel styles |

**Hoist choice (lock):** Put `useMappingDemoPlayer` and `useMappingLiveMirror` in **`Orbisview.tsx` once** (or a tiny `MappingVizEffects.tsx` mounted under Orbisview) so 2D/3D panels only render HUD and don’t double-fire timers. HUD only on Map2D (primary map viewport); View3D does not need a second HUD.

---

### Task 1: Frame helpers (TDD)

**Files:**
- Create: `autonomy/orbisview/frontend/src/renderer/map2d/mappingDemo.ts`
- Create: `autonomy/orbisview/frontend/src/renderer/map2d/mappingDemo.test.ts`

**Interfaces:**

```ts
/** Natural sort by basename so stage_2.png < stage_10.png */
export function sortMappingFrameUrls(urls: string[]): string[];
export function sortMappingFrameFiles(files: File[]): File[];

export type MappingBasemapMeta = {
  originX: number;
  originY: number;
  resolution: number;
  widthPx: number;
  heightPx: number;
};

/**
 * Build StaticSlamBasemap for one demo frame (source: 'file' | 'url').
 * Does not call store — caller sets via setBasemap.
 */
export function basemapFromDemoFrame(args: {
  imageSrc: string;
  meta: MappingBasemapMeta;
  label?: string;
  source?: 'file' | 'url';
}): import('./staticSlam').StaticSlamBasemap;
```

- [ ] **Step 1: Write failing tests**

```ts
import { describe, expect, it } from 'vitest';
import { sortMappingFrameUrls, basemapFromDemoFrame } from './mappingDemo';

describe('sortMappingFrameUrls', () => {
  it('sorts numeric suffixes naturally', () => {
    expect(
      sortMappingFrameUrls([
        'blob:x/stage_10.png',
        'blob:x/stage_2.png',
        'blob:x/stage_1.png',
      ]),
    ).toEqual([
      'blob:x/stage_1.png',
      'blob:x/stage_2.png',
      'blob:x/stage_10.png',
    ]);
  });
});

describe('basemapFromDemoFrame', () => {
  it('copies meta onto StaticSlamBasemap', () => {
    const b = basemapFromDemoFrame({
      imageSrc: 'blob:abc',
      meta: { originX: -10, originY: -10, resolution: 0.05, widthPx: 200, heightPx: 200 },
      label: 'demo',
    });
    expect(b.originX).toBe(-10);
    expect(b.widthPx).toBe(200);
    expect(b.imageSrc).toBe('blob:abc');
    expect(b.source).toBe('file');
  });
});
```

- [ ] **Step 2: Run test — expect FAIL**

```bash
cd autonomy/orbisview/frontend && npx vitest run src/renderer/map2d/mappingDemo.test.ts
```

- [ ] **Step 3: Implement `mappingDemo.ts`**

Natural sort: extract basename after last `/`, compare with `localeCompare(..., { numeric: true })`.

- [ ] **Step 4: Run test — expect PASS**

- [ ] **Step 5: Commit**

```bash
git commit -m "$(cat <<'EOF'
feat(orbisview): add mapping demo frame sort helpers

Natural-sort stage URLs and build StaticSlamBasemap from demo meta.
EOF
)"
```

---

### Task 2: mappingVizStore (TDD)

**Files:**
- Create: `autonomy/orbisview/frontend/src/store/mappingVizStore.ts`
- Create: `autonomy/orbisview/frontend/src/store/mappingVizStore.test.ts`

**Interfaces:**

```ts
export type MappingVizMode = 'demo' | 'live';

export interface MappingVizState {
  mode: MappingVizMode;
  demo: {
    frameUrls: string[];
    frameIndex: number;
    playing: boolean;
    intervalMs: number;
    basemapMeta: MappingBasemapMeta | null;
  };
  live: { mirrorMapToBasemap: boolean };
  elapsedMs: number;
  setMode: (m: MappingVizMode) => void;
  setIntervalMs: (ms: number) => void;
  setMirrorMapToBasemap: (v: boolean) => void;
  /** Revokes previous blobs, stores sorted urls, resets index; does not setBasemap */
  loadDemoFrames: (urls: string[], meta: MappingBasemapMeta | null) => void;
  clearDemoFrames: () => void;
  play: () => void;
  pause: () => void;
  reset: () => void;
  /** Advance one frame; returns false if already at last or empty */
  nextFrame: () => boolean;
  setFrameIndex: (i: number) => void;
  addElapsed: (dt: number) => void;
}
```

**Behavior (lock):**

- `nextFrame`: if `frameIndex >= frameUrls.length - 1` → `playing=false`, return false; else `frameIndex++`, return true.
- `reset`: `frameIndex=0`, `playing=false`, `elapsedMs=0`.
- `play`: no-op if `frameUrls.length === 0`; else `playing=true`.
- `loadDemoFrames`: revoke old blob urls not in new list; sort via `sortMappingFrameUrls`; set `frameIndex=0`, `playing=false`.
- Persist `partialize`: `{ mode, demo: { intervalMs }, live: { mirrorMapToBasemap } }` only.

- [ ] **Step 1: Write failing tests** (pure store actions; no DOM)

```ts
import { beforeEach, describe, expect, it } from 'vitest';
import { useMappingVizStore } from './mappingVizStore';

beforeEach(() => {
  useMappingVizStore.setState({
    mode: 'demo',
    demo: {
      frameUrls: ['a.png', 'b.png', 'c.png'],
      frameIndex: 0,
      playing: false,
      intervalMs: 500,
      basemapMeta: null,
    },
    live: { mirrorMapToBasemap: false },
    elapsedMs: 0,
  });
});

describe('nextFrame', () => {
  it('advances until last then stops playing', () => {
    const s = useMappingVizStore.getState;
    expect(s().nextFrame()).toBe(true);
    expect(s().demo.frameIndex).toBe(1);
    expect(s().nextFrame()).toBe(true);
    expect(s().demo.frameIndex).toBe(2);
    expect(s().nextFrame()).toBe(false);
    expect(s().demo.playing).toBe(false);
    expect(s().demo.frameIndex).toBe(2);
  });
});

describe('reset', () => {
  it('returns to frame 0', () => {
    useMappingVizStore.getState().setFrameIndex(2);
    useMappingVizStore.getState().play();
    useMappingVizStore.getState().reset();
    expect(useMappingVizStore.getState().demo.frameIndex).toBe(0);
    expect(useMappingVizStore.getState().demo.playing).toBe(false);
  });
});
```

- [ ] **Step 2: Run — expect FAIL**

```bash
cd autonomy/orbisview/frontend && npx vitest run src/store/mappingVizStore.test.ts
```

- [ ] **Step 3: Implement store**

- [ ] **Step 4: Run — expect PASS**

- [ ] **Step 5: Commit**

```bash
git commit -m "$(cat <<'EOF'
feat(orbisview): add mappingVizStore for demo/live mapping viz

Persist mode, interval, and mirror flag only; never blob frame lists.
EOF
)"
```

---

### Task 3: Demo player + apply frame (effects)

**Files:**
- Create: `autonomy/orbisview/frontend/src/hooks/useMappingDemoPlayer.ts`
- Create: `autonomy/orbisview/frontend/src/components/MappingVizEffects.tsx` (mount once from Orbisview)
- Modify: `autonomy/orbisview/frontend/src/components/Orbisview.tsx`

**Behavior:**

1. Subscribe to `demo.playing`, `demo.frameIndex`, `demo.frameUrls`, `demo.intervalMs`, `demo.basemapMeta`, `mode`.
2. When `mode==='demo'` and `frameUrls[frameIndex]` + `basemapMeta` exist → call `setBasemap(basemapFromDemoFrame(...))` (P1 cache clears inside `setBasemap`).
3. While `playing`, `setInterval` every `intervalMs`: `nextFrame()`; if false, clear interval. Also `addElapsed(intervalMs)`.
4. On unmount / mode→live: `pause()`.
5. Guard: if `basemapMeta` null, skip apply (panel must set meta before play).

**Important:** `setBasemap` revokes previous blob if `imageSrc` changes. Demo frames are **different** blob URLs — that is intended. Do **not** revoke URLs still listed in `frameUrls` from the store’s `clearDemoFrames` only. Override: when applying demo frames, call a lighter path if needed:

```ts
// Prefer: add staticSlamStore.replaceBasemapImage(src) that clears cache
// but does NOT revoke if src is still in mappingVizStore.demo.frameUrls.
```

**Lock for P2:** Add `replaceBasemapImage(imageSrc: string, meta?: Partial<StaticSlamBasemap>)` on `staticSlamStore`:

- Updates `basemap.imageSrc` (+ optional width/height probe later);
- `sharedStaticSlamCanvasCache.clear()`;
- **Does not** revoke the previous URL if it starts with `blob:` **and** `useMappingVizStore.getState().demo.frameUrls.includes(prev)` — otherwise revoke as today.

Or simpler lock: **never revoke inside `setBasemap` when caller passes `{ preservePrevBlob: true }`**. Prefer:

```ts
setBasemap: (b, opts?: { revokePrevious?: boolean }) => void
// default revokePrevious = true; demo player passes false
```

Update existing `setBasemap` call sites — default behavior unchanged.

- [ ] **Step 1: Extend `staticSlamStore.setBasemap` with optional `{ revokePrevious?: boolean }`**

- [ ] **Step 2: Implement `useMappingDemoPlayer` + `MappingVizEffects`**

- [ ] **Step 3: Mount `<MappingVizEffects />` in Orbisview**

- [ ] **Step 4: Manual smoke not in CI; `npm test` PASS**

- [ ] **Step 5: Commit**

```bash
git commit -m "$(cat <<'EOF'
feat(orbisview): wire demo frame player to static slam basemap

Interval advances frames; setBasemap can skip revoking demo blobs.
EOF
)"
```

---

### Task 4: Live mirror Occupancy → basemap

**Files:**
- Create: `autonomy/orbisview/frontend/src/hooks/useMappingLiveMirror.ts`
- Create: `autonomy/orbisview/frontend/src/renderer/map2d/mirrorOccupancyBasemap.ts`
- Create: `autonomy/orbisview/frontend/src/renderer/map2d/mirrorOccupancyBasemap.test.ts`
- Modify: `MappingVizEffects.tsx` to call the hook

**Interfaces:**

```ts
/** Paint map-mode occupancy into an offscreen canvas (RGBA), return handle-like canvas */
export function occupancyGridToCanvas(
  grid: OccupancyGridJson,
  maxEdge?: number,
): HTMLCanvasElement;

/**
 * Build StaticSlamBasemap whose imageSrc is a blob: URL of the canvas PNG/bitmap.
 * Caller owns revoke of previous mirror blob (store via mappingViz or staticSlam).
 */
export async function basemapFromOccupancyGrid(
  grid: OccupancyGridJson,
  label?: string,
): Promise<StaticSlamBasemap>;
```

Use `fillOccupancyRgba` + `computeTextureSize` with mode `'map'`. Origin/resolution/width/height from grid. `source: 'url'` or `'file'` with blob URL.

Hook:

- Active when `mode==='live' && live.mirrorMapToBasemap`.
- Resolve map OccupancyGrid the same way Map2D does (`mapDisplayBinding` / envelopes) — **reuse** existing helper if exported; else duplicate minimal lookup from `Channels/mapDisplayBinding.ts`.
- Debounce ~100–200ms; on change → `basemapFromOccupancyGrid` → `setBasemap(..., { revokePrevious: true })` (mirror blobs are ephemeral — revoke OK).
- When mirror turns off: do **not** auto-clear basemap (user may keep last bake).

- [ ] **Step 1: Failing test for `occupancyGridToCanvas` size + known cell color**

- [ ] **Step 2: Implement + PASS**

- [ ] **Step 3: Hook + wire into MappingVizEffects**

- [ ] **Step 4: Commit**

```bash
git commit -m "$(cat <<'EOF'
feat(orbisview): optional live mirror of map occupancy onto basemap

Bakes OccupancyGrid into StaticSlamBasemap when mapping viz mirror is on.
EOF
)"
```

---

### Task 5: MapMappingHud

**Files:**
- Create: `autonomy/orbisview/frontend/src/components/Map/MapMappingHud.tsx`
- Modify: `Map2DPanel.tsx`, `main.css`

**UX:**

- Position: bottom-left of `.map-viewport`, above status if needed; class `map-mapping-hud`.
- Demo: `帧 {i+1}/{n} · 播放中|已暂停|完成` + optional Pause button calling `pause()`.
- Live: read Mapping envelope (`status`, `keyframes`); show `keyframes {n} · {status}`; if mirror on, append `· mirror`.
- Hide when: demo with 0 frames **and** live with no Mapping payload and mirror off — or always show a one-line idle `建图: 演示/实时` when Mapping panel has been used. **Lock:** always show when `mode==='demo' && frameUrls.length>0` OR `mode==='live'` (live always visible as compact status).

- [ ] **Step 1: Implement HUD + mount in Map2DPanel**

- [ ] **Step 2: CSS**

- [ ] **Step 3: Commit**

```bash
git commit -m "$(cat <<'EOF'
feat(orbisview): add Map mapping process HUD

Compact demo progress and live Mapping status on Map2D.
EOF
)"
```

---

### Task 6: MappingPanel full UI

**Files:**
- Create: `autonomy/orbisview/frontend/src/components/Tasks/MappingPanel.tsx`
- Modify: `TaskPanels.tsx` (remove stub; re-export from MappingPanel.tsx)
- Modify: `components/index.ts` if import path changes
- Modify: `main.css`

**UX sections:**

1. Mode toggle: 演示回放 | 实时建图 → `setMode`
2. **Demo:**
   - Multi-file input (images); optional sidecar yaml/json (reuse `parseStaticSlamSidecar` + formDraft like StaticSlamLoadPop)
   - On pick: `URL.createObjectURL` each → `loadDemoFrames(sorted, meta)`; if meta null use `formDraft` + probe first image size (`createImageBitmap` / Image)
   - Interval number input → `setIntervalMs`
   - Play / Pause / Reset
   - `<input type="range">` bound to `frameIndex` → `setFrameIndex`
   - Optional JSON textarea: paste URL array → `loadDemoFrames`
3. **Live:**
   - Checkbox `mirrorMapToBasemap`
   - Mapping payload `<pre>` (keep current TaskPanel behavior)
   - Hint: 可关 Channels map 层避免与 mirror 双重绘制
4. While demo `playing`, show hint that 静态底图工具栏 is disabled (Task 7)

Reuse patterns from `StaticSlamLoadPop.tsx` for sidecar / form fields (origin, resolution).

- [ ] **Step 1: Implement MappingPanel**

- [ ] **Step 2: Wire exports**

- [ ] **Step 3: Commit**

```bash
git commit -m "$(cat <<'EOF'
feat(orbisview): Mapping panel for demo replay and live controls

Replaces JSON-only Mapping TaskPanel with mode, playback, and mirror UI.
EOF
)"
```

---

### Task 7: Toolbar guard + polish

**Files:**
- Modify: `MapFloatToolbar.tsx` — disable「静态底图」or show title when `demo.playing`
- Modify: `StaticSlamLoadPop.tsx` — early return banner if playing (optional belt-and-suspenders)

- [ ] **Step 1: Disable basemap open while playing**

- [ ] **Step 2: Commit**

```bash
git commit -m "$(cat <<'EOF'
fix(orbisview): block static basemap load while mapping demo plays

Avoid fighting the demo player for basemap imageSrc ownership.
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

- [ ] **Step 2: Manual checklist**

1. Mapping panel → demo → pick 5+ PNG/PGM (+ yaml) → Play → basemap advances; Pause/Reset OK; HUD updates.
2. Live mode + mock `/orbisview/mock/mapping` → HUD keyframes/status; panel JSON updates.
3. Enable mirror → map Occupancy changes update basemap; disable map layer → baked basemap still visible.
4. 2D↔3D basemap stay aligned; measure/nav tools unaffected.
5. Reload page: mode/interval/mirror restore; demo frames **not** restored (expected).

- [ ] **Step 3: Fixups commit only if needed**

---

## Spec coverage

| Spec § | Task |
|--------|------|
| Demo frame sort / apply | T1, T3 |
| mappingVizStore | T2 |
| Demo timer → basemap | T3 |
| Live Mapping HUD + mirror | T4, T5 |
| MappingPanel UI | T6 |
| Basemap toolbar conflict | T7 |
| Tests + build | T1–T2, T4, T8 |

## Type consistency

- Store name: `useMappingVizStore` / file `mappingVizStore.ts`
- Persist: `orbisview-mapping-viz-v1`
- Modes: `'demo' | 'live'` exactly
- `setBasemap(b, opts?: { revokePrevious?: boolean })` — default revoke true
- Effects mounted once via `MappingVizEffects` under Orbisview; HUD only on Map2D
