import { create } from 'zustand';
import { persist } from 'zustand/middleware';
import type { MosaicNode } from 'react-mosaic-component';

export type PanelId = string;
export type SidebarTab = 'panels' | 'channels' | 'task' | 'setting';

export const GROUND_ROBOT_LAYOUT: MosaicNode<string> = {
  type: 'split',
  direction: 'row',
  children: [
    'map',
    {
      type: 'split',
      direction: 'column',
      children: [
        'dashboard',
        {
          type: 'split',
          direction: 'column',
          children: ['robot_status', 'waypoints'],
          splitPercentages: [50, 50],
        },
      ],
      splitPercentages: [35, 65],
    },
  ],
  splitPercentages: [68, 32],
};

/** Minimal default: Map only. */
export const DEFAULT_LAYOUT: MosaicNode<string> = 'map';

/** Recommended ops layout (alias of ground robot preset). */
export const RECOMMENDED_LAYOUT: MosaicNode<string> = GROUND_ROBOT_LAYOUT;

export type LayoutPresetId = 'default' | 'recommended';

export const LAYOUT_PRESETS: Record<
  LayoutPresetId,
  { id: LayoutPresetId; label: string; hint: string; node: MosaicNode<string> }
> = {
  default: {
    id: 'default',
    label: '默认布局',
    hint: '仅 Map 主视口',
    node: DEFAULT_LAYOUT,
  },
  recommended: {
    id: 'recommended',
    label: '推荐布局',
    hint: 'Map + Dashboard + Status + Waypoints',
    node: RECOMMENDED_LAYOUT,
  },
};

/** Collect mosaic leaf panel ids (depth-first). */
export function collectMosaicIds(node: MosaicNode<string> | null): string[] {
  if (node == null) return [];
  if (typeof node === 'string') return [node];
  if ('children' in node && Array.isArray(node.children)) {
    return node.children.flatMap((c) => collectMosaicIds(c as MosaicNode<string>));
  }
  return [];
}

/** Rewrite legacy map2d / view3d leaves to unified `map`. */
export function migrateMapLeaves(
  node: MosaicNode<string>,
): MosaicNode<string> {
  if (typeof node === 'string') {
    if (node === 'map2d' || node === 'view3d') return 'map';
    return node;
  }
  if (!('children' in node) || !Array.isArray(node.children)) return node;
  return {
    ...node,
    children: node.children.map((c) =>
      migrateMapLeaves(c as MosaicNode<string>),
    ) as typeof node.children,
  };
}

/** Drop corrupt trees (duplicate leaf ids crash react-mosaic). */
export function sanitizeMosaic(
  node: MosaicNode<string> | null,
): MosaicNode<string> | null {
  if (!node) return null;
  const migrated = migrateMapLeaves(node);
  const ids = collectMosaicIds(migrated);
  if (ids.length === 0) return GROUND_ROBOT_LAYOUT;
  if (new Set(ids).size !== ids.length) return GROUND_ROBOT_LAYOUT;
  return migrated;
}

function equalSplitPercents(n: number): number[] {
  if (n <= 0) return [];
  const base = Math.floor((10000 / n)) / 100;
  const out = Array.from({ length: n }, () => base);
  const sum = out.reduce((a, b) => a + b, 0);
  out[out.length - 1] = Math.round((out[out.length - 1] + (100 - sum)) * 100) / 100;
  return out;
}

/**
 * Insert a leaf into the mosaic for tiling: prefer appending into an existing
 * same-direction split (n-ary grid); otherwise create a balanced 50/50 split.
 */
export function insertMosaicLeaf(
  cur: MosaicNode<string> | null,
  id: string,
  direction: 'row' | 'column' = 'row',
): MosaicNode<string> {
  if (!cur) return id;
  if (typeof cur !== 'string' && cur.type === 'split' && cur.direction === direction) {
    const children = [...cur.children, id];
    return {
      ...cur,
      children,
      splitPercentages: equalSplitPercents(children.length),
    };
  }
  return {
    type: 'split',
    direction,
    children: [cur, id],
    splitPercentages: [50, 50],
  };
}

export interface LayoutState {
  mosaic: MosaicNode<string> | null;
  catalogOpen: boolean;
  sidebarTab: SidebarTab;
  setMosaic: (node: MosaicNode<string> | null) => void;
  setCatalogOpen: (v: boolean) => void;
  setSidebarTab: (t: SidebarTab) => void;
  resetGroundPreset: () => void;
  applyLayoutPreset: (id: LayoutPresetId) => void;
  exportLayoutJson: () => string;
  importLayoutJson: (raw: string) => boolean;
  /** Open a single-instance panel if missing from the mosaic. */
  ensurePanel: (id: string) => void;
}

export const useLayoutStore = create<LayoutState>()(
  persist(
    (set, get) => ({
      mosaic: GROUND_ROBOT_LAYOUT,
      catalogOpen: true,
      sidebarTab: 'panels',
      setMosaic: (node) => set({ mosaic: sanitizeMosaic(node) }),
      setCatalogOpen: (v) => set({ catalogOpen: v }),
      setSidebarTab: (t) => {
        const { sidebarTab, catalogOpen } = get();
        if (catalogOpen && sidebarTab === t) {
          set({ catalogOpen: false });
          return;
        }
        set({ sidebarTab: t, catalogOpen: true });
      },
      resetGroundPreset: () => set({ mosaic: GROUND_ROBOT_LAYOUT }),
      applyLayoutPreset: (id) => {
        const preset = LAYOUT_PRESETS[id];
        if (!preset) return;
        set({ mosaic: sanitizeMosaic(structuredClone(preset.node)) });
      },
      exportLayoutJson: () => {
        const mosaic = get().mosaic;
        return JSON.stringify({ version: 1, mosaic }, null, 2);
      },
      importLayoutJson: (raw) => {
        try {
          const parsed = JSON.parse(raw) as { mosaic?: MosaicNode<string> } | MosaicNode<string>;
          const node =
            parsed && typeof parsed === 'object' && 'mosaic' in parsed
              ? parsed.mosaic ?? null
              : (parsed as MosaicNode<string>);
          const clean = sanitizeMosaic(node);
          if (!clean) return false;
          set({ mosaic: clean });
          return true;
        } catch {
          return false;
        }
      },
      ensurePanel: (id) => {
        const cur = get().mosaic;
        const existing = collectMosaicIds(cur);
        if (existing.some((leaf) => leaf === id || leaf.startsWith(`${id}#`))) return;
        set({ mosaic: sanitizeMosaic(insertMosaicLeaf(cur, id, 'row')) });
      },
    }),
    {
      name: 'orbisview-layout-v11',
      merge: (persisted, current) => {
        const p = (persisted ?? {}) as Partial<LayoutState> & { sidebarTab?: string };
        const tabRaw = p.sidebarTab as string | undefined;
        const sidebarTab: SidebarTab =
          tabRaw === 'channels' || tabRaw === 'resources'
            ? 'channels'
            : tabRaw === 'task'
              ? 'task'
              : tabRaw === 'setting'
                ? 'setting'
                : tabRaw === 'panels'
                  ? 'panels'
                  : current.sidebarTab;
        return {
          ...current,
          ...p,
          sidebarTab,
          mosaic: sanitizeMosaic(p.mosaic ?? current.mosaic),
        };
      },
    },
  ),
);

export type LayerKey =
  | 'grid'
  | 'basemap'
  | 'map'
  | 'costmap'
  | 'semantic'
  | 'vectormap'
  | 'poi'
  | 'draw'
  | 'path'
  | 'robot'
  | 'footprint'
  | 'obstacles'
  | 'prediction'
  | 'laser'
  | 'tf'
  | 'pointcloud'
  | 'image'
  | 'depth';

export interface LayerState extends Record<LayerKey, boolean> {
  followRobot: boolean;
  setLayer: (key: LayerKey, value: boolean) => void;
  setFollowRobot: (v: boolean) => void;
}

export const useLayerStore = create<LayerState>()(
  persist(
    (set) => ({
      grid: true,
      basemap: true,
      map: true,
      costmap: true,
      semantic: true,
      vectormap: true,
      poi: true,
      draw: true,
      path: true,
      robot: true,
      footprint: true,
      obstacles: true,
      prediction: true,
      laser: true,
      tf: true,
      pointcloud: true,
      image: true,
      depth: true,
      followRobot: true,
      setLayer: (key, value) => set({ [key]: value }),
      setFollowRobot: (v) => set({ followRobot: v }),
    }),
    {
      name: 'orbisview-layers-v8',
      merge: (persisted, current) => {
        const p = (persisted ?? {}) as Partial<LayerState>;
        return {
          ...current,
          ...p,
          basemap: p.basemap ?? true,
          semantic: p.semantic ?? true,
          poi: p.poi ?? true,
          draw: p.draw ?? true,
        };
      },
    },
  ),
);
