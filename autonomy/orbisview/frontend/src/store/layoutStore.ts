import { create } from 'zustand';
import { persist } from 'zustand/middleware';
import type { MosaicNode } from 'react-mosaic-component';

export type PanelId = string;
export type BottomMode = 'teleop' | 'pnc' | 'ops';
export type SidebarTab = 'mode' | 'panels' | 'resources' | 'layers';

export const GROUND_ROBOT_LAYOUT: MosaicNode<string> = {
  type: 'split',
  direction: 'row',
  children: [
    'map2d',
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

/** Collect mosaic leaf panel ids (depth-first). */
export function collectMosaicIds(node: MosaicNode<string> | null): string[] {
  if (node == null) return [];
  if (typeof node === 'string') return [node];
  if ('children' in node && Array.isArray(node.children)) {
    return node.children.flatMap((c) => collectMosaicIds(c as MosaicNode<string>));
  }
  return [];
}

/** Drop corrupt trees (duplicate leaf ids crash react-mosaic). */
export function sanitizeMosaic(
  node: MosaicNode<string> | null,
): MosaicNode<string> | null {
  if (!node) return null;
  const ids = collectMosaicIds(node);
  if (ids.length === 0) return GROUND_ROBOT_LAYOUT;
  if (new Set(ids).size !== ids.length) return GROUND_ROBOT_LAYOUT;
  return node;
}

export interface LayoutState {
  mosaic: MosaicNode<string> | null;
  bottomMode: BottomMode;
  catalogOpen: boolean;
  sidebarTab: SidebarTab;
  setMosaic: (node: MosaicNode<string> | null) => void;
  setBottomMode: (mode: BottomMode) => void;
  setCatalogOpen: (v: boolean) => void;
  setSidebarTab: (t: SidebarTab) => void;
  resetGroundPreset: () => void;
}

export const useLayoutStore = create<LayoutState>()(
  persist(
    (set) => ({
      mosaic: GROUND_ROBOT_LAYOUT,
      bottomMode: 'teleop',
      catalogOpen: true,
      sidebarTab: 'panels',
      setMosaic: (node) => set({ mosaic: sanitizeMosaic(node) }),
      setBottomMode: (mode) => set({ bottomMode: mode }),
      setCatalogOpen: (v) => set({ catalogOpen: v }),
      setSidebarTab: (t) => set({ sidebarTab: t, catalogOpen: true }),
      resetGroundPreset: () => set({ mosaic: GROUND_ROBOT_LAYOUT }),
    }),
    {
      name: 'orbisview-layout-v8',
      merge: (persisted, current) => {
        const p = (persisted ?? {}) as Partial<LayoutState>;
        return {
          ...current,
          ...p,
          mosaic: sanitizeMosaic(p.mosaic ?? current.mosaic),
        };
      },
    },
  ),
);

export type LayerKey =
  | 'grid'
  | 'map'
  | 'costmap'
  | 'vectormap'
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
      map: true,
      costmap: true,
      vectormap: true,
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
    { name: 'orbisview-layers-v5' },
  ),
);
