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
  /** Legacy aux for non-mosaic fallbacks */
  auxActive: PanelId;
  open: PanelId[];
  setAuxActive: (id: PanelId) => void;
  toggleOpen: (id: PanelId) => void;
  active: PanelId;
  setActive: (id: PanelId) => void;
}

export const useLayoutStore = create<LayoutState>()(
  persist(
    (set, get) => ({
      mosaic: GROUND_ROBOT_LAYOUT,
      bottomMode: 'teleop',
      catalogOpen: true,
      sidebarTab: 'panels',
      auxActive: 'dashboard',
      active: 'dashboard',
      open: [
        'map2d',
        'dashboard',
        'mode_settings',
        'module_delay',
        'resources',
        'robot_status',
        'waypoints',
        'view3d',
        'image',
        'pnc',
        'charts',
        'components',
        'hmi',
        'routing',
        'tf_tree',
        'log',
      ],
      setMosaic: (node) => set({ mosaic: node }),
      setBottomMode: (mode) => set({ bottomMode: mode }),
      setCatalogOpen: (v) => set({ catalogOpen: v }),
      setSidebarTab: (t) => set({ sidebarTab: t, catalogOpen: true }),
      resetGroundPreset: () => set({ mosaic: GROUND_ROBOT_LAYOUT }),
      setAuxActive: (id) => set({ auxActive: id, active: id }),
      setActive: (id) => set({ auxActive: id, active: id }),
      toggleOpen: (id) => {
        const open = get().open.includes(id)
          ? get().open.filter((x) => x !== id)
          : [...get().open, id];
        set({ open });
      },
    }),
    { name: 'orbisview-layout-v6' },
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
