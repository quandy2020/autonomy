import { create } from 'zustand';
import { persist } from 'zustand/middleware';
import type { CloudColorMode, View3DOpts } from '@/renderer/view3d/types';

interface View3DStore extends View3DOpts {
  setCloudColor: (v: CloudColorMode) => void;
  setLaserHeight: (v: number) => void;
  setMapOpacity: (v: number) => void;
}

export const useView3DStore = create<View3DStore>()(
  persist(
    (set) => ({
      cloudColor: 'intensity',
      laserHeight: 0.1,
      mapOpacity: 0.7,
      setCloudColor: (cloudColor) => set({ cloudColor }),
      setLaserHeight: (laserHeight) => set({ laserHeight }),
      setMapOpacity: (mapOpacity) => set({ mapOpacity }),
    }),
    { name: 'orbisview-view3d-opts-v2' },
  ),
);
