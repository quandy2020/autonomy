import { create } from 'zustand';
import { persist } from 'zustand/middleware';

export interface ImagePanelOpts {
  channel: string | null;
}

export interface PanelOptsState {
  image: Record<string, ImagePanelOpts>;
  setImageChannel: (panelId: string, channel: string | null) => void;
  prunePanels: (aliveIds: string[]) => void;
}

export const usePanelOptsStore = create<PanelOptsState>()(
  persist(
    (set) => ({
      image: {},
      setImageChannel: (panelId, channel) =>
        set((s) => ({
          image: {
            ...s.image,
            [panelId]: { channel },
          },
        })),
      prunePanels: (aliveIds) =>
        set((s) => {
          const alive = new Set(aliveIds);
          const next: Record<string, ImagePanelOpts> = {};
          for (const [id, opts] of Object.entries(s.image)) {
            if (alive.has(id)) next[id] = opts;
          }
          return { image: next };
        }),
    }),
    { name: 'orbisview-panel-opts-v1' },
  ),
);
