import { create } from 'zustand';
import { persist } from 'zustand/middleware';

export interface Waypoint {
  id: string;
  x: number;
  y: number;
  label?: string;
}

interface WaypointState {
  waypoints: Waypoint[];
  selectedId: string | null;
  add: (x: number, y: number, label?: string) => void;
  remove: (id: string) => void;
  moveUp: (id: string) => void;
  moveDown: (id: string) => void;
  select: (id: string | null) => void;
  clear: () => void;
}

export const useWaypointStore = create<WaypointState>()(
  persist(
    (set, get) => ({
      waypoints: [],
      selectedId: null,
      add: (x, y, label) => {
        const id = `wp-${Date.now()}-${Math.random().toString(36).slice(2, 6)}`;
        set({
          waypoints: [...get().waypoints, { id, x, y, label }],
          selectedId: id,
        });
      },
      remove: (id) => {
        const waypoints = get().waypoints.filter((w) => w.id !== id);
        const selectedId = get().selectedId === id ? waypoints[0]?.id ?? null : get().selectedId;
        set({ waypoints, selectedId });
      },
      moveUp: (id) => {
        const list = [...get().waypoints];
        const i = list.findIndex((w) => w.id === id);
        if (i <= 0) return;
        [list[i - 1], list[i]] = [list[i], list[i - 1]];
        set({ waypoints: list });
      },
      moveDown: (id) => {
        const list = [...get().waypoints];
        const i = list.findIndex((w) => w.id === id);
        if (i < 0 || i >= list.length - 1) return;
        [list[i], list[i + 1]] = [list[i + 1], list[i]];
        set({ waypoints: list });
      },
      select: (id) => set({ selectedId: id }),
      clear: () => set({ waypoints: [], selectedId: null }),
    }),
    { name: 'orbisview-waypoints-v1' },
  ),
);
