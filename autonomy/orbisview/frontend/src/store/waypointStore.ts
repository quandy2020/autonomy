import { create } from 'zustand';
import { persist } from 'zustand/middleware';

export interface Waypoint {
  id: string;
  x: number;
  y: number;
  yaw: number;
  label?: string;
}

/** Distinct colors for multi-point route markers (index-stable). */
export const WAYPOINT_PALETTE = [
  '#4fc3f7',
  '#81c784',
  '#ffb74d',
  '#ce93d8',
  '#ef9a9a',
  '#80cbc4',
  '#fff176',
  '#90caf9',
  '#a5d6a7',
  '#f48fb1',
] as const;

export function waypointColor(index: number): string {
  return WAYPOINT_PALETTE[index % WAYPOINT_PALETTE.length];
}

/** Halo / emphasis when a waypoint is selected. */
export const WAYPOINT_SELECTED_HALO = '#ffee58';

export type WaypointPatch = Partial<Pick<Waypoint, 'x' | 'y' | 'yaw' | 'label'>>;

interface WaypointState {
  waypoints: Waypoint[];
  selectedId: string | null;
  add: (x: number, y: number, yaw?: number, label?: string) => void;
  update: (id: string, patch: WaypointPatch) => void;
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
      add: (x, y, yaw = 0, label) => {
        const id = `wp-${Date.now()}-${Math.random().toString(36).slice(2, 6)}`;
        set({
          waypoints: [...get().waypoints, { id, x, y, yaw, label }],
          selectedId: id,
        });
      },
      update: (id, patch) => {
        set({
          waypoints: get().waypoints.map((w) => (w.id === id ? { ...w, ...patch } : w)),
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
    {
      name: 'orbisview-waypoints-v2',
      merge: (persisted, current) => {
        const p = (persisted ?? {}) as Partial<WaypointState>;
        const waypoints = (p.waypoints ?? []).map((w) => ({
          ...w,
          yaw: typeof w.yaw === 'number' ? w.yaw : 0,
        }));
        return { ...current, ...p, waypoints };
      },
    },
  ),
);
