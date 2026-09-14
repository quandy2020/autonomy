import { create } from 'zustand';
import type { TfXform } from '@/renderer/map2d/tfCompose';

interface TfBufferState {
  /** Latest edge keyed by child frame (TF: one parent per child). */
  byChild: Record<string, TfXform>;
  /** Merge transforms from a TfTree envelope (does not wipe other children). */
  ingest: (transforms: TfXform[] | null | undefined) => void;
  clear: () => void;
  list: () => TfXform[];
}

export const useTfBufferStore = create<TfBufferState>((set, get) => ({
  byChild: {},
  ingest: (transforms) => {
    if (!transforms?.length) return;
    set((s) => {
      let changed = false;
      const next = { ...s.byChild };
      for (const t of transforms) {
        if (!t?.parent || !t?.child || t.parent === t.child) continue;
        const prev = next[t.child];
        if (
          !prev ||
          prev.parent !== t.parent ||
          prev.x !== t.x ||
          prev.y !== t.y ||
          (prev.yaw ?? 0) !== (t.yaw ?? 0)
        ) {
          next[t.child] = {
            parent: t.parent,
            child: t.child,
            x: t.x,
            y: t.y,
            yaw: t.yaw ?? 0,
          };
          changed = true;
        }
      }
      return changed ? { byChild: next } : s;
    });
  },
  clear: () => set({ byChild: {} }),
  list: () => Object.values(get().byChild),
}));
