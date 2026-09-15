import { create } from 'zustand';
import { persist } from 'zustand/middleware';

export type ChartTimeRange = { t0: number; t1: number };

/**
 * Shared Charts timebase: freeze + optional cross-panel view sync
 * (so cmd_vel / odom plots share the same window when debugging).
 */
export interface ChartTimeState {
  /** Pause live follow across all Charts panels. */
  frozen: boolean;
  /** Wall-clock "now" captured at freeze; used as stable t1 / age origin. */
  freezeNowMs: number | null;
  /** When true, zoom/pan range is shared by all Charts panels. */
  synced: boolean;
  /** Shared zoom window; null = live follow (subject to freeze). */
  view: ChartTimeRange | null;
  setFrozen: (frozen: boolean) => void;
  toggleFrozen: () => void;
  setSynced: (synced: boolean) => void;
  setView: (view: ChartTimeRange | null) => void;
  /** Effective "now" for drawing / relative labels. */
  effectiveNowMs: () => number;
}

export const useChartTimeStore = create<ChartTimeState>()(
  persist(
    (set, get) => ({
      frozen: false,
      freezeNowMs: null,
      synced: true,
      view: null,
      setFrozen: (frozen) =>
        set(() =>
          frozen
            ? { frozen: true, freezeNowMs: Date.now() }
            : { frozen: false, freezeNowMs: null },
        ),
      toggleFrozen: () => {
        const { frozen } = get();
        get().setFrozen(!frozen);
      },
      setSynced: (synced) => set({ synced }),
      setView: (view) => set({ view }),
      effectiveNowMs: () => {
        const s = get();
        if (s.frozen && s.freezeNowMs != null) return s.freezeNowMs;
        return Date.now();
      },
    }),
    {
      name: 'orbisview-chart-time-v1',
      partialize: (s) => ({ synced: s.synced }),
      merge: (persisted, current) => {
        const p = (persisted ?? {}) as Partial<Pick<ChartTimeState, 'synced'>>;
        return {
          ...current,
          synced: p.synced ?? true,
          // Never restore freeze across reloads.
          frozen: false,
          freezeNowMs: null,
          view: null,
        };
      },
    },
  ),
);
