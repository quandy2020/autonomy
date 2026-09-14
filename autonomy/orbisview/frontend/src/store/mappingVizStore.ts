import { create } from 'zustand';
import { persist } from 'zustand/middleware';
import {
  sortMappingFrameUrls,
  type MappingBasemapMeta,
} from '@/renderer/map2d/mappingDemo';

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

function revokeBlob(src: string) {
  if (!src.startsWith('blob:')) return;
  try {
    URL.revokeObjectURL(src);
  } catch {
    /* ignore */
  }
}

function revokeAll(urls: string[]) {
  for (const u of urls) revokeBlob(u);
}

export const useMappingVizStore = create<MappingVizState>()(
  persist(
    (set, get) => ({
      mode: 'demo',
      demo: {
        frameUrls: [],
        frameIndex: 0,
        playing: false,
        intervalMs: 500,
        basemapMeta: null,
      },
      live: { mirrorMapToBasemap: false },
      elapsedMs: 0,
      setMode: (m) => {
        if (m === 'live') {
          set({
            mode: m,
            demo: { ...get().demo, playing: false },
          });
        } else {
          set({ mode: m });
        }
      },
      setIntervalMs: (ms) =>
        set({
          demo: {
            ...get().demo,
            intervalMs: Math.max(50, Math.min(10_000, Math.round(ms))),
          },
        }),
      setMirrorMapToBasemap: (v) =>
        set({ live: { ...get().live, mirrorMapToBasemap: v } }),
      loadDemoFrames: (urls, meta) => {
        const prev = get().demo.frameUrls;
        const sorted = sortMappingFrameUrls(urls);
        const keep = new Set(sorted);
        for (const u of prev) {
          if (!keep.has(u)) revokeBlob(u);
        }
        set({
          demo: {
            ...get().demo,
            frameUrls: sorted,
            frameIndex: 0,
            playing: false,
            basemapMeta: meta,
          },
          elapsedMs: 0,
        });
      },
      clearDemoFrames: () => {
        revokeAll(get().demo.frameUrls);
        set({
          demo: {
            ...get().demo,
            frameUrls: [],
            frameIndex: 0,
            playing: false,
            basemapMeta: null,
          },
          elapsedMs: 0,
        });
      },
      play: () => {
        const { demo } = get();
        if (demo.frameUrls.length === 0) return;
        set({ demo: { ...demo, playing: true } });
      },
      pause: () => set({ demo: { ...get().demo, playing: false } }),
      reset: () =>
        set({
          demo: { ...get().demo, frameIndex: 0, playing: false },
          elapsedMs: 0,
        }),
      nextFrame: () => {
        const { demo } = get();
        const last = demo.frameUrls.length - 1;
        if (last < 0 || demo.frameIndex >= last) {
          set({ demo: { ...demo, playing: false } });
          return false;
        }
        set({ demo: { ...demo, frameIndex: demo.frameIndex + 1 } });
        return true;
      },
      setFrameIndex: (i) => {
        const { demo } = get();
        if (demo.frameUrls.length === 0) return;
        const clamped = Math.max(0, Math.min(demo.frameUrls.length - 1, Math.round(i)));
        set({ demo: { ...demo, frameIndex: clamped } });
      },
      addElapsed: (dt) => set({ elapsedMs: get().elapsedMs + dt }),
    }),
    {
      name: 'orbisview-mapping-viz-v1',
      partialize: (s) => ({
        mode: s.mode,
        demo: { intervalMs: s.demo.intervalMs },
        live: { mirrorMapToBasemap: s.live.mirrorMapToBasemap },
      }),
      merge: (persisted, current) => {
        const p = persisted as Partial<{
          mode: MappingVizMode;
          demo: { intervalMs?: number };
          live: { mirrorMapToBasemap?: boolean };
        }> | undefined;
        return {
          ...current,
          mode: p?.mode === 'live' || p?.mode === 'demo' ? p.mode : current.mode,
          demo: {
            ...current.demo,
            intervalMs: p?.demo?.intervalMs ?? current.demo.intervalMs,
          },
          live: {
            mirrorMapToBasemap:
              p?.live?.mirrorMapToBasemap ?? current.live.mirrorMapToBasemap,
          },
        };
      },
    },
  ),
);
