import { create } from 'zustand';
import { persist } from 'zustand/middleware';
import type { StaticSlamBasemap } from '@/renderer/map2d/staticSlam';
import { sharedStaticSlamCanvasCache } from '@/renderer/map2d/staticSlam';

export type SetBasemapOpts = { revokePrevious?: boolean };

interface StaticSlamState {
  basemap: StaticSlamBasemap | null;
  formDraft: { originX: number; originY: number; resolution: number };
  setBasemap: (b: StaticSlamBasemap | null, opts?: SetBasemapOpts) => void;
  setFormDraft: (p: Partial<StaticSlamState['formDraft']>) => void;
  clearBasemap: () => void;
}

function revokeIfBlob(src: string | undefined) {
  if (src?.startsWith('blob:')) {
    try {
      URL.revokeObjectURL(src);
    } catch {
      /* ignore */
    }
  }
}

export const useStaticSlamStore = create<StaticSlamState>()(
  persist(
    (set, get) => ({
      basemap: null,
      formDraft: { originX: -10, originY: -10, resolution: 0.05 },
      setBasemap: (b, opts) => {
        const prev = get().basemap;
        const revokePrevious = opts?.revokePrevious !== false;
        if (
          revokePrevious &&
          prev &&
          prev.imageSrc !== b?.imageSrc
        ) {
          revokeIfBlob(prev.imageSrc);
        }
        sharedStaticSlamCanvasCache.clear();
        set({
          basemap: b,
          formDraft: b
            ? {
                originX: b.originX,
                originY: b.originY,
                resolution: b.resolution,
              }
            : get().formDraft,
        });
      },
      setFormDraft: (p) => set({ formDraft: { ...get().formDraft, ...p } }),
      clearBasemap: () => {
        const prev = get().basemap;
        revokeIfBlob(prev?.imageSrc);
        sharedStaticSlamCanvasCache.clear();
        set({ basemap: null });
      },
    }),
    {
      name: 'orbisview-static-slam-v1',
      partialize: (s) => {
        const b = s.basemap;
        const persistBasemap =
          b && b.source !== 'file' && !b.imageSrc.startsWith('blob:')
            ? b
            : b
              ? {
                  ...b,
                  imageSrc: '',
                  // Keep geometry for re-attach after re-pick
                }
              : null;
        return {
          formDraft: s.formDraft,
          basemap:
            persistBasemap && persistBasemap.imageSrc
              ? persistBasemap
              : b && b.source === 'file'
                ? {
                    imageSrc: '',
                    originX: b.originX,
                    originY: b.originY,
                    resolution: b.resolution,
                    widthPx: b.widthPx,
                    heightPx: b.heightPx,
                    label: b.label,
                    source: 'file' as const,
                  }
                : persistBasemap,
        };
      },
      merge: (persisted, current) => {
        const p = persisted as Partial<StaticSlamState> | undefined;
        const basemap = p?.basemap;
        // Drop restored file stubs without a usable imageSrc
        const restored =
          basemap && basemap.imageSrc ? basemap : null;
        return {
          ...current,
          ...p,
          basemap: restored,
          formDraft: { ...current.formDraft, ...p?.formDraft },
        };
      },
    },
  ),
);
