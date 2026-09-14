import { useEffect, useRef } from 'react';
import { asPayload, pickDisplayEnvelope } from '@/components/Channels/mapDisplayBinding';
import { basemapFromOccupancyGrid } from '@/renderer/map2d/mirrorOccupancyBasemap';
import type { OccupancyGridJson } from '@/renderer/map2d/types';
import { occupancyCacheKey } from '@/renderer/map2d/occupancyTexture';
import { useDataStore } from '@/store/dataStore';
import { useDisplayStore } from '@/store/displayStore';
import { useLayerStore } from '@/store/layoutStore';
import { useMappingVizStore } from '@/store/mappingVizStore';
import { useStaticSlamStore } from '@/store/staticSlamStore';

const DEBOUNCE_MS = 150;

/** When live+mirror: bake map OccupancyGrid into StaticSlamBasemap. */
export function useMappingLiveMirror(): void {
  const mode = useMappingVizStore((s) => s.mode);
  const mirror = useMappingVizStore((s) => s.live.mirrorMapToBasemap);
  const envelopes = useDataStore((s) => s.envelopes);
  const displays = useDisplayStore((s) => s.displays);
  const lastKey = useRef('');
  const timer = useRef<number | null>(null);
  const inflight = useRef(0);

  useEffect(() => {
    if (mode !== 'live' || !mirror) {
      lastKey.current = '';
      return;
    }

    const env = pickDisplayEnvelope(envelopes, displays, 'map');
    const grid = asPayload<OccupancyGridJson>(env);
    if (!grid || grid.width <= 0 || grid.height <= 0) return;

    const key = occupancyCacheKey(grid, 'map');
    if (key === lastKey.current) return;

    if (timer.current != null) window.clearTimeout(timer.current);
    timer.current = window.setTimeout(() => {
      timer.current = null;
      if (key === lastKey.current) return;
      const seq = ++inflight.current;
      void basemapFromOccupancyGrid(grid)
        .then((b) => {
          if (seq !== inflight.current) {
            try {
              URL.revokeObjectURL(b.imageSrc);
            } catch {
              /* ignore */
            }
            return;
          }
          const st = useMappingVizStore.getState();
          if (st.mode !== 'live' || !st.live.mirrorMapToBasemap) {
            try {
              URL.revokeObjectURL(b.imageSrc);
            } catch {
              /* ignore */
            }
            return;
          }
          lastKey.current = key;
          useStaticSlamStore.getState().setBasemap(b, { revokePrevious: true });
          useLayerStore.getState().setLayer('basemap', true);
        })
        .catch(() => {
          /* ignore bake errors */
        });
    }, DEBOUNCE_MS);

    return () => {
      if (timer.current != null) {
        window.clearTimeout(timer.current);
        timer.current = null;
      }
    };
  }, [mode, mirror, envelopes, displays]);
}
