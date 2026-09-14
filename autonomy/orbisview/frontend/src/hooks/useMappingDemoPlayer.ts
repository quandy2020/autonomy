import { useEffect, useRef } from 'react';
import { basemapFromDemoFrame } from '@/renderer/map2d/mappingDemo';
import { useMappingVizStore } from '@/store/mappingVizStore';
import { useStaticSlamStore } from '@/store/staticSlamStore';
import { useLayerStore } from '@/store/layoutStore';

/** Interval advances demo frames and applies each frame to P1 basemap. */
export function useMappingDemoPlayer(): void {
  const mode = useMappingVizStore((s) => s.mode);
  const playing = useMappingVizStore((s) => s.demo.playing);
  const frameIndex = useMappingVizStore((s) => s.demo.frameIndex);
  const frameUrls = useMappingVizStore((s) => s.demo.frameUrls);
  const intervalMs = useMappingVizStore((s) => s.demo.intervalMs);
  const basemapMeta = useMappingVizStore((s) => s.demo.basemapMeta);
  const appliedKey = useRef('');

  // Apply current frame whenever index/urls/meta change in demo mode.
  useEffect(() => {
    if (mode !== 'demo') return;
    if (!basemapMeta || frameUrls.length === 0) return;
    const src = frameUrls[frameIndex];
    if (!src) return;
    const key = `${frameIndex}|${src}|${basemapMeta.originX}|${basemapMeta.originY}|${basemapMeta.resolution}|${basemapMeta.widthPx}|${basemapMeta.heightPx}`;
    if (appliedKey.current === key) return;
    appliedKey.current = key;
    useStaticSlamStore.getState().setBasemap(
      basemapFromDemoFrame({
        imageSrc: src,
        meta: basemapMeta,
        label: `demo ${frameIndex + 1}/${frameUrls.length}`,
        source: 'file',
      }),
      { revokePrevious: false },
    );
    useLayerStore.getState().setLayer('basemap', true);
  }, [mode, frameIndex, frameUrls, basemapMeta]);

  // Playback timer
  useEffect(() => {
    if (mode !== 'demo' || !playing) return;
    const id = window.setInterval(() => {
      const st = useMappingVizStore.getState();
      if (!st.demo.playing) return;
      const ok = st.nextFrame();
      st.addElapsed(st.demo.intervalMs);
      if (!ok) {
        /* nextFrame already paused */
      }
    }, intervalMs);
    return () => window.clearInterval(id);
  }, [mode, playing, intervalMs]);

  // Leaving demo → pause
  useEffect(() => {
    if (mode === 'live') useMappingVizStore.getState().pause();
  }, [mode]);
}
