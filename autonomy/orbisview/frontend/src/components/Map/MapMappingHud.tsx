import { useMemo } from 'react';
import { useDataStore } from '@/store/dataStore';
import { useMappingVizStore } from '@/store/mappingVizStore';
import { SCHEMAS } from '@/store/websocket/types';

type MappingPayload = { status?: string; keyframes?: number; mode?: string };

/** Compact Map overlay for mapping demo / live status. */
export function MapMappingHud() {
  const mode = useMappingVizStore((s) => s.mode);
  const demo = useMappingVizStore((s) => s.demo);
  const mirror = useMappingVizStore((s) => s.live.mirrorMapToBasemap);
  const pause = useMappingVizStore((s) => s.pause);
  const envelopes = useDataStore((s) => s.envelopes);

  const mapping = useMemo(() => {
    const e = Object.values(envelopes).find((x) => x.schema === SCHEMAS.Mapping);
    return (e?.payload as MappingPayload | undefined) ?? null;
  }, [envelopes]);

  const showDemo = mode === 'demo' && demo.frameUrls.length > 0;
  const showLive = mode === 'live';
  if (!showDemo && !showLive) return null;

  let body: string;
  if (showDemo) {
    const n = demo.frameUrls.length;
    const i = demo.frameIndex + 1;
    let state = '已暂停';
    if (demo.playing) state = '播放中';
    else if (demo.frameIndex >= n - 1 && n > 0) state = '完成';
    body = `帧 ${i}/${n} · ${state}`;
  } else {
    const kf = mapping?.keyframes ?? '—';
    const st = mapping?.status ?? mapping?.mode ?? 'waiting';
    body = `keyframes ${kf} · ${st}`;
    if (mirror) body += ' · mirror';
  }

  return (
    <div className="map-mapping-hud" role="status">
      <span className="map-mapping-hud-mode">{mode === 'demo' ? '演示' : '实时'}</span>
      <span className="map-mapping-hud-body">{body}</span>
      {showDemo && demo.playing ? (
        <button type="button" className="map-mapping-hud-btn" onClick={() => pause()}>
          暂停
        </button>
      ) : null}
    </div>
  );
}
