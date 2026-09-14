import { useEffect, useMemo, useRef } from 'react';
import { useDataStore } from '@/store/dataStore';
import { SCHEMAS } from '@/store/websocket/types';
import { wsClient } from '@/store/websocket/client';
import { usePanelOptsStore } from '@/store/panelOptsStore';
import { useDisplayStore } from '@/store/displayStore';
import { enabledImageChannels } from '@/components/Channels/sensorDisplay';
import type { PanelProps } from '@/components/registry';

const IMAGE_SCHEMAS: ReadonlySet<string> = new Set([SCHEMAS.Image, SCHEMAS.DepthImage]);

function depthToRgb(v: number): [number, number, number] {
  const t = Math.max(0, Math.min(1, v / 255));
  const r = Math.round(255 * Math.min(1, t * 2));
  const g = Math.round(255 * (1 - Math.abs(t - 0.5) * 2));
  const b = Math.round(255 * Math.min(1, (1 - t) * 2));
  return [r, g, b];
}

function drawMono(
  canvas: HTMLCanvasElement,
  width: number,
  height: number,
  data: number[],
  mode: 'gray' | 'depth',
) {
  const ctx = canvas.getContext('2d');
  if (!ctx) return;
  canvas.width = width;
  canvas.height = height;
  const img = ctx.createImageData(width, height);
  for (let i = 0; i < width * height; i++) {
    const v = data[i] ?? 0;
    if (mode === 'depth') {
      const [r, g, b] = depthToRgb(v);
      img.data[i * 4] = r;
      img.data[i * 4 + 1] = g;
      img.data[i * 4 + 2] = b;
    } else {
      img.data[i * 4] = v;
      img.data[i * 4 + 1] = v;
      img.data[i * 4 + 2] = v;
    }
    img.data[i * 4 + 3] = 255;
  }
  ctx.putImageData(img, 0, 0);
}

function shortName(channel: string): string {
  const parts = channel.split('/');
  return parts[parts.length - 1] || channel;
}

/** Single-channel image / depth viewer; channel chosen per mosaic instance. */
export function ImagePanel({ panelId }: PanelProps) {
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const channels = useDataStore((s) => s.channels);
  const envelopes = useDataStore((s) => s.envelopes);
  const markSubscribed = useDataStore((s) => s.markSubscribed);
  const displays = useDisplayStore((s) => s.displays);
  const channel = usePanelOptsStore((s) => s.image[panelId]?.channel ?? null);
  const setImageChannel = usePanelOptsStore((s) => s.setImageChannel);

  const displayImageChannels = useMemo(() => enabledImageChannels(displays), [displays]);

  const candidates = useMemo(() => {
    const fromList = channels.filter((c) => IMAGE_SCHEMAS.has(c.schema));
    const seen = new Set(fromList.map((c) => c.name));
    for (const env of Object.values(envelopes)) {
      if (IMAGE_SCHEMAS.has(env.schema) && !seen.has(env.channel)) {
        fromList.push({
          name: env.channel,
          schema: env.schema,
          msg_type: '',
          has_writer: true,
          mock: false,
        });
        seen.add(env.channel);
      }
    }
    for (const name of displayImageChannels) {
      if (seen.has(name)) continue;
      const env = envelopes[name];
      fromList.unshift({
        name,
        schema: env?.schema ?? SCHEMAS.Image,
        msg_type: '',
        has_writer: true,
        mock: false,
      });
      seen.add(name);
    }
    return fromList.sort((a, b) => {
      const ap = displayImageChannels.includes(a.name) ? 0 : 1;
      const bp = displayImageChannels.includes(b.name) ? 0 : 1;
      if (ap !== bp) return ap - bp;
      return a.name.localeCompare(b.name);
    });
  }, [channels, envelopes, displayImageChannels]);

  // Prefer this panel's bound channel (set when opening from Channels).
  const active =
    (channel && (candidates.some((c) => c.name === channel) || !!envelopes[channel])
      ? channel
      : null) ||
    (candidates[0]?.name ?? null);

  useEffect(() => {
    if (channel) return;
    if (candidates[0]) setImageChannel(panelId, candidates[0].name);
  }, [channel, candidates, panelId, setImageChannel]);

  useEffect(() => {
    if (!active) return;
    wsClient.subscribe(active, 10);
    markSubscribed(active, 10);
  }, [active, markSubscribed]);

  const env = active ? envelopes[active] : undefined;
  const mode: 'gray' | 'depth' =
    env?.schema === SCHEMAS.DepthImage ? 'depth' : 'gray';

  useEffect(() => {
    const payload = env?.payload as
      | { width: number; height: number; data: number[] }
      | undefined;
    if (canvasRef.current && payload) {
      drawMono(canvasRef.current, payload.width, payload.height, payload.data, mode);
    }
  }, [env, mode]);

  return (
    <div className="panel image-panel">
      <div className="image-panel-toolbar">
        <label className="image-channel-label">
          <span className="muted">channel</span>
          <select
            className="image-channel-select"
            value={active ?? ''}
            disabled={candidates.length === 0}
            onChange={(e) => setImageChannel(panelId, e.target.value || null)}
          >
            {candidates.length === 0 ? (
              <option value="">no image channels</option>
            ) : (
              candidates.map((c) => (
                <option key={c.name} value={c.name}>
                  {shortName(c.name)}
                  {displayImageChannels.includes(c.name) ? ' · display' : ''}
                  {c.schema === SCHEMAS.DepthImage ? ' · depth' : ''}
                </option>
              ))
            )}
          </select>
        </label>
        {env?.stale ? <span className="stale-badge">stale</span> : null}
      </div>
      {active && env?.payload ? (
        <canvas
          ref={canvasRef}
          className="image-canvas"
          style={{ width: '100%', maxWidth: 720, height: 'auto', imageRendering: 'pixelated' }}
        />
      ) : (
        <p className="muted">
          {candidates.length === 0
            ? 'Add an Image display in Channels, or wait for discovery'
            : 'waiting for frames…'}
        </p>
      )}
    </div>
  );
}
