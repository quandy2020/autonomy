import { useEffect, useMemo, useRef } from 'react';
import { useDataStore } from '@/store/dataStore';
import { SCHEMAS } from '@/store/websocket/types';
import { wsClient } from '@/store/websocket/client';
import { usePanelOptsStore } from '@/store/panelOptsStore';
import { useDisplayStore } from '@/store/displayStore';
import { enabledImageChannels } from '@/components/Channels/sensorDisplay';
import type { PanelProps } from '@/components/registry';

const IMAGE_SCHEMAS: ReadonlySet<string> = new Set([SCHEMAS.Image, SCHEMAS.DepthImage]);

/** Near = warm, far = cool; v==0 → void (black), not colormap blue. */
function depthToRgb(v: number): [number, number, number] {
  if (!(v > 0)) return [0, 0, 0];
  const t = Math.max(0, Math.min(1, v / 255));
  // Invert prior map: near(0)→red, mid→white, far(1)→blue.
  const r = Math.round(255 * Math.min(1, (1 - t) * 2));
  const g = Math.round(255 * (1 - Math.abs(t - 0.5) * 2));
  const b = Math.round(255 * Math.min(1, t * 2));
  return [r, g, b];
}

function drawMono(
  canvas: HTMLCanvasElement,
  width: number,
  height: number,
  data: ArrayLike<number>,
  mode: 'gray' | 'depth',
) {
  const ctx = canvas.getContext('2d');
  if (!ctx) return;
  canvas.width = width;
  canvas.height = height;
  const img = ctx.createImageData(width, height);
  const n = width * height;
  for (let i = 0; i < n; i++) {
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

function drawRgb(
  canvas: HTMLCanvasElement,
  width: number,
  height: number,
  data: ArrayLike<number>,
) {
  const ctx = canvas.getContext('2d');
  if (!ctx) return;
  canvas.width = width;
  canvas.height = height;
  const img = ctx.createImageData(width, height);
  const n = width * height;
  for (let i = 0; i < n; i++) {
    const o = i * 3;
    img.data[i * 4] = data[o] ?? 0;
    img.data[i * 4 + 1] = data[o + 1] ?? 0;
    img.data[i * 4 + 2] = data[o + 2] ?? 0;
    img.data[i * 4 + 3] = 255;
  }
  ctx.putImageData(img, 0, 0);
}

function decodeImagePixels(payload: {
  data?: number[];
  data_b64?: string;
}): ArrayLike<number> | null {
  if (typeof payload.data_b64 === 'string' && payload.data_b64.length > 0) {
    try {
      const bin = atob(payload.data_b64);
      const out = new Uint8Array(bin.length);
      for (let i = 0; i < bin.length; i++) out[i] = bin.charCodeAt(i);
      return out;
    } catch {
      return null;
    }
  }
  if (Array.isArray(payload.data) && payload.data.length > 0) return payload.data;
  return null;
}

function isDepthChannel(name: string, schema?: string): boolean {
  if (schema === SCHEMAS.DepthImage) return true;
  return /depth|disparity/i.test(name);
}

function isRgbChannel(name: string): boolean {
  return /\/rgb\/|\/color\/|\/camera_color/i.test(name) && !isDepthChannel(name);
}

function channelOptionLabel(
  name: string,
  schema: string,
  inDisplay: boolean,
): string {
  const tags: string[] = [];
  if (inDisplay) tags.push('display');
  if (isDepthChannel(name, schema)) tags.push('depth');
  if (isRgbChannel(name)) tags.push('rgb');
  return tags.length ? `${name}  (${tags.join(', ')})` : name;
}

function imageRank(name: string, schema: string, inDisplay: boolean): number {
  // Lower sorts first. Prefer enabled display + RGB camera over depth.
  let rank = inDisplay ? 0 : 100;
  if (isRgbChannel(name)) rank += 0;
  else if (isDepthChannel(name, schema)) rank += 20;
  else rank += 10;
  return rank;
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
      const ar = imageRank(a.name, a.schema, displayImageChannels.includes(a.name));
      const br = imageRank(b.name, b.schema, displayImageChannels.includes(b.name));
      if (ar !== br) return ar - br;
      return a.name.localeCompare(b.name);
    });
  }, [channels, envelopes, displayImageChannels]);

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
    wsClient.subscribe(active, 0);
    markSubscribed(active, 0);
  }, [active, markSubscribed]);

  const env = active ? envelopes[active] : undefined;
  const mode: 'gray' | 'depth' =
    env?.schema === SCHEMAS.DepthImage || isDepthChannel(active ?? '')
      ? 'depth'
      : 'gray';
  const drawGenRef = useRef(0);

  useEffect(() => {
    const payload = env?.payload as
      | { width: number; height: number; encoding?: string; data?: number[]; data_b64?: string }
      | undefined;
    if (!canvasRef.current || !payload?.width || !payload?.height) {
      return;
    }
    // Drop intermediate frames: only paint the newest envelope after layout.
    const gen = ++drawGenRef.current;
    const canvas = canvasRef.current;
    const paintMode = mode;
    const raf = window.requestAnimationFrame(() => {
      if (gen !== drawGenRef.current || !canvas) return;
      const data = decodeImagePixels(payload);
      if (!data) return;
      const { width, height } = payload;
      const enc = (payload.encoding ?? '').toLowerCase();
      const n = width * height;
      if (enc === 'rgb8' || enc === 'bgr8' || enc === 'rgba8' || enc === 'bgra8') {
        if (data.length < n * 3) return;
        drawRgb(canvas, width, height, data);
        return;
      }
      const asRgb =
        enc !== 'mono8' && data.length >= n * 3 && data.length !== n;
      if (asRgb && data.length >= n * 3) {
        drawRgb(canvas, width, height, data);
        return;
      }
      if (data.length < n) return;
      drawMono(canvas, width, height, data, paintMode);
    });
    return () => window.cancelAnimationFrame(raf);
  }, [env, mode]);

  return (
    <div className="panel image-panel">
      <div className="image-panel-toolbar">
        <label className="image-channel-label">
          <span className="muted image-channel-k">channel</span>
          <select
            className="image-channel-select"
            value={active ?? ''}
            disabled={candidates.length === 0}
            onChange={(e) => setImageChannel(panelId, e.target.value || null)}
            title={active ?? undefined}
          >
            {candidates.length === 0 ? (
              <option value="">no image channels</option>
            ) : (
              candidates.map((c) => (
                <option key={c.name} value={c.name}>
                  {channelOptionLabel(
                    c.name,
                    c.schema,
                    displayImageChannels.includes(c.name),
                  )}
                </option>
              ))
            )}
          </select>
        </label>
        {env?.stale ? <span className="stale-badge">stale</span> : null}
      </div>
      {active ? (
        <div className="image-channel-path" title={active}>
          {active}
        </div>
      ) : null}
      <div className="image-panel-viewport">
        {active && env?.payload ? (
          <canvas ref={canvasRef} className="image-canvas" />
        ) : (
          <p className="muted">
            {candidates.length === 0
              ? 'Add an Image display in Channels, or wait for discovery'
              : 'waiting for frames…'}
          </p>
        )}
      </div>
    </div>
  );
}
