import { useCallback, useEffect, useMemo, useRef, useState } from 'react';
import { useDataStore } from '@/store/dataStore';
import { SCHEMAS } from '@/store/websocket/types';
import { wsClient } from '@/store/websocket/client';
import { usePanelOptsStore } from '@/store/panelOptsStore';
import { useDisplayStore } from '@/store/displayStore';
import { enabledImageChannels } from '@/components/Channels/sensorDisplay';
import type { PanelProps } from '@/components/registry';

const IMAGE_SCHEMAS: ReadonlySet<string> = new Set([SCHEMAS.Image, SCHEMAS.DepthImage]);
const MIN_ZOOM = 0.25;
const MAX_ZOOM = 12;

/**
 * OpenCV-style JET for depth: near (small v) → red/yellow, far → blue.
 * v==0 → black (void / invalid).
 */
function depthToRgb(v: number): [number, number, number] {
  if (!(v > 0)) return [0, 0, 0];
  // Invert so near maps to the warm end of Jet (matches common depth viz).
  const t = 1 - Math.max(0, Math.min(1, v / 255));
  let r: number;
  let g: number;
  let b: number;
  if (t < 0.125) {
    r = 0;
    g = 0;
    b = 0.5 + (t / 0.125) * 0.5;
  } else if (t < 0.375) {
    r = 0;
    g = (t - 0.125) / 0.25;
    b = 1;
  } else if (t < 0.625) {
    r = (t - 0.375) / 0.25;
    g = 1;
    b = 1 - (t - 0.375) / 0.25;
  } else if (t < 0.875) {
    r = 1;
    g = 1 - (t - 0.625) / 0.25;
    b = 0;
  } else {
    r = 1 - ((t - 0.875) / 0.125) * 0.5;
    g = 0;
    b = 0;
  }
  return [
    Math.round(255 * Math.max(0, Math.min(1, r))),
    Math.round(255 * Math.max(0, Math.min(1, g))),
    Math.round(255 * Math.max(0, Math.min(1, b))),
  ];
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
  let rank = inDisplay ? 0 : 100;
  if (isRgbChannel(name)) rank += 0;
  else if (isDepthChannel(name, schema)) rank += 20;
  else rank += 10;
  return rank;
}

function shortChannel(name: string): string {
  const parts = name.split('/').filter(Boolean);
  if (parts.length <= 2) return name;
  return `…/${parts.slice(-2).join('/')}`;
}

type ViewState = { zoom: number; x: number; y: number };

/**
 * Image / depth viewer: compact chrome, contain-fit, wheel zoom + drag pan.
 */
export function ImagePanel({ panelId }: PanelProps) {
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const viewportRef = useRef<HTMLDivElement>(null);
  const stageRef = useRef<HTMLDivElement>(null);
  const channels = useDataStore((s) => s.channels);
  const envelopes = useDataStore((s) => s.envelopes);
  const connected = useDataStore((s) => s.connected);
  const markSubscribed = useDataStore((s) => s.markSubscribed);
  const displays = useDisplayStore((s) => s.displays);
  const channel = usePanelOptsStore((s) => s.image[panelId]?.channel ?? null);
  const setImageChannel = usePanelOptsStore((s) => s.setImageChannel);

  const [view, setView] = useState<ViewState>({ zoom: 1, x: 0, y: 0 });
  const [imgSize, setImgSize] = useState<{ w: number; h: number } | null>(null);
  const [fitSize, setFitSize] = useState<{ w: number; h: number }>({ w: 0, h: 0 });
  const [nowTick, setNowTick] = useState(0);
  const viewRef = useRef(view);
  viewRef.current = view;
  const dragRef = useRef<{ x: number; y: number; vx: number; vy: number } | null>(null);
  const drawGenRef = useRef(0);
  const frameTimesRef = useRef<number[]>([]);

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

  // Reset view when switching channel.
  useEffect(() => {
    setView({ zoom: 1, x: 0, y: 0 });
  }, [active]);

  const env = active ? envelopes[active] : undefined;
  const mode: 'gray' | 'depth' =
    env?.schema === SCHEMAS.DepthImage || isDepthChannel(active ?? '')
      ? 'depth'
      : 'gray';

  const payloadDims = useMemo(() => {
    const p = env?.payload as { width?: number; height?: number } | undefined;
    if (!p?.width || !p?.height) return null;
    return { w: p.width, h: p.height };
  }, [env]);

  useEffect(() => {
    if (payloadDims) setImgSize(payloadDims);
    else setImgSize(null);
  }, [payloadDims]);

  useEffect(() => {
    const payload = env?.payload as
      | { width: number; height: number; encoding?: string; data?: number[]; data_b64?: string }
      | undefined;
    // Canvas must already be mounted (hasPayload) — do not gate mount on imgSize.
    if (!canvasRef.current || !payload?.width || !payload?.height) return;

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
      } else {
        const asRgb = enc !== 'mono8' && data.length >= n * 3 && data.length !== n;
        if (asRgb && data.length >= n * 3) {
          drawRgb(canvas, width, height, data);
        } else {
          if (data.length < n) return;
          drawMono(canvas, width, height, data, paintMode);
        }
      }
      const now = Date.now();
      const times = frameTimesRef.current;
      times.push(now);
      while (times.length > 0 && times[0]! < now - 2000) times.shift();
    });
    return () => window.cancelAnimationFrame(raf);
  }, [env, mode, payloadDims]);

  const recomputeFit = useCallback(() => {
    const vp = viewportRef.current;
    if (!vp || !imgSize) {
      setFitSize({ w: 0, h: 0 });
      return;
    }
    const pw = Math.max(1, vp.clientWidth);
    const ph = Math.max(1, vp.clientHeight);
    const s = Math.min(pw / imgSize.w, ph / imgSize.h);
    setFitSize({
      w: Math.max(1, Math.floor(imgSize.w * s)),
      h: Math.max(1, Math.floor(imgSize.h * s)),
    });
  }, [imgSize]);

  useEffect(() => {
    recomputeFit();
    const vp = viewportRef.current;
    if (!vp || typeof ResizeObserver === 'undefined') return;
    const ro = new ResizeObserver(() => recomputeFit());
    ro.observe(vp);
    return () => ro.disconnect();
  }, [recomputeFit]);

  useEffect(() => {
    const id = window.setInterval(() => setNowTick((n) => n + 1), 400);
    return () => window.clearInterval(id);
  }, []);

  const resetView = useCallback(() => {
    setView({ zoom: 1, x: 0, y: 0 });
  }, []);

  // Wheel zoom around cursor (non-passive).
  useEffect(() => {
    const el = viewportRef.current;
    if (!el) return;
    const onWheel = (e: WheelEvent) => {
      e.preventDefault();
      e.stopPropagation();
      const rect = el.getBoundingClientRect();
      const mx = e.clientX - rect.left - rect.width / 2;
      const my = e.clientY - rect.top - rect.height / 2;
      const prev = viewRef.current;
      const factor = Math.exp(-e.deltaY * 0.0018);
      const nextZoom = Math.min(MAX_ZOOM, Math.max(MIN_ZOOM, prev.zoom * factor));
      if (Math.abs(nextZoom - 1) < 0.02 && factor > 1 === prev.zoom < 1) {
        setView({ zoom: 1, x: 0, y: 0 });
        return;
      }
      // Keep point under cursor stable.
      const u = (mx - prev.x) / prev.zoom;
      const v = (my - prev.y) / prev.zoom;
      setView({
        zoom: nextZoom,
        x: mx - u * nextZoom,
        y: my - v * nextZoom,
      });
    };
    el.addEventListener('wheel', onWheel, { passive: false });
    return () => el.removeEventListener('wheel', onWheel);
  }, []);

  const onPointerDown = (e: React.PointerEvent) => {
    if (e.button !== 0) return;
    const v = viewRef.current;
    if (v.zoom <= 1.02) return; // only pan when zoomed
    dragRef.current = { x: e.clientX, y: e.clientY, vx: v.x, vy: v.y };
    (e.currentTarget as HTMLElement).setPointerCapture?.(e.pointerId);
  };

  const onPointerMove = (e: React.PointerEvent) => {
    const d = dragRef.current;
    if (!d) return;
    setView({
      zoom: viewRef.current.zoom,
      x: d.vx + (e.clientX - d.x),
      y: d.vy + (e.clientY - d.y),
    });
  };

  const onPointerUp = () => {
    dragRef.current = null;
  };

  const hz = useMemo(() => {
    const times = frameTimesRef.current;
    if (times.length < 2) return null;
    const span = times[times.length - 1]! - times[0]!;
    if (span < 1) return null;
    return ((times.length - 1) / span) * 1000;
  }, [env, nowTick]);

  const ageMs =
    env?.timestamp && env.timestamp > 0
      ? Math.max(0, Date.now() - env.timestamp / 1e6)
      : null;
  void nowTick;

  const kind =
    mode === 'depth' ? 'depth' : isRgbChannel(active ?? '') ? 'rgb' : 'mono';
  const zoomPct = Math.round(view.zoom * 100);
  // Mount canvas as soon as payload dims exist (do not wait for paint → imgSize).
  const hasPayload = !!(active && payloadDims);

  return (
    <div className="panel image-panel">
      <div className="image-panel-toolbar">
        <select
          className="image-channel-select"
          value={active ?? ''}
          disabled={candidates.length === 0}
          onChange={(e) => setImageChannel(panelId, e.target.value || null)}
          title={active ?? 'Select image channel'}
          aria-label="Image channel"
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
        <span className={`image-kind-pill kind-${kind}`}>{kind}</span>
        {env?.stale ? <span className="stale-badge">stale</span> : null}
        {!connected ? <span className="image-meta-pill tone-bad">offline</span> : null}
        <button
          type="button"
          className="image-tool-btn"
          title="Fit image (or double-click)"
          disabled={view.zoom === 1 && view.x === 0 && view.y === 0}
          onClick={resetView}
        >
          Fit
        </button>
      </div>

      <div
        ref={viewportRef}
        className={`image-panel-viewport${view.zoom > 1.02 ? ' is-zoomed' : ''}`}
        onPointerDown={onPointerDown}
        onPointerMove={onPointerMove}
        onPointerUp={onPointerUp}
        onPointerCancel={onPointerUp}
        onDoubleClick={resetView}
      >
        {hasPayload ? (
          <div
            ref={stageRef}
            className="image-stage"
            style={{
              width: fitSize.w || undefined,
              height: fitSize.h || undefined,
              transform: `translate(${view.x}px, ${view.y}px) scale(${view.zoom})`,
            }}
          >
            <canvas ref={canvasRef} className="image-canvas" />
          </div>
        ) : (
          <p className="image-empty muted">
            {candidates.length === 0
              ? 'Add an Image display in Channels, or wait for discovery'
              : 'waiting for frames…'}
          </p>
        )}

        {hasPayload && imgSize ? (
          <div className="image-hud" aria-hidden>
            <span className="mono" title={active ?? ''}>
              {shortChannel(active!)}
            </span>
            <span className="mono">
              {imgSize.w}×{imgSize.h}
            </span>
            {hz != null ? <span className="mono">{hz.toFixed(hz >= 10 ? 0 : 1)}Hz</span> : null}
            {ageMs != null && Number.isFinite(ageMs) ? (
              <span className={`mono${ageMs > 1000 ? ' tone-warn' : ''}`}>
                {ageMs < 1000 ? `${Math.round(ageMs)}ms` : `${(ageMs / 1000).toFixed(1)}s`}
              </span>
            ) : null}
            <span className="mono">{zoomPct}%</span>
          </div>
        ) : null}
      </div>
    </div>
  );
}
