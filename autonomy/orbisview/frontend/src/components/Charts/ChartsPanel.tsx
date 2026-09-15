import { useCallback, useEffect, useMemo, useRef, useState } from 'react';
import { useDataStore } from '@/store/dataStore';
import { wsClient } from '@/store/websocket/client';
import {
  DEFAULT_CHART_WINDOW_MS,
  type ChartSeriesSpec,
  usePanelOptsStore,
} from '@/store/panelOptsStore';
import {
  type ChartTimeRange,
  useChartTimeStore,
} from '@/store/chartTimeStore';
import type { PanelProps } from '@/components/registry';
import { collectNumericPaths, envelopeJsonPayload, readNumericPath } from './fieldPaths';
import {
  canvasXToTime,
  drawTimePlot,
  findNearestSample,
  formatSampleAge,
  formatSampleClock,
  type HoverHit,
  type PlotLayout,
  type PlotPoint,
} from './drawTimePlot';

const MAX_POINTS_PER_SERIES = 4000;
const WINDOW_OPTIONS_MS = [5_000, 15_000, 30_000, 60_000, 120_000] as const;
const EMPTY_SERIES: ChartSeriesSpec[] = [];
const MIN_ZOOM_MS = 40;
const MAX_ZOOM_MS = 300_000;
const HZ_WINDOW_MS = 2_000;

type SeriesLinkStats = {
  hz: number | null;
  ageMs: number | null;
  lastT: number | null;
};

function envelopeTimeMs(timestamp: number): number {
  const now = Date.now();
  if (!(timestamp > 0)) return now;
  // Wire timestamp is nanoseconds; reject sim/bad stamps far from wall clock.
  const ms = timestamp / 1e6;
  if (!Number.isFinite(ms) || Math.abs(now - ms) > 120_000) return now;
  return ms;
}

function trimBuffer(buf: PlotPoint[], cutoffMs: number): void {
  while (buf.length > 0 && buf[0]!.t < cutoffMs) buf.shift();
  if (buf.length > MAX_POINTS_PER_SERIES) {
    buf.splice(0, buf.length - MAX_POINTS_PER_SERIES);
  }
}

function estimateHz(times: number[], nowMs: number): number | null {
  const cutoff = nowMs - HZ_WINDOW_MS;
  const recent = times.filter((t) => t >= cutoff);
  if (recent.length < 2) return null;
  const span = recent[recent.length - 1]! - recent[0]!;
  if (!(span > 0)) return null;
  return ((recent.length - 1) / span) * 1000;
}

function formatHz(hz: number | null): string {
  if (hz == null || !Number.isFinite(hz)) return '—Hz';
  if (hz >= 100) return `${hz.toFixed(0)}Hz`;
  if (hz >= 10) return `${hz.toFixed(1)}Hz`;
  return `${hz.toFixed(2)}Hz`;
}

function formatAgeShort(ageMs: number | null): string {
  if (ageMs == null || !Number.isFinite(ageMs)) return '—';
  if (ageMs < 1000) return `${Math.max(0, Math.round(ageMs))}ms`;
  if (ageMs < 10_000) return `${(ageMs / 1000).toFixed(1)}s`;
  return `${(ageMs / 1000).toFixed(0)}s`;
}

function ageTone(ageMs: number | null): 'ok' | 'warn' | 'bad' | 'none' {
  if (ageMs == null || !Number.isFinite(ageMs)) return 'none';
  if (ageMs <= 250) return 'ok';
  if (ageMs <= 1000) return 'warn';
  return 'bad';
}

/** Foxglove-like Plot: pick channel + numeric field paths as time-series curves. */
export function ChartsPanel({ panelId }: PanelProps) {
  const envelopes = useDataStore((s) => s.envelopes);
  const channels = useDataStore((s) => s.channels);
  const connected = useDataStore((s) => s.connected);
  const markSubscribed = useDataStore((s) => s.markSubscribed);

  const series = usePanelOptsStore(
    (s) => s.charts?.[panelId]?.series ?? EMPTY_SERIES,
  );
  const windowMs = usePanelOptsStore(
    (s) => s.charts?.[panelId]?.windowMs ?? DEFAULT_CHART_WINDOW_MS,
  );
  const addChartSeries = usePanelOptsStore((s) => s.addChartSeries);
  const removeChartSeries = usePanelOptsStore((s) => s.removeChartSeries);
  const setChartSeriesEnabled = usePanelOptsStore((s) => s.setChartSeriesEnabled);
  const setChartSeriesColor = usePanelOptsStore((s) => s.setChartSeriesColor);
  const setChartWindowMs = usePanelOptsStore((s) => s.setChartWindowMs);
  const clearChartSeries = usePanelOptsStore((s) => s.clearChartSeries);

  const frozen = useChartTimeStore((s) => s.frozen);
  const synced = useChartTimeStore((s) => s.synced);
  const sharedView = useChartTimeStore((s) => s.view);
  const freezeNowMs = useChartTimeStore((s) => s.freezeNowMs);
  const setFrozen = useChartTimeStore((s) => s.setFrozen);
  const setSynced = useChartTimeStore((s) => s.setSynced);
  const setSharedView = useChartTimeStore((s) => s.setView);

  const [pickChannel, setPickChannel] = useState('');
  const [pickField, setPickField] = useState('');
  const [fieldFilter, setFieldFilter] = useState('');
  const [fieldsOpen, setFieldsOpen] = useState(false);
  /** Local zoom when Sync is off. */
  const [localZoom, setLocalZoom] = useState<ChartTimeRange | null>(null);
  const [hover, setHover] = useState<HoverHit | null>(null);
  const [hoverCss, setHoverCss] = useState<{ x: number; y: number } | null>(null);

  const zoom = synced ? sharedView : localZoom;
  const setZoom = useCallback(
    (next: ChartTimeRange | null) => {
      if (synced) setSharedView(next);
      else setLocalZoom(next);
    },
    [synced, setSharedView],
  );

  // Keep view continuous when toggling Sync.
  const prevSyncedRef = useRef(synced);
  useEffect(() => {
    if (prevSyncedRef.current === synced) return;
    prevSyncedRef.current = synced;
    if (synced) {
      if (localZoom && !useChartTimeStore.getState().view) {
        setSharedView(localZoom);
      }
    } else {
      setLocalZoom(useChartTimeStore.getState().view);
    }
  }, [synced, localZoom, setSharedView]);

  const wrapRef = useRef<HTMLDivElement>(null);
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const buffersRef = useRef<Map<string, PlotPoint[]>>(new Map());
  /** Sample timestamps (wall) for Hz estimate. */
  const sampleTimesRef = useRef<Map<string, number[]>>(new Map());
  /** Last ingested envelope object per series (not sequence — backend often sends 0). */
  const lastEnvRef = useRef<Map<string, unknown>>(new Map());
  const drawRafRef = useRef<number | null>(null);
  const layoutRef = useRef<PlotLayout | null>(null);
  const dragRef = useRef<{ x0: number; x1: number } | null>(null);
  const hoverRef = useRef<HoverHit | null>(null);
  const [rev, setRev] = useState(0);

  const viewNowMs = frozen && freezeNowMs != null ? freezeNowMs : Date.now();

  const channelNames = useMemo(() => {
    const names = new Set<string>();
    for (const c of channels) names.add(c.name);
    for (const ch of Object.keys(envelopes)) names.add(ch);
    for (const s of series) if (s.channel) names.add(s.channel);
    return Array.from(names).sort((a, b) => a.localeCompare(b));
  }, [channels, envelopes, series]);

  const pickPayload = pickChannel
    ? envelopeJsonPayload(envelopes[pickChannel])
    : undefined;

  const schemaFields = useMemo(() => {
    if (!pickChannel) return [] as string[];
    const ch = channels.find((c) => c.name === pickChannel);
    return ch?.fields ?? [];
  }, [channels, pickChannel]);

  const fieldOptions = useMemo(() => {
    const live = pickPayload ? collectNumericPaths(pickPayload) : [];
    const merged = Array.from(new Set([...schemaFields, ...live]));
    return merged.sort((a, b) => fieldSortKey(a).localeCompare(fieldSortKey(b)));
  }, [schemaFields, pickPayload]);

  const filteredFields = useMemo(() => {
    const q = fieldFilter.trim().toLowerCase();
    if (!q) return fieldOptions;
    return fieldOptions.filter((f) => f.toLowerCase().includes(q));
  }, [fieldOptions, fieldFilter]);

  useEffect(() => {
    if (pickField && fieldOptions.length > 0 && !fieldOptions.includes(pickField)) {
      return;
    }
    if (!pickField && fieldOptions[0]) setPickField(fieldOptions[0]);
  }, [pickField, fieldOptions]);

  useEffect(() => {
    if (!connected) return;
    const need = new Set(series.map((s) => s.channel).filter(Boolean));
    if (pickChannel) need.add(pickChannel);
    for (const ch of need) {
      wsClient.subscribe(ch, 0);
      markSubscribed(ch, 0);
    }
  }, [connected, series, pickChannel, markSubscribed]);

  useEffect(() => {
    const alive = new Set(series.map((s) => s.id));
    for (const id of buffersRef.current.keys()) {
      if (!alive.has(id)) {
        buffersRef.current.delete(id);
        lastEnvRef.current.delete(id);
        sampleTimesRef.current.delete(id);
      }
    }
  }, [series]);

  const scheduleDraw = useCallback(() => {
    if (drawRafRef.current != null) return;
    drawRafRef.current = window.requestAnimationFrame(() => {
      drawRafRef.current = null;
      const canvas = canvasRef.current;
      const wrap = wrapRef.current;
      if (!canvas || !wrap) return;
      const dpr = window.devicePixelRatio || 1;
      const cssW = Math.max(1, wrap.clientWidth);
      const cssH = Math.max(1, wrap.clientHeight);
      const pw = Math.round(cssW * dpr);
      const ph = Math.round(cssH * dpr);
      if (canvas.width !== pw || canvas.height !== ph) {
        canvas.width = pw;
        canvas.height = ph;
      }
      const nowMs = useChartTimeStore.getState().effectiveNowMs();
      const view = useChartTimeStore.getState().synced
        ? useChartTimeStore.getState().view
        : localZoom;
      const drawSeries = series
        .filter((s) => s.enabled)
        .map((s) => ({
          color: s.color,
          points: buffersRef.current.get(s.id) ?? [],
          label: `${s.channel} · ${s.field}`,
        }));
      layoutRef.current = drawTimePlot(canvas, drawSeries, {
        nowMs,
        windowMs,
        t0: view?.t0,
        t1: view?.t1,
        selection: dragRef.current,
        hover: hoverRef.current,
        showMarkers: true,
      });
    });
  }, [series, windowMs, localZoom, synced, sharedView, frozen, freezeNowMs]);

  useEffect(() => {
    const wallNow = Date.now();
    const viewNow = useChartTimeStore.getState().effectiveNowMs();
    let changed = false;
    for (const s of series) {
      const env = envelopes[s.channel];
      const payload = envelopeJsonPayload(env);
      if (!env || payload == null) continue;
      const key = s.id;
      if (lastEnvRef.current.get(key) === env) continue;
      const y = readNumericPath(payload, s.field);
      if (y == null) continue;
      lastEnvRef.current.set(key, env);
      let buf = buffersRef.current.get(key);
      if (!buf) {
        buf = [];
        buffersRef.current.set(key, buf);
      }
      const tMsg = envelopeTimeMs(env.timestamp);
      const t = Math.abs(wallNow - tMsg) <= 2000 ? tMsg : wallNow;
      const last = buf[buf.length - 1];
      if (last && last.t === t && last.y === y) continue;
      buf.push({ t, y });

      let times = sampleTimesRef.current.get(key);
      if (!times) {
        times = [];
        sampleTimesRef.current.set(key, times);
      }
      times.push(t);
      const hzCutoff = wallNow - HZ_WINDOW_MS * 2;
      while (times.length > 0 && times[0]! < hzCutoff) times.shift();
      if (times.length > 500) times.splice(0, times.length - 500);

      const cutoff =
        Math.min(viewNow - windowMs * 1.25, zoom?.t0 ?? viewNow) - 2000;
      // While frozen, keep a longer history so resume still has context.
      const keepCutoff = frozen ? Math.min(cutoff, viewNow - windowMs * 3) : cutoff;
      trimBuffer(buf, keepCutoff);
      changed = true;
    }
    if (changed) {
      setRev((n) => n + 1);
      scheduleDraw();
    }
  }, [envelopes, series, windowMs, zoom, frozen, scheduleDraw]);

  useEffect(() => {
    const id = window.setInterval(() => {
      setRev((n) => n + 1);
      scheduleDraw();
    }, 200);
    return () => window.clearInterval(id);
  }, [scheduleDraw]);

  useEffect(() => {
    const wrap = wrapRef.current;
    if (!wrap || typeof ResizeObserver === 'undefined') {
      scheduleDraw();
      return;
    }
    const ro = new ResizeObserver(() => scheduleDraw());
    ro.observe(wrap);
    scheduleDraw();
    return () => ro.disconnect();
  }, [scheduleDraw]);

  useEffect(
    () => () => {
      if (drawRafRef.current != null) window.cancelAnimationFrame(drawRafRef.current);
    },
    [],
  );

  const latestValues = useMemo(() => {
    const out: Record<string, number | null> = {};
    for (const s of series) {
      const buf = buffersRef.current.get(s.id);
      out[s.id] = buf && buf.length > 0 ? buf[buf.length - 1]!.y : null;
    }
    return out;
  }, [series, rev]);

  const seriesStats = useMemo(() => {
    const wallNow = Date.now();
    const out: Record<string, SeriesLinkStats> = {};
    for (const s of series) {
      const times = sampleTimesRef.current.get(s.id) ?? [];
      const lastT = times.length > 0 ? times[times.length - 1]! : null;
      out[s.id] = {
        hz: estimateHz(times, wallNow),
        ageMs: lastT == null ? null : Math.max(0, wallNow - lastT),
        lastT,
      };
    }
    return out;
  }, [series, rev]);

  const canvasPoint = (clientX: number, clientY: number): { x: number; y: number } => {
    const canvas = canvasRef.current;
    if (!canvas) return { x: 0, y: 0 };
    const rect = canvas.getBoundingClientRect();
    return {
      x: ((clientX - rect.left) / Math.max(1, rect.width)) * canvas.width,
      y: ((clientY - rect.top) / Math.max(1, rect.height)) * canvas.height,
    };
  };

  const updateHoverAt = (clientX: number, clientY: number) => {
    const wrap = wrapRef.current;
    const layout = layoutRef.current;
    const canvas = canvasRef.current;
    if (!wrap || !layout || !canvas) return;
    const { x, y } = canvasPoint(clientX, clientY);
    const drawSeries = series
      .filter((s) => s.enabled)
      .map((s) => ({
        color: s.color,
        points: buffersRef.current.get(s.id) ?? [],
        label: `${s.channel} · ${s.field}`,
      }));
    const hit = findNearestSample(
      drawSeries,
      layout,
      x,
      y,
      useChartTimeStore.getState().effectiveNowMs(),
    );
    hoverRef.current = hit;
    setHover(hit);
    if (hit) {
      const rect = wrap.getBoundingClientRect();
      setHoverCss({ x: clientX - rect.left, y: clientY - rect.top });
    } else {
      setHoverCss(null);
    }
    scheduleDraw();
  };

  const clearHover = () => {
    if (!hoverRef.current && !hover) return;
    hoverRef.current = null;
    setHover(null);
    setHoverCss(null);
    scheduleDraw();
  };

  const onPlotPointerDown = (e: React.PointerEvent<HTMLDivElement>) => {
    if (e.button !== 0) return;
    const layout = layoutRef.current;
    const canvas = canvasRef.current;
    if (!layout || !canvas) return;
    const { x } = canvasPoint(e.clientX, e.clientY);
    if (x < layout.pad.left || x > layout.pad.left + layout.plotW) return;
    clearHover();
    dragRef.current = { x0: x, x1: x };
    e.currentTarget.setPointerCapture(e.pointerId);
    scheduleDraw();
  };

  const onPlotPointerMove = (e: React.PointerEvent<HTMLDivElement>) => {
    const layout = layoutRef.current;
    if (!layout) return;
    if (dragRef.current) {
      const { x } = canvasPoint(e.clientX, e.clientY);
      const clamped = Math.min(
        layout.pad.left + layout.plotW,
        Math.max(layout.pad.left, x),
      );
      dragRef.current = { ...dragRef.current, x1: clamped };
      scheduleDraw();
      return;
    }
    updateHoverAt(e.clientX, e.clientY);
  };

  const onPlotPointerUp = () => {
    const drag = dragRef.current;
    dragRef.current = null;
    const layout = layoutRef.current;
    if (!drag || !layout) {
      scheduleDraw();
      return;
    }
    const dx = Math.abs(drag.x1 - drag.x0);
    if (dx < 8) {
      scheduleDraw();
      return;
    }
    const ta = canvasXToTime(layout, drag.x0);
    const tb = canvasXToTime(layout, drag.x1);
    const t0 = Math.min(ta, tb);
    const t1 = Math.max(ta, tb);
    if (t1 - t0 < 40) {
      scheduleDraw();
      return;
    }
    setZoom({ t0, t1 });
  };

  const onPlotPointerLeave = () => {
    if (dragRef.current) return;
    clearHover();
  };

  const onPlotDoubleClick = () => {
    dragRef.current = null;
    clearHover();
    setZoom(null);
  };

  useEffect(() => {
    const el = wrapRef.current;
    if (!el) return;

    const onWheel = (e: WheelEvent) => {
      e.preventDefault();
      e.stopPropagation();
      const layout = layoutRef.current;
      const canvas = canvasRef.current;
      if (!layout || !canvas) return;

      const rect = canvas.getBoundingClientRect();
      const x = ((e.clientX - rect.left) / Math.max(1, rect.width)) * canvas.width;
      const nowMs = useChartTimeStore.getState().effectiveNowMs();
      const curZoom = useChartTimeStore.getState().synced
        ? useChartTimeStore.getState().view
        : localZoom;
      let t0 = curZoom?.t0 ?? layout.t0;
      let t1 = curZoom?.t1 ?? layout.t1;
      if (!curZoom) {
        t0 = layout.t0;
        t1 = layout.t1;
      }

      const span = Math.max(MIN_ZOOM_MS, t1 - t0);
      const anchorT =
        x >= layout.pad.left && x <= layout.pad.left + layout.plotW
          ? canvasXToTime(layout, x)
          : (t0 + t1) / 2;
      const u = Math.min(1, Math.max(0, (anchorT - t0) / span));

      const factor = Math.exp(e.deltaY * 0.0015);
      const nextSpan = Math.min(MAX_ZOOM_MS, Math.max(MIN_ZOOM_MS, span * factor));

      let nextT0 = anchorT - u * nextSpan;
      let nextT1 = nextT0 + nextSpan;

      if (nextSpan >= windowMs * 0.98 && nextT1 >= nowMs - 80) {
        setZoom(null);
        scheduleDraw();
        return;
      }

      if (nextT1 > nowMs + 250) {
        const shift = nextT1 - (nowMs + 250);
        nextT0 -= shift;
        nextT1 -= shift;
      }

      setZoom({ t0: nextT0, t1: nextT1 });
    };

    el.addEventListener('wheel', onWheel, { passive: false });
    return () => el.removeEventListener('wheel', onWheel);
  }, [localZoom, synced, sharedView, windowMs, scheduleDraw, setZoom]);

  const onAdd = () => {
    if (!pickChannel || !pickField) return;
    const exists = series.some((s) => s.channel === pickChannel && s.field === pickField);
    if (exists) return;
    addChartSeries(panelId, pickChannel, pickField);
  };

  const onAddField = (field: string) => {
    if (!pickChannel || !field) return;
    setPickField(field);
    const exists = series.some((s) => s.channel === pickChannel && s.field === field);
    if (exists) return;
    addChartSeries(panelId, pickChannel, field);
  };

  const liveFieldValue = (field: string): string => {
    if (!pickPayload) return '—';
    const y = readNumericPath(pickPayload, field);
    return y == null ? '—' : formatValue(y);
  };

  return (
    <div className={`chart-panel${frozen ? ' is-frozen' : ''}`}>
      <div className="chart-controls">
        <div className="chart-toolbar">
          <select
            className="chart-select chart-select-channel"
            value={pickChannel}
            title="Channel"
            aria-label="Channel"
            onChange={(e) => {
              setPickChannel(e.target.value);
              setPickField('');
              setFieldFilter('');
            }}
          >
            <option value="">channel…</option>
            {channelNames.map((name) => (
              <option key={name} value={name}>
                {name}
              </option>
            ))}
          </select>
          <select
            className="chart-select chart-select-field"
            value={pickField}
            title="Field"
            aria-label="Field"
            disabled={!pickChannel || fieldOptions.length === 0}
            onChange={(e) => setPickField(e.target.value)}
          >
            <option value="">
              {!pickChannel
                ? 'field…'
                : fieldOptions.length === 0
                  ? 'no fields'
                  : 'field…'}
            </option>
            {fieldOptions.map((f) => (
              <option key={f} value={f}>
                {f}
              </option>
            ))}
          </select>
          <button
            type="button"
            className="chart-panel-btn chart-panel-btn-primary"
            disabled={!pickChannel || !pickField}
            onClick={onAdd}
            title="Add series"
          >
            +
          </button>
          {pickChannel ? (
            <button
              type="button"
              className={`chart-panel-btn chart-fields-toggle ${fieldsOpen ? 'is-open' : ''}`}
              onClick={() => setFieldsOpen((v) => !v)}
              title="Browse proto numeric fields"
            >
              Fields{fieldsOpen ? '▴' : '▾'}
            </button>
          ) : null}
          <select
            className="chart-select chart-select-win"
            value={windowMs}
            title="Time window"
            aria-label="Time window"
            onChange={(e) => setChartWindowMs(panelId, Number(e.target.value))}
          >
            {WINDOW_OPTIONS_MS.map((ms) => (
              <option key={ms} value={ms}>
                {ms / 1000}s
              </option>
            ))}
          </select>
          <button
            type="button"
            className={`chart-panel-btn chart-freeze-btn${frozen ? ' is-active' : ''}`}
            title={frozen ? 'Resume live follow (all Charts)' : 'Freeze timebase (all Charts)'}
            onClick={() => setFrozen(!frozen)}
          >
            {frozen ? '▶' : '❚❚'}
          </button>
          <label
            className={`chart-sync-toggle${synced ? ' is-on' : ''}`}
            title="Sync time axis across Charts panels"
          >
            <input
              type="checkbox"
              checked={synced}
              onChange={(e) => setSynced(e.target.checked)}
            />
            <span>Sync</span>
          </label>
          <button
            type="button"
            className="chart-panel-btn"
            title="Clear samples"
            onClick={() => {
              buffersRef.current.clear();
              lastEnvRef.current.clear();
              sampleTimesRef.current.clear();
              setRev((n) => n + 1);
              scheduleDraw();
            }}
          >
            Clear
          </button>
          {zoom ? (
            <button
              type="button"
              className="chart-panel-btn"
              title="Reset zoom (or double-click plot)"
              onClick={() => {
                dragRef.current = null;
                setZoom(null);
              }}
            >
              Reset
            </button>
          ) : null}
          {series.length > 1 ? (
            <button
              type="button"
              className="chart-panel-btn chart-panel-btn-ghost"
              title="Remove all series"
              onClick={() => {
                clearChartSeries(panelId);
                buffersRef.current.clear();
                lastEnvRef.current.clear();
                sampleTimesRef.current.clear();
                setZoom(null);
                scheduleDraw();
              }}
            >
              ×all
            </button>
          ) : null}
        </div>

        {pickChannel && fieldsOpen ? (
          <div className="chart-field-browser">
            <div className="chart-field-browser-head">
              <span className="muted">
                {filteredFields.length}/{fieldOptions.length}
              </span>
              <input
                className="chart-field-filter"
                value={fieldFilter}
                placeholder="filter…"
                onChange={(e) => setFieldFilter(e.target.value)}
              />
            </div>
            <ul className="chart-field-list">
              {filteredFields.length === 0 ? (
                <li className="muted chart-field-empty">
                  {connected ? 'No matching fields' : 'Connect to load schemas'}
                </li>
              ) : (
                filteredFields.map((f) => {
                  const active = series.some(
                    (s) => s.channel === pickChannel && s.field === f,
                  );
                  return (
                    <li key={f}>
                      <button
                        type="button"
                        className={`chart-field-item ${active ? 'is-active' : ''}`}
                        title={active ? 'Already plotted' : `Add ${f}`}
                        onClick={() => onAddField(f)}
                      >
                        <span className="chart-field-path">{f}</span>
                        <span className="chart-field-live mono">{liveFieldValue(f)}</span>
                      </button>
                    </li>
                  );
                })
              )}
            </ul>
          </div>
        ) : null}

        {series.length > 0 ? (
          <ul className="chart-series-list">
            {series.map((s) => {
              const st = seriesStats[s.id];
              const tone = ageTone(st?.ageMs ?? null);
              return (
                <li key={s.id} className="chart-series-row">
                  <input
                    type="checkbox"
                    checked={s.enabled}
                    onChange={(e) =>
                      setChartSeriesEnabled(panelId, s.id, e.target.checked)
                    }
                    aria-label="toggle series"
                  />
                  <input
                    type="color"
                    className="chart-series-color"
                    value={normalizeHex(s.color)}
                    onChange={(e) => setChartSeriesColor(panelId, s.id, e.target.value)}
                    aria-label="series color"
                  />
                  <span className="chart-series-path" title={`${s.channel}.${s.field}`}>
                    <span className="chart-series-channel">{s.channel}</span>
                    <span className="muted">·</span>
                    <span className="chart-series-field">{s.field}</span>
                  </span>
                  <span
                    className={`chart-series-link mono tone-${tone}`}
                    title="Estimated rate · age since last sample"
                  >
                    {formatHz(st?.hz ?? null)}
                    <span className="muted">·</span>
                    {formatAgeShort(st?.ageMs ?? null)}
                  </span>
                  <span className="chart-series-value mono">
                    {latestValues[s.id] == null ? '—' : formatValue(latestValues[s.id]!)}
                  </span>
                  <button
                    type="button"
                    className="chart-panel-btn chart-series-remove"
                    onClick={() => removeChartSeries(panelId, s.id)}
                    aria-label="remove series"
                  >
                    ×
                  </button>
                </li>
              );
            })}
          </ul>
        ) : null}
      </div>

      <div
        className="chart-panel-plot"
        ref={wrapRef}
        onPointerDown={onPlotPointerDown}
        onPointerMove={onPlotPointerMove}
        onPointerUp={onPlotPointerUp}
        onPointerCancel={onPlotPointerUp}
        onPointerLeave={onPlotPointerLeave}
        onDoubleClick={onPlotDoubleClick}
      >
        <canvas ref={canvasRef} className="chart-panel-canvas" />
        {hover && hoverCss ? (
          <div
            className="chart-hover-tip"
            style={{
              left: Math.min(hoverCss.x + 12, (wrapRef.current?.clientWidth ?? 200) - 8),
              top: Math.max(8, hoverCss.y - 8),
              borderColor: hover.color,
            }}
          >
            <div className="chart-hover-tip-label" style={{ color: hover.color }}>
              {hover.label || 'sample'}
            </div>
            <div className="mono">
              {formatSampleClock(hover.t)}{' '}
              <span className="muted">
                ({formatSampleAge(
                  hover.t,
                  viewNowMs,
                  zoom ? zoom.t1 - zoom.t0 : windowMs,
                )})
              </span>
            </div>
            <div className="mono">y = {formatValue(hover.y)}</div>
          </div>
        ) : null}
        {series.length === 0 ? (
          <div className="chart-panel-empty">
            <strong>Charts</strong>
            <span>Select channel + field, then +</span>
          </div>
        ) : frozen ? (
          <div className="chart-zoom-hint chart-frozen-hint">Frozen</div>
        ) : zoom ? (
          <div className="chart-zoom-hint muted">
            Zoomed{synced ? ' · synced' : ''} · dbl-click reset
          </div>
        ) : null}
      </div>
    </div>
  );
}

function normalizeHex(c: string): string {
  if (/^#[0-9a-fA-F]{6}$/.test(c)) return c;
  return '#4fc3f7';
}

/** Prefer vectors / scalars over large covariance index lists. */
function fieldSortKey(path: string): string {
  const cov = /covariance/i.test(path) ? '2' : '0';
  const idx = /\[\d+\]/.test(path) ? '1' : '0';
  return `${cov}${idx}:${path}`;
}

function formatValue(v: number): string {
  const a = Math.abs(v);
  if (a >= 1000 || (a > 0 && a < 1e-3)) return v.toExponential(2);
  if (a >= 100) return v.toFixed(1);
  if (a >= 1) return v.toFixed(3);
  return v.toFixed(4);
}
