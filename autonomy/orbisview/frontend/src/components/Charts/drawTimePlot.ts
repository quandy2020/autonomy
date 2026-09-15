export type PlotPoint = { t: number; y: number };

export type PlotSeriesDraw = {
  color: string;
  points: PlotPoint[];
  label?: string;
};

export const PLOT_PAD = { left: 52, right: 12, top: 12, bottom: 28 };

export type PlotLayout = {
  pad: typeof PLOT_PAD;
  plotW: number;
  plotH: number;
  t0: number;
  t1: number;
  yMin: number;
  yMax: number;
};

export type HoverHit = {
  seriesIndex: number;
  label: string;
  color: string;
  t: number;
  y: number;
  canvasX: number;
  canvasY: number;
};

function niceStep(span: number, targetTicks: number): number {
  if (!(span > 0) || !Number.isFinite(span)) return 1;
  const raw = span / Math.max(1, targetTicks);
  const pow = 10 ** Math.floor(Math.log10(raw));
  const n = raw / pow;
  const nice = n <= 1 ? 1 : n <= 2 ? 2 : n <= 5 ? 5 : 10;
  return nice * pow;
}

/** Prefer 1/2/5·10ⁿ steps down to 1 ms for time axes. */
function niceTimeStepMs(spanMs: number, targetTicks: number): number {
  const step = niceStep(Math.max(1, spanMs), targetTicks);
  return Math.max(1, step);
}

function formatTick(v: number): string {
  if (!Number.isFinite(v)) return '';
  const a = Math.abs(v);
  if (a >= 1000 || (a > 0 && a < 0.01)) return v.toExponential(1);
  if (a >= 100) return v.toFixed(0);
  if (a >= 10) return v.toFixed(1);
  return v.toFixed(2);
}

/**
 * Relative time label vs now. Precision follows visible span so zoomed-in
 * windows still distinguish ticks (ms when needed).
 */
export function formatRelativeTimeLabel(ageMs: number, spanMs: number): string {
  if (!Number.isFinite(ageMs)) return '';
  const abs = Math.abs(ageMs);
  const near = Math.max(1, Math.min(8, spanMs * 0.004));
  if (abs <= near) return 'now';

  const sign = ageMs >= 0 ? '-' : '+';

  // Very tight zoom: integer milliseconds.
  if (spanMs <= 800) {
    return `${sign}${Math.round(abs)}ms`;
  }
  // Sub-second / few-second windows: always show ms as fractional seconds.
  if (spanMs <= 5_000) {
    return `${sign}${(abs / 1000).toFixed(3)}s`;
  }
  if (spanMs <= 20_000) {
    return `${sign}${(abs / 1000).toFixed(2)}s`;
  }
  if (spanMs <= 120_000) {
    return `${sign}${(abs / 1000).toFixed(1)}s`;
  }
  return `${sign}${(abs / 1000).toFixed(0)}s`;
}

export function yBoundsForRange(
  series: PlotSeriesDraw[],
  t0: number,
  t1: number,
): { yMin: number; yMax: number } {
  let yMin = Infinity;
  let yMax = -Infinity;
  for (const s of series) {
    for (const p of s.points) {
      if (p.t < t0 || p.t > t1) continue;
      if (p.y < yMin) yMin = p.y;
      if (p.y > yMax) yMax = p.y;
    }
  }
  if (!Number.isFinite(yMin) || !Number.isFinite(yMax)) {
    yMin = -1;
    yMax = 1;
  }
  if (yMax - yMin < 1e-9) {
    const pad = Math.max(1, Math.abs(yMin) * 0.1);
    yMin -= pad;
    yMax += pad;
  }
  const yPad = (yMax - yMin) * 0.08;
  return { yMin: yMin - yPad, yMax: yMax + yPad };
}

function pointRadius(plotW: number, nPts: number): number {
  if (nPts <= 0) return 2.5;
  const spacing = plotW / Math.max(1, nPts);
  if (spacing >= 14) return 3.2;
  if (spacing >= 8) return 2.6;
  if (spacing >= 4) return 2.1;
  return 1.6;
}

/** Draw multi-series point-line plot (samples connected; markers = message times). */
export function drawTimePlot(
  canvas: HTMLCanvasElement,
  series: PlotSeriesDraw[],
  opts: {
    nowMs: number;
    windowMs: number;
    t0?: number;
    t1?: number;
    selection?: { x0: number; x1: number } | null;
    hover?: HoverHit | null;
    showMarkers?: boolean;
    pad?: { left: number; right: number; top: number; bottom: number };
  },
): PlotLayout {
  const ctx = canvas.getContext('2d');
  const w = canvas.width;
  const h = canvas.height;
  const pad = opts.pad ?? PLOT_PAD;
  const plotW = Math.max(1, w - pad.left - pad.right);
  const plotH = Math.max(1, h - pad.top - pad.bottom);
  const t1 = opts.t1 ?? opts.nowMs;
  const t0 = opts.t0 ?? t1 - Math.max(100, opts.windowMs);
  const { yMin, yMax } = yBoundsForRange(series, t0, t1);
  const layout: PlotLayout = { pad, plotW, plotH, t0, t1, yMin, yMax };
  const showMarkers = opts.showMarkers !== false;

  if (!ctx) return layout;

  ctx.fillStyle = '#0a0e12';
  ctx.fillRect(0, 0, w, h);

  const xOf = (t: number) => pad.left + ((t - t0) / Math.max(1e-6, t1 - t0)) * plotW;
  const yOf = (y: number) => pad.top + (1 - (y - yMin) / Math.max(1e-9, yMax - yMin)) * plotH;

  ctx.strokeStyle = '#1e2a38';
  ctx.lineWidth = 1;
  ctx.fillStyle = '#6b7c8f';
  ctx.font = `${Math.max(10, Math.round(11 * (window.devicePixelRatio || 1)))}px ui-monospace, monospace`;
  ctx.textAlign = 'right';
  ctx.textBaseline = 'middle';

  const yStep = niceStep(yMax - yMin, 5);
  const yStart = Math.ceil(yMin / yStep) * yStep;
  for (let y = yStart; y <= yMax + yStep * 0.01; y += yStep) {
    const py = yOf(y);
    ctx.beginPath();
    ctx.moveTo(pad.left, py);
    ctx.lineTo(pad.left + plotW, py);
    ctx.stroke();
    ctx.fillText(formatTick(y), pad.left - 6, py);
  }

  ctx.textAlign = 'center';
  ctx.textBaseline = 'top';
  const spanMs = Math.max(1, t1 - t0);
  const tStep = niceTimeStepMs(spanMs, 6);
  const tStart = Math.ceil(t0 / tStep) * tStep;
  for (let t = tStart; t <= t1 + tStep * 0.01; t += tStep) {
    const px = xOf(t);
    ctx.beginPath();
    ctx.moveTo(px, pad.top);
    ctx.lineTo(px, pad.top + plotH);
    ctx.stroke();
    const ageMs = opts.nowMs - t;
    const label = formatRelativeTimeLabel(ageMs, spanMs);
    ctx.fillText(label, px, pad.top + plotH + 6);
  }

  ctx.strokeStyle = '#2a3a4c';
  ctx.strokeRect(pad.left + 0.5, pad.top + 0.5, plotW - 1, plotH - 1);

  ctx.save();
  ctx.beginPath();
  ctx.rect(pad.left, pad.top, plotW, plotH);
  ctx.clip();

  for (const s of series) {
    const pts = s.points.filter((p) => p.t >= t0 && p.t <= t1);
    if (pts.length === 0) continue;

    // Connect samples with lines (gaps show silence between messages).
    if (pts.length >= 2) {
      ctx.strokeStyle = s.color;
      ctx.lineWidth = 1.6;
      ctx.lineJoin = 'round';
      ctx.lineCap = 'round';
      ctx.beginPath();
      pts.forEach((p, i) => {
        const x = xOf(p.t);
        const y = yOf(p.y);
        if (i === 0) ctx.moveTo(x, y);
        else ctx.lineTo(x, y);
      });
      ctx.stroke();
    }

    if (showMarkers) {
      const r = pointRadius(plotW, pts.length);
      for (const p of pts) {
        const x = xOf(p.t);
        const y = yOf(p.y);
        ctx.fillStyle = s.color;
        ctx.beginPath();
        ctx.arc(x, y, r, 0, Math.PI * 2);
        ctx.fill();
        ctx.strokeStyle = '#0a0e12';
        ctx.lineWidth = 1;
        ctx.stroke();
      }
    } else if (pts.length === 1) {
      ctx.fillStyle = s.color;
      ctx.beginPath();
      ctx.arc(xOf(pts[0]!.t), yOf(pts[0]!.y), 2.5, 0, Math.PI * 2);
      ctx.fill();
    }
  }

  if (opts.hover) {
    const hov = opts.hover;
    ctx.strokeStyle = 'rgba(227, 237, 247, 0.35)';
    ctx.lineWidth = 1;
    ctx.setLineDash([4, 4]);
    ctx.beginPath();
    ctx.moveTo(hov.canvasX, pad.top);
    ctx.lineTo(hov.canvasX, pad.top + plotH);
    ctx.stroke();
    ctx.setLineDash([]);
    ctx.fillStyle = hov.color;
    ctx.beginPath();
    ctx.arc(hov.canvasX, hov.canvasY, 5, 0, Math.PI * 2);
    ctx.fill();
    ctx.strokeStyle = '#e3edf7';
    ctx.lineWidth = 1.5;
    ctx.stroke();
  }

  if (opts.selection) {
    const x0 = Math.min(opts.selection.x0, opts.selection.x1);
    const x1 = Math.max(opts.selection.x0, opts.selection.x1);
    ctx.fillStyle = 'rgba(79, 195, 247, 0.18)';
    ctx.fillRect(x0, pad.top, Math.max(1, x1 - x0), plotH);
    ctx.strokeStyle = 'rgba(79, 195, 247, 0.85)';
    ctx.lineWidth = 1;
    ctx.strokeRect(x0 + 0.5, pad.top + 0.5, Math.max(1, x1 - x0 - 1), plotH - 1);
  }
  ctx.restore();

  return layout;
}

export function canvasXToTime(layout: PlotLayout, canvasX: number): number {
  const u = (canvasX - layout.pad.left) / Math.max(1, layout.plotW);
  return layout.t0 + Math.min(1, Math.max(0, u)) * (layout.t1 - layout.t0);
}

export function findNearestSample(
  series: PlotSeriesDraw[],
  layout: PlotLayout,
  canvasX: number,
  canvasY: number,
  nowMs: number,
  maxDistPx = 18,
): HoverHit | null {
  const xOf = (t: number) =>
    layout.pad.left + ((t - layout.t0) / Math.max(1e-6, layout.t1 - layout.t0)) * layout.plotW;
  const yOf = (y: number) =>
    layout.pad.top +
    (1 - (y - layout.yMin) / Math.max(1e-9, layout.yMax - layout.yMin)) * layout.plotH;

  let best: HoverHit | null = null;
  let bestD = maxDistPx * maxDistPx;

  series.forEach((s, seriesIndex) => {
    for (const p of s.points) {
      if (p.t < layout.t0 || p.t > layout.t1) continue;
      const cx = xOf(p.t);
      const cy = yOf(p.y);
      const dx = cx - canvasX;
      const dy = cy - canvasY;
      const d = dx * dx + dy * dy;
      if (d < bestD) {
        bestD = d;
        best = {
          seriesIndex,
          label: s.label ?? '',
          color: s.color,
          t: p.t,
          y: p.y,
          canvasX: cx,
          canvasY: cy,
        };
      }
    }
  });

  // Prefer time-proximity along X if nothing close in 2D (sparse Y).
  if (!best) {
    const t = canvasXToTime(layout, canvasX);
    let bestDt = Infinity;
    series.forEach((s, seriesIndex) => {
      for (const p of s.points) {
        if (p.t < layout.t0 || p.t > layout.t1) continue;
        const dt = Math.abs(p.t - t);
        if (dt < bestDt) {
          bestDt = dt;
          const cx = xOf(p.t);
          const cy = yOf(p.y);
          if (Math.abs(cx - canvasX) <= maxDistPx * 1.5) {
            best = {
              seriesIndex,
              label: s.label ?? '',
              color: s.color,
              t: p.t,
              y: p.y,
              canvasX: cx,
              canvasY: cy,
            };
          }
        }
      }
    });
  }

  void nowMs;
  return best;
}

export function formatSampleClock(tMs: number): string {
  const d = new Date(tMs);
  const hh = String(d.getHours()).padStart(2, '0');
  const mm = String(d.getMinutes()).padStart(2, '0');
  const ss = String(d.getSeconds()).padStart(2, '0');
  const ms = String(d.getMilliseconds()).padStart(3, '0');
  return `${hh}:${mm}:${ss}.${ms}`;
}

export function formatSampleAge(tMs: number, nowMs: number, spanMs = 5_000): string {
  return formatRelativeTimeLabel(nowMs - tMs, spanMs);
}
